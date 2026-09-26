// SPDX-License-Identifier: GPL-2.0-only
/*
 * Zhihe A210 DesignWare PCIe host controller.
 * Based on the Zhihe vendor driver, Copyright (C) 2025 Zhihe.
 */
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/sizes.h>

#include "pcie-designware.h"

#define A210_PCIE_DBI2_OFFSET	SZ_1M

#define A210_PCIE_CONTROL	0x000
#define A210_PCIE_TYPE		GENMASK(3, 0)
#define A210_PCIE_ROOT_PORT	4
#define A210_PCIE_TYPE_OVERRIDE	BIT(4)
#define A210_PCIE_LTSSM_ENABLE	BIT(8)
#define A210_PCIE_LINK_STATUS	0x034
#define A210_PCIE_RDLH_UP	BIT(0)
#define A210_PCIE_SMLH_UP	BIT(4)

struct a210_pcie {
	struct dw_pcie pci;
	void __iomem *app;
	struct phy *phy;
	struct regulator_bulk_data supplies[3];
	unsigned int num_supplies;
};

static struct a210_pcie *to_a210_pcie(struct dw_pcie *pci)
{
	return container_of(pci, struct a210_pcie, pci);
}

static void a210_pcie_update(struct a210_pcie *pcie, u32 mask, u32 value)
{
	u32 reg = readl(pcie->app + A210_PCIE_CONTROL);

	writel((reg & ~mask) | value, pcie->app + A210_PCIE_CONTROL);
}

static bool a210_pcie_link_up(struct dw_pcie *pci)
{
	struct a210_pcie *pcie = to_a210_pcie(pci);
	u32 mask = A210_PCIE_RDLH_UP | A210_PCIE_SMLH_UP;

	return (readl(pcie->app + A210_PCIE_LINK_STATUS) & mask) == mask;
}

static int a210_pcie_start_link(struct dw_pcie *pci)
{
	struct a210_pcie *pcie = to_a210_pcie(pci);
	u16 cap, status;
	u32 val;

	/* Endpoint power and reference clock must precede PERST# release. */
	msleep(100);
	gpiod_set_value_cansleep(pci->pe_rst, 0);
	usleep_range(1000, 2000);
	a210_pcie_update(pcie, A210_PCIE_LTSSM_ENABLE,
			 A210_PCIE_LTSSM_ENABLE);
	if (pci->max_link_speed <= 1)
		return 0;

	/* The common host path handles an absent or late endpoint. */
	if (dw_pcie_wait_for_link(pci))
		return 0;

	cap = dw_pcie_find_capability(pci, PCI_CAP_ID_EXP);
	status = dw_pcie_readw_dbi(pci, cap + PCI_EXP_LNKSTA);
	if (FIELD_GET(PCI_EXP_LNKSTA_CLS, status) < pci->max_link_speed) {
		/* A210 needs the speed-change request after reaching L0. */
		val = dw_pcie_readl_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL);
		val |= PORT_LOGIC_SPEED_CHANGE;
		dw_pcie_writel_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL, val);
	}
	return 0;
}

static void a210_pcie_stop_link(struct dw_pcie *pci)
{
	a210_pcie_update(to_a210_pcie(pci), A210_PCIE_LTSSM_ENABLE, 0);
	gpiod_set_value_cansleep(pci->pe_rst, 1);
}

static const struct dw_pcie_ops a210_pcie_ops = {
	.link_up = a210_pcie_link_up,
	.start_link = a210_pcie_start_link,
	.stop_link = a210_pcie_stop_link,
};

static void a210_pcie_disable_aspm(struct dw_pcie *pci)
{
	u16 cap = dw_pcie_find_capability(pci, PCI_CAP_ID_EXP);
	u32 lnkcap;
	u16 lnkctl;

	if (!cap)
		return;

	/* The A210 PHY wakeup path is not configured for ASPM yet. */
	dw_pcie_dbi_ro_wr_en(pci);
	lnkcap = dw_pcie_readl_dbi(pci, cap + PCI_EXP_LNKCAP);
	lnkcap &= ~PCI_EXP_LNKCAP_ASPMS;
	dw_pcie_writel_dbi(pci, cap + PCI_EXP_LNKCAP, lnkcap);
	lnkctl = dw_pcie_readw_dbi(pci, cap + PCI_EXP_LNKCTL);
	lnkctl &= ~PCI_EXP_LNKCTL_ASPMC;
	dw_pcie_writew_dbi(pci, cap + PCI_EXP_LNKCTL, lnkctl);
	dw_pcie_dbi_ro_wr_dis(pci);
}

static int a210_pcie_host_init(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct a210_pcie *pcie = to_a210_pcie(pci);
	int ret;

	ret = regulator_bulk_enable(pcie->num_supplies,
				    pcie->supplies);
	if (ret)
		return ret;
	ret = clk_bulk_prepare_enable(DW_PCIE_NUM_APP_CLKS, pci->app_clks);
	if (ret)
		goto disable_supplies;
	ret = clk_bulk_prepare_enable(DW_PCIE_NUM_CORE_CLKS, pci->core_clks);
	if (ret)
		goto disable_app_clocks;
	ret = reset_control_bulk_deassert(DW_PCIE_NUM_APP_RSTS,
					 pci->app_rsts);
	if (ret)
		goto disable_core_clocks;

	a210_pcie_update(pcie, A210_PCIE_LTSSM_ENABLE, 0);
	a210_pcie_update(pcie, A210_PCIE_TYPE | A210_PCIE_TYPE_OVERRIDE,
			 FIELD_PREP(A210_PCIE_TYPE, A210_PCIE_ROOT_PORT) |
			 A210_PCIE_TYPE_OVERRIDE);
	ret = phy_set_mode(pcie->phy, PHY_MODE_PCIE);
	if (ret)
		goto assert_app_resets;
	ret = phy_init(pcie->phy);
	if (ret)
		goto assert_app_resets;
	ret = phy_power_on(pcie->phy);
	if (ret)
		goto exit_phy;
	ret = reset_control_bulk_deassert(DW_PCIE_NUM_CORE_RSTS,
					 pci->core_rsts);
	if (!ret) {
		/* BAR masks are accessed through the AXI DBI CS2 window. */
		pci->dbi_base2 = pci->dbi_base + A210_PCIE_DBI2_OFFSET;
		dw_pcie_writel_dbi2(pci, PCI_BASE_ADDRESS_0, 0);
		dw_pcie_writel_dbi2(pci, PCI_BASE_ADDRESS_1, 0);
		a210_pcie_disable_aspm(pci);
		return 0;
	}
	phy_power_off(pcie->phy);
exit_phy:
	phy_exit(pcie->phy);
assert_app_resets:
	reset_control_bulk_assert(DW_PCIE_NUM_APP_RSTS, pci->app_rsts);
disable_core_clocks:
	clk_bulk_disable_unprepare(DW_PCIE_NUM_CORE_CLKS, pci->core_clks);
disable_app_clocks:
	clk_bulk_disable_unprepare(DW_PCIE_NUM_APP_CLKS, pci->app_clks);
disable_supplies:
	regulator_bulk_disable(pcie->num_supplies, pcie->supplies);
	return ret;
}

static void a210_pcie_host_deinit(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct a210_pcie *pcie = to_a210_pcie(pci);

	a210_pcie_stop_link(pci);
	reset_control_bulk_assert(DW_PCIE_NUM_CORE_RSTS, pci->core_rsts);
	phy_power_off(pcie->phy);
	phy_exit(pcie->phy);
	reset_control_bulk_assert(DW_PCIE_NUM_APP_RSTS, pci->app_rsts);
	clk_bulk_disable_unprepare(DW_PCIE_NUM_CORE_CLKS, pci->core_clks);
	clk_bulk_disable_unprepare(DW_PCIE_NUM_APP_CLKS, pci->app_clks);
	regulator_bulk_disable(pcie->num_supplies, pcie->supplies);
}

static const struct dw_pcie_host_ops a210_host_ops = {
	.init = a210_pcie_host_init,
	.deinit = a210_pcie_host_deinit,
};

static int a210_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	static const char * const supply_names[] = {
		"vpcie3v3", "vpcie1v5", "vpcie12v",
	};
	struct a210_pcie *pcie;
	int i, ret;

	pcie = devm_kzalloc(dev, sizeof(*pcie), GFP_KERNEL);
	if (!pcie)
		return -ENOMEM;
	pcie->pci.dev = dev;
	pcie->pci.ops = &a210_pcie_ops;
	pcie->pci.pp.ops = &a210_host_ops;
	dw_pcie_cap_set(&pcie->pci, REQ_RES);
	platform_set_drvdata(pdev, pcie);
	pcie->app = devm_platform_ioremap_resource_byname(pdev, "app");
	if (IS_ERR(pcie->app))
		return PTR_ERR(pcie->app);
	pcie->phy = devm_phy_get(dev, "pcie");
	if (IS_ERR(pcie->phy))
		return dev_err_probe(dev, PTR_ERR(pcie->phy),
				     "failed to get PHY\n");
	for (i = 0; i < ARRAY_SIZE(supply_names); i++) {
		struct regulator *supply;

		supply = devm_regulator_get_optional(dev, supply_names[i]);
		if (IS_ERR(supply)) {
			if (PTR_ERR(supply) == -ENODEV)
				continue;
			return dev_err_probe(dev, PTR_ERR(supply),
					     "failed to get %s supply\n",
					     supply_names[i]);
		}
		pcie->supplies[pcie->num_supplies].supply = supply_names[i];
		pcie->supplies[pcie->num_supplies++].consumer = supply;
	}
	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;
	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0)
		return ret;
	ret = dw_pcie_host_init(&pcie->pci.pp);
	if (ret)
		pm_runtime_put_sync(dev);
	return ret;
}

static void a210_pcie_remove(struct platform_device *pdev)
{
	struct a210_pcie *pcie = platform_get_drvdata(pdev);

	dw_pcie_host_deinit(&pcie->pci.pp);
	pm_runtime_put_sync(&pdev->dev);
}

static const struct of_device_id a210_pcie_match[] = {
	{ .compatible = "zhihe,a210-pcie" },
	{ }
};
MODULE_DEVICE_TABLE(of, a210_pcie_match);

static struct platform_driver a210_pcie_driver = {
	.probe = a210_pcie_probe,
	.remove = a210_pcie_remove,
	.driver = {
		.name = "a210-pcie",
		.of_match_table = a210_pcie_match,
	},
};
module_platform_driver(a210_pcie_driver);

MODULE_DESCRIPTION("Zhihe A210 DesignWare PCIe host controller");
MODULE_LICENSE("GPL");
