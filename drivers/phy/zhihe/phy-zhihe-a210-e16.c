// SPDX-License-Identifier: GPL-2.0-only
/*
 * A210 Synopsys E16 shared PCIe/SATA PHY.
 * Based on the Zhihe vendor driver, Copyright (C) 2025 Zhihe.
 */
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/reset.h>

#define E16_GLOBAL_CTRL		0x000
#define E16_PORT_SOURCE		0x004
#define E16_PROTOCOL		0x008
#define E16_SRAM_STATUS		0x00c
#define E16_SRAM_INIT_DONE	(BIT(8) | BIT(24))
#define E16_RES_TUNE		0x048
#define E16_PCIE_OVERRIDE	0x108
#define E16_GLOBAL_OVERRIDE	(BIT(16) | BIT(20))
#define E16_SPLIT_PCIE		BIT(8)
#define E16_SATA_ENABLE		BIT(0)
#define E16_SATA_SOURCE		(BIT(8) | BIT(12))
#define E16_SATA_PROTOCOL	(BIT(9) | BIT(13))
#define E16_RTUNE_REQUEST	BIT(0)
#define E16_RES_ACK_IN		BIT(16)
#define E16_RTUNE_ACK		BIT(4)
#define E16_PCIE_OVERRIDE_ENABLE	(BIT(0) | BIT(4))

/* Analog settings from the vendor's 100 MHz reference-clock profile. */
#define E16_MPLL_CTRL		0x0048
#define E16_MPLL_100MHZ		0x0350
#define E16_RX_EQ_CTRL(lane)	(0x4008 + (lane) * 0x400)
#define E16_RX_EQ_DEFAULT	0x01f8
#define E16_RX_PLL_CTRL(lane)	(0xc020 + (lane) * 0x400)
#define E16_RX_PLL_DEFAULT	0x000f
#define E16_LANES_PER_PHY	2

struct a210_e16 {
	void __iomem *sys;
	void __iomem *analog[2];
	struct clk_bulk_data clocks[2];
	struct reset_control *apb_reset;
	struct reset_control *phy_reset;
	u32 global_ctrl;
	bool sata;
};

static int a210_e16_write(struct phy *phy, void __iomem *base,
			  unsigned int bank, u32 reg, u32 value)
{
	u32 readback;

	writel(value, base + reg);
	readback = readl(base + reg);
	if (readback != value) {
		dev_err(&phy->dev, "PHY%u CR %#x: wrote %#x, read %#x\n",
			bank, reg, value, readback);
		return -EIO;
	}
	return 0;
}

static int a210_e16_init(struct phy *phy)
{
	struct a210_e16 *e16 = phy_get_drvdata(phy);
	u32 val;
	int ret, bank, lane;

	ret = clk_bulk_prepare_enable(ARRAY_SIZE(e16->clocks), e16->clocks);
	if (ret)
		return ret;

	if (clk_get_rate(e16->clocks[1].clk) != 100000000) {
		ret = -EINVAL;
		goto disable_clocks;
	}

	ret = reset_control_deassert(e16->apb_reset);
	if (ret)
		goto disable_clocks;
	ret = reset_control_assert(e16->phy_reset);
	if (ret)
		goto assert_apb;
	usleep_range(100, 200);

	writel(e16->global_ctrl, e16->sys + E16_GLOBAL_CTRL);
	writel(e16->sata ? E16_SATA_SOURCE : 0,
	       e16->sys + E16_PORT_SOURCE);
	writel(e16->sata ? E16_SATA_PROTOCOL : 0,
	       e16->sys + E16_PROTOCOL);
	writel(E16_PCIE_OVERRIDE_ENABLE, e16->sys + E16_PCIE_OVERRIDE);
	/* Request calibration only after the CR interface has been set up. */
	writel(E16_RES_ACK_IN, e16->sys + E16_RES_TUNE);
	ret = reset_control_deassert(e16->phy_reset);
	if (ret)
		goto assert_phy;
	ret = readl_poll_timeout(e16->sys + E16_SRAM_STATUS, val,
				(val & E16_SRAM_INIT_DONE) == E16_SRAM_INIT_DONE,
				10, 100000);
	if (ret) {
		dev_err(&phy->dev, "PHY initialization timed out: %#x\n", val);
		goto assert_phy;
	}

	/* Analog CR writes made under reset do not survive its release. */
	for (bank = 0; bank < ARRAY_SIZE(e16->analog); bank++) {
		void __iomem *base = e16->analog[bank];

		ret = a210_e16_write(phy, base, bank, E16_MPLL_CTRL,
				     E16_MPLL_100MHZ);
		if (ret)
			goto assert_phy;
		for (lane = 0; lane < E16_LANES_PER_PHY; lane++) {
			ret = a210_e16_write(phy, base, bank,
					     E16_RX_PLL_CTRL(lane),
					     E16_RX_PLL_DEFAULT);
			if (ret)
				goto assert_phy;
			ret = a210_e16_write(phy, base, bank,
					     E16_RX_EQ_CTRL(lane),
					     E16_RX_EQ_DEFAULT);
			if (ret)
				goto assert_phy;
		}
	}

	writel(E16_RES_ACK_IN | E16_RTUNE_REQUEST,
	       e16->sys + E16_RES_TUNE);
	ret = readl_poll_timeout(e16->sys + E16_RES_TUNE, val,
				val & E16_RTUNE_ACK, 10, 100000);
	if (ret) {
		dev_err(&phy->dev, "resistance calibration timed out\n");
		goto assert_phy;
	}

	/* Allow the analog PLLs to settle before either host starts LTSSM. */
	msleep(200);
	return 0;

assert_phy:
	reset_control_assert(e16->phy_reset);
assert_apb:
	reset_control_assert(e16->apb_reset);
disable_clocks:
	clk_bulk_disable_unprepare(ARRAY_SIZE(e16->clocks), e16->clocks);
	return ret;
}

static int a210_e16_exit(struct phy *phy)
{
	struct a210_e16 *e16 = phy_get_drvdata(phy);

	reset_control_assert(e16->phy_reset);
	reset_control_assert(e16->apb_reset);
	clk_bulk_disable_unprepare(ARRAY_SIZE(e16->clocks), e16->clocks);
	return 0;
}

static int a210_e16_set_mode(struct phy *phy, enum phy_mode mode,
			   int submode)
{
	struct a210_e16 *e16 = phy_get_drvdata(phy);

	/* All consumers share the board's fixed lane routing. */
	if (mode == PHY_MODE_PCIE || (mode == PHY_MODE_SATA && e16->sata))
		return 0;
	return -EOPNOTSUPP;
}

static const struct phy_ops a210_e16_ops = {
	.init = a210_e16_init,
	.exit = a210_e16_exit,
	.set_mode = a210_e16_set_mode,
	.owner = THIS_MODULE,
};

static int a210_e16_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct phy_provider *provider;
	struct a210_e16 *e16;
	struct phy *phy;
	u32 lanes[4];
	int ret;

	e16 = devm_kzalloc(dev, sizeof(*e16), GFP_KERNEL);
	if (!e16)
		return -ENOMEM;
	e16->sys = devm_platform_ioremap_resource_byname(pdev, "sysreg");
	if (IS_ERR(e16->sys))
		return PTR_ERR(e16->sys);
	e16->analog[0] = devm_platform_ioremap_resource_byname(pdev, "phy0");
	if (IS_ERR(e16->analog[0]))
		return PTR_ERR(e16->analog[0]);
	e16->analog[1] = devm_platform_ioremap_resource_byname(pdev, "phy1");
	if (IS_ERR(e16->analog[1]))
		return PTR_ERR(e16->analog[1]);

	ret = device_property_read_u32_array(dev, "data-lanes", lanes, 4);
	if (ret)
		return dev_err_probe(dev, ret, "missing lane routing\n");
	e16->global_ctrl = E16_GLOBAL_OVERRIDE;
	if (lanes[0] != 1)
		return -EINVAL;
	if (lanes[1] == 1 && lanes[2] == 1 && lanes[3] == 1) {
		/* One four-lane PCIe port. */
	} else if (lanes[1] == 1 && lanes[2] == 2 && lanes[3] == 0) {
		e16->global_ctrl |= E16_SPLIT_PCIE;
	} else if ((lanes[1] == 1 || lanes[1] == 2) &&
		   lanes[2] == 3 && lanes[3] == 4) {
		e16->sata = true;
		e16->global_ctrl |= E16_SATA_ENABLE;
		if (lanes[1] == 2)
			e16->global_ctrl |= E16_SPLIT_PCIE;
	} else {
		return dev_err_probe(dev, -EINVAL, "invalid lane routing\n");
	}

	e16->clocks[0].id = "apb";
	e16->clocks[1].id = "ref";
	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(e16->clocks), e16->clocks);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get clocks\n");
	e16->apb_reset = devm_reset_control_get_exclusive(dev, "apb");
	if (IS_ERR(e16->apb_reset))
		return dev_err_probe(dev, PTR_ERR(e16->apb_reset),
				     "failed to get APB reset\n");
	e16->phy_reset = devm_reset_control_get_exclusive(dev, "phy");
	if (IS_ERR(e16->phy_reset))
		return dev_err_probe(dev, PTR_ERR(e16->phy_reset),
				     "failed to get PHY reset\n");

	phy = devm_phy_create(dev, NULL, &a210_e16_ops);
	if (IS_ERR(phy))
		return PTR_ERR(phy);
	phy_set_drvdata(phy, e16);
	provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	return PTR_ERR_OR_ZERO(provider);
}

static const struct of_device_id a210_e16_match[] = {
	{ .compatible = "zhihe,a210-e16-phy" },
	{ }
};
MODULE_DEVICE_TABLE(of, a210_e16_match);

static struct platform_driver a210_e16_driver = {
	.probe = a210_e16_probe,
	.driver = {
		.name = "a210-e16-phy",
		.of_match_table = a210_e16_match,
	},
};
module_platform_driver(a210_e16_driver);

MODULE_DESCRIPTION("Zhihe A210 E16 PCIe/SATA PHY driver");
MODULE_LICENSE("GPL");
