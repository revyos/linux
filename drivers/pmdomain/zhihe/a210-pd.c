// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2025 Zhihe Computing Limited.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_clk.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/slab.h>

#include "a210-pd.h"

#define A210_CCU_DELAY			0x00
#define A210_CCU_CONTROL		0x04
#define  A210_CCU_GATING_ENABLE		0x04
#define  A210_CCU_SCALING_ENABLE	0x02
#define A210_CCU_LOW_POWER_RATIO		0x18

#define A210_BPC_BYPASS			0x000
#define A210_BPC_POWER_CONTROL		0x004
#define A210_BPC_RESET_CONTROL		0x13c

#define A210_PCA_CONTROL		0x20

#define A210_PCU_REQUEST		0x08
#define A210_PCU_LOW_POWER_STATE	0x0c
#define A210_PCU_IRQ_ENABLE		0x24
#define A210_PCU_IRQ_CLEAR		0x28
#define A210_PCU_IRQ_STATUS		0x2c

#define A210_IOPMP_GROUPS		64
#define A210_IOPMP_CONFIG(n)		((n) * 4)
#define A210_IOPMP_ADDRESS(n)		(0x800 + (n) * 0x20)
#define A210_IOPMP_DEVICE_SHIFT		14
#define A210_IOPMP_TOR_RWX		0x0f0f
#define A210_IOPMP_LIMIT_HIGH		0x20
#define A210_IOPMP_GPU_WRAPPER		0x33
#define A210_IOPMP_GPU			0x34

#define A210_VO_PATH_CONTROL		0x048
#define A210_VO_CLOCK_ENABLE		0x200
#define A210_VO_PIXEL_DIVIDER		0x204
#define A210_VO_BUS_DIVIDER		0x208
#define A210_VO_PIXEL_SYNC		(BIT(8) | BIT(18) | BIT(28))
#define A210_IOPMP_ADDR_WORDS		8
#define A210_VO_BUS_SYNC			(BIT(4) | BIT(12) | BIT(20))

struct a210_vo_context {
	void __iomem *base;
	void __iomem *iopmp;
	u32 iopmp_cfg[A210_IOPMP_GROUPS];
	u32 iopmp_addr[A210_IOPMP_GROUPS][A210_IOPMP_ADDR_WORDS];
	u32 path;
	u32 gates;
	u32 pixel_div;
	u32 bus_div;
	bool valid;
};

static void a210_vo_save(struct a210_vo_context *vo)
{
	unsigned int group, word;

	if (!vo)
		return;
	vo->path = readl(vo->base + A210_VO_PATH_CONTROL);
	vo->gates = readl(vo->base + A210_VO_CLOCK_ENABLE);
	vo->pixel_div = readl(vo->base + A210_VO_PIXEL_DIVIDER);
	vo->bus_div = readl(vo->base + A210_VO_BUS_DIVIDER);
	if (vo->iopmp) {
		for (group = 0; group < A210_IOPMP_GROUPS; group++) {
			vo->iopmp_cfg[group] = readl(vo->iopmp +
						   A210_IOPMP_CONFIG(group));
			for (word = 0; word < A210_IOPMP_ADDR_WORDS; word++)
				vo->iopmp_addr[group][word] = readl(vo->iopmp +
					A210_IOPMP_ADDRESS(group) + word * 4);
		}
	}
	vo->valid = true;
}

static void a210_vo_restore(struct a210_vo_context *vo)
{
	if (!vo || !vo->valid)
		return;

	/* All consumers are suspended; latch dividers before ungating. */
	writel(0, vo->base + A210_VO_CLOCK_ENABLE);
	writel(vo->pixel_div & ~A210_VO_PIXEL_SYNC,
	       vo->base + A210_VO_PIXEL_DIVIDER);
	writel(vo->bus_div & ~A210_VO_BUS_SYNC,
	       vo->base + A210_VO_BUS_DIVIDER);
	udelay(1);
	writel(vo->pixel_div, vo->base + A210_VO_PIXEL_DIVIDER);
	writel(vo->bus_div, vo->base + A210_VO_BUS_DIVIDER);
	writel(vo->path, vo->base + A210_VO_PATH_CONTROL);
	writel(vo->gates, vo->base + A210_VO_CLOCK_ENABLE);
	readl(vo->base + A210_VO_CLOCK_ENABLE);
}

static int a210_vo_restore_iopmp(struct a210_vo_context *vo)
{
	unsigned int group, word;

	if (!vo || !vo->valid || !vo->iopmp)
		return 0;

	/* Preserve the boot policy, including disabled entries and locks. */
	for (group = 0; group < A210_IOPMP_GROUPS; group++)
		writel(0, vo->iopmp + A210_IOPMP_CONFIG(group));
	for (group = 0; group < A210_IOPMP_GROUPS; group++) {
		for (word = 0; word < A210_IOPMP_ADDR_WORDS; word++) {
			void __iomem *addr = vo->iopmp +
				A210_IOPMP_ADDRESS(group) + word * 4;

			writel(vo->iopmp_addr[group][word], addr);
			if (readl(addr) != vo->iopmp_addr[group][word])
				return -EIO;
		}
		writel(vo->iopmp_cfg[group],
		       vo->iopmp + A210_IOPMP_CONFIG(group));
		if (readl(vo->iopmp + A210_IOPMP_CONFIG(group)) !=
		    vo->iopmp_cfg[group])
			return -EIO;
	}
	return 0;
}

/* One complete config word per master, preserving other masters' rules. */
static int a210_gpu_iopmp_enable(struct a210_pm_domain *pd)
{
	static const u32 masters[] = {
		A210_IOPMP_GPU_WRAPPER, A210_IOPMP_GPU,
	};
	void __iomem *base = pd->iopmp_base;
	unsigned int slots[ARRAY_SIZE(masters)];
	unsigned int i, g;

	if (!base)
		return 0;

	/* Find both slots before changing any permissions. */
	for (i = 0; i < ARRAY_SIZE(masters); i++) {
		unsigned int free_slot = A210_IOPMP_GROUPS;
		u32 tag = masters[i] << A210_IOPMP_DEVICE_SHIFT;

		slots[i] = A210_IOPMP_GROUPS;
		for (g = 0; g < A210_IOPMP_GROUPS; g++) {
			void __iomem *addr = base + A210_IOPMP_ADDRESS(g);
			u32 cfg;

			if (i && g == slots[0])
				continue;
			cfg = readl(base + A210_IOPMP_CONFIG(g));
			if (!cfg && free_slot == A210_IOPMP_GROUPS)
				free_slot = g;
			if (cfg == A210_IOPMP_TOR_RWX && !readl(addr) &&
			    readl(addr + 4) == tag && !readl(addr + 8) &&
			    readl(addr + 12) == (tag | A210_IOPMP_LIMIT_HIGH)) {
				slots[i] = g;
				break;
			}
		}
		if (slots[i] == A210_IOPMP_GROUPS)
			slots[i] = free_slot;
		if (slots[i] == A210_IOPMP_GROUPS)
			return -ENOSPC;
	}

	/* TOR pairs cover the SoC's [0, 2^39) physical address space. */
	for (i = 0; i < ARRAY_SIZE(masters); i++) {
		void __iomem *addr = base + A210_IOPMP_ADDRESS(slots[i]);
		u32 tag = masters[i] << A210_IOPMP_DEVICE_SHIFT;

		writel(0, base + A210_IOPMP_CONFIG(slots[i]));
		writel(0, addr);
		writel(tag, addr + 4);
		writel(0, addr + 8);
		writel(tag | A210_IOPMP_LIMIT_HIGH, addr + 12);
		writel(A210_IOPMP_TOR_RWX,
		       base + A210_IOPMP_CONFIG(slots[i]));
		if (readl(base + A210_IOPMP_CONFIG(slots[i])) !=
		    A210_IOPMP_TOR_RWX || readl(addr) || readl(addr + 8) ||
		    readl(addr + 4) != tag ||
		    readl(addr + 12) != (tag | A210_IOPMP_LIMIT_HIGH)) {
			/* The GPU has not been released to a consumer yet. */
			for (g = 0; g <= i; g++)
				writel(0, base + A210_IOPMP_CONFIG(slots[g]));
			return -EIO;
		}
	}
	return 0;
}

static inline struct a210_pm_domain *to_a210_pd(struct generic_pm_domain *domain)
{
	return container_of(domain, struct a210_pm_domain, pd);
}

static void __iomem *a210_ioremap_by_index(struct device *dev,
					   struct device_node *np, int index)
{
	struct resource res;
	void __iomem *base;
	int ret;

	ret = of_address_to_resource(np, index, &res);
	if (ret) {
		dev_err(dev, "failed to get %pOF resource %d: %d\n", np,
			index, ret);
		return ERR_PTR(ret);
	}

	/*
	 * The domain-local CCU registers are also covered by the main clock
	 * controller's register windows, so they cannot be requested
	 * exclusively here.
	 */
	base = devm_ioremap(dev, res.start, resource_size(&res));
	if (!base)
		return ERR_PTR(-ENOMEM);

	return base;
}

static void __iomem *a210_ioremap_by_name(struct device *dev,
					  struct device_node *np,
					  const char *name)
{
	int idx;

	idx = of_property_match_string(np, "reg-names", name);
	if (idx < 0)
		return NULL;

	return a210_ioremap_by_index(dev, np, idx);
}

static int a210_parse_ccu(struct device *dev, struct device_node *np,
			  struct a210_pm_domain *a210_pd)
{
	int ret;

	INIT_LIST_HEAD(&a210_pd->ccu_grp);
	for_each_available_child_of_node_scoped(np, child) {
		struct a210_ccu_grp *ccu_grp;
		int count;
		int i;

		if (!of_device_is_compatible(child, "zhihe,a210-ccu"))
			continue;

		count = of_address_count(child);
		if (count <= 0)
			continue;

		ccu_grp = devm_kzalloc(dev, struct_size(ccu_grp, ccu, count), GFP_KERNEL);
		if (!ccu_grp)
			return -ENOMEM;

		ccu_grp->num_ccus = count;
		ccu_grp->mode = CCU_MODE_GATING;
		of_property_read_u32(child, "zhihe,ccu-mode", &ccu_grp->mode);

		for (i = 0; i < ccu_grp->num_ccus; i++) {
			ccu_grp->ccu[i].base =
				a210_ioremap_by_index(dev, child, i);
			if (IS_ERR(ccu_grp->ccu[i].base))
				return PTR_ERR(ccu_grp->ccu[i].base);

			ret = of_property_read_u32_index(child,
							 "zhihe,ccu-dly-time", i,
							 &ccu_grp->ccu[i].delay);
			if (ret) {
				dev_err(dev,
					"ccu failed to get dly-time for %pOF idx=%d\n",
					child, i);
				return -EINVAL;
			}

			ret = of_property_read_u32_index(child,
							 "zhihe,ccu-dly-time-step", i,
							 &ccu_grp->ccu[i].step);
			if (ret) {
				dev_err(dev,
					"ccu failed to get dly-time-step for %pOF idx=%d\n",
					child, i);
				return -EINVAL;
			}

			if (ccu_grp->mode == CCU_MODE_SCALING) {
				ret = of_property_read_u32_index(child,
								 "zhihe,ccu-ratio", i,
								 &ccu_grp->ccu[i].ratio);
				if (ret) {
					dev_err(dev,
						"ccu failed to get ratio for %pOF idx=%d\n",
						child, i);
					return -EINVAL;
				}
			}
		}

		list_add_tail(&ccu_grp->list, &a210_pd->ccu_grp);
	}

	return 0;
}

static void a210_config_ccu(struct a210_pm_domain *a210_pd)
{
	struct a210_ccu_grp *ccu_grp;

	list_for_each_entry(ccu_grp, &a210_pd->ccu_grp, list) {
		for (int i = 0; i < ccu_grp->num_ccus; i++) {
			u32 val = ccu_grp->ccu[i].delay |
				  (ccu_grp->ccu[i].step << 8);

			writel(val, ccu_grp->ccu[i].base + A210_CCU_DELAY);
			if (ccu_grp->mode == CCU_MODE_GATING) {
				writel(A210_CCU_GATING_ENABLE,
				       ccu_grp->ccu[i].base + A210_CCU_CONTROL);
			} else {
				writel(A210_CCU_SCALING_ENABLE,
				       ccu_grp->ccu[i].base + A210_CCU_CONTROL);
				writel(ccu_grp->ccu[i].ratio,
				       ccu_grp->ccu[i].base +
				       A210_CCU_LOW_POWER_RATIO);
			}
		}
	}
}

static void bpc_config(struct device *dev, const char *name,
		       void __iomem *base_addr, u32 bpc_ctrl)
{
	if ((bpc_ctrl & BIT(0)) != 0) {
		dev_dbg(dev, "enter %s software mode\n", name);
		writel(0x1, base_addr + A210_BPC_BYPASS);
		writel(0x18, base_addr + A210_BPC_RESET_CONTROL);
	} else {
		dev_dbg(dev, "enter %s hardware mode\n", name);
		writel(0x0, base_addr + A210_BPC_BYPASS);
	}
	writel(0x10101, base_addr + A210_BPC_POWER_CONTROL);
}

static int pcu_wait(struct device *dev, const char *name,
		    void __iomem *base_addr)
{
	u32 data;
	int ret;

	ret = readl_poll_timeout(base_addr + A210_PCU_IRQ_STATUS, data, data,
				 1, 10000);
	if (ret) {
		dev_err(dev, "%s PCU request timed out\n", name);
		return ret;
	}

	if (((data & BIT(0)) != 0) || ((data & BIT(3)) != 0))
		dev_dbg(dev, "%s PCU request accepted\n", name);
	writel(data, base_addr + A210_PCU_IRQ_CLEAR);

	if ((data & BIT(1)) || (data & BIT(4))) {
		dev_err(dev, "%s PCU request denied\n", name);
		return -EACCES;
	}
	if ((data & BIT(2)) || (data & BIT(5))) {
		dev_err(dev, "%s PCU request timed out\n", name);
		return -ETIMEDOUT;
	}

	return 0;
}

static int pcu_config(struct device *dev, const char *name,
		      void __iomem *base_addr, u32 pcu_ctrl, u32 state)
{
	writel(0x3f, base_addr + A210_PCU_IRQ_CLEAR);
	writel(0x3f, base_addr + A210_PCU_IRQ_ENABLE);
	if ((pcu_ctrl & BIT(0)) != 0) {
		dev_dbg(dev, "%s PCU state %#x\n", name, state);
		writel(state & 0x1f, base_addr + A210_PCU_LOW_POWER_STATE);
		writel(0x1, base_addr + A210_PCU_REQUEST);
		return pcu_wait(dev, name, base_addr);
	}

	dev_dbg(dev, "%s waits for R2P trigger\n", name);
	return 0;
}

static int a210_pd_power_switch(struct generic_pm_domain *domain,
				enum power_mode mode)
{
	struct a210_pm_domain *a210_pd = to_a210_pd(domain);
	struct a210_pd_soc *soc = a210_pd->soc;
	struct device *dev = soc->dev;
	struct regulator *regulator = soc->regulators[a210_pd->index];
	const char *name = domain->name;
	int ret;

	if (mode == ON && regulator) {
		ret = regulator_enable(regulator);
		if (ret) {
			dev_err(dev, "failed to enable %s regulator: %d\n",
				name, ret);
			return ret;
		}
	}

	/* config pca if needed */
	if (mode == ON && a210_pd->pca_base)
		writel(0x0, a210_pd->pca_base + A210_PCA_CONTROL);

	if (a210_pd->bpc_base)
		bpc_config(dev, name, a210_pd->bpc_base, BPC_HW_MODEL);
	if (a210_pd->pcu_base) {
		ret = pcu_config(dev, name, a210_pd->pcu_base,
				 PCU_REG_TRIGGER, mode);
		if (ret) {
			if (mode == ON && regulator)
				regulator_disable(regulator);
			return ret;
		}
	}

	if (mode == OFF && regulator) {
		ret = regulator_disable(regulator);
		if (ret) {
			dev_err(dev, "failed to disable %s regulator: %d\n",
				name, ret);
			return ret;
		}
	}

	return 0;
}

static int a210_pd_power_off(struct generic_pm_domain *domain)
{
	struct a210_pm_domain *pd = to_a210_pd(domain);
	int ret;

	a210_vo_save(pd->vo);

	/* Reset registers may reside inside the domain being powered down. */
	ret = reset_control_assert(pd->reset);
	if (ret)
		return ret;

	/* The downstream low-power handshake still needs the bus clocks. */
	ret = a210_pd_power_switch(domain, OFF);
	if (ret) {
		reset_control_deassert(pd->reset);
		return ret;
	}
	clk_bulk_disable(pd->num_clks, pd->clks);
	return 0;
}

static int a210_pd_power_on(struct generic_pm_domain *domain)
{
	struct a210_pm_domain *pd = to_a210_pd(domain);
	int ret;

	ret = clk_bulk_enable(pd->num_clks, pd->clks);
	if (ret)
		return ret;

	ret = a210_pd_power_switch(domain, ON);
	if (ret)
		goto disable_clocks;
	/* Power cycling resets VOSYS behind the clock framework. */
	a210_vo_restore(pd->vo);
	ret = reset_control_deassert(pd->reset);
	if (ret) {
		a210_pd_power_switch(domain, OFF);
		goto disable_clocks;
	}

	a210_config_ccu(pd);
	ret = a210_vo_restore_iopmp(pd->vo);
	if (!ret)
		ret = a210_gpu_iopmp_enable(pd);
	if (ret) {
		reset_control_assert(pd->reset);
		a210_pd_power_switch(domain, OFF);
		goto disable_clocks;
	}
	return 0;

disable_clocks:
	clk_bulk_disable(pd->num_clks, pd->clks);
	return ret;
}

struct a210_pd_match_data {
	const char *compatible;
	u32 id;
};

static const struct a210_pd_match_data a210_pd_matches[] = {
	{ .compatible = "zhihe,a210-pd-gpu", .id = A210_PD_GPU },
	{ .compatible = "zhihe,a210-pd-npu-wrapper", .id = A210_PD_NPU_WRAPPER },
	{ .compatible = "zhihe,a210-pd-npu-ip", .id = A210_PD_NPU_IP },
	{ .compatible = "zhihe,a210-pd-d2d", .id = A210_PD_D2D },
	{ .compatible = "zhihe,a210-pd-pcie0", .id = A210_PD_PCIE0 },
	{ .compatible = "zhihe,a210-pd-pcie1", .id = A210_PD_PCIE1 },
	{ .compatible = "zhihe,a210-pd-sata", .id = A210_PD_SATA },
	{ .compatible = "zhihe,a210-pd-usb", .id = A210_PD_USB },
	{ .compatible = "zhihe,a210-pd-vi-wrap", .id = A210_PD_VI_WRAP },
	{ .compatible = "zhihe,a210-pd-vi-isp", .id = A210_PD_VI_ISP },
	{ .compatible = "zhihe,a210-pd-vo", .id = A210_PD_VO },
	{ .compatible = "zhihe,a210-pd-vp-wrap", .id = A210_PD_VP_WRAP },
	{ .compatible = "zhihe,a210-pd-venc", .id = A210_PD_VENC },
	{ .compatible = "zhihe,a210-pd-vdec", .id = A210_PD_VDEC },
	{ .compatible = "zhihe,a210-pd-top", .id = A210_PD_TOP },
	{ .compatible = "zhihe,a210-pd-peri0", .id = A210_PD_PERI0 },
	{ .compatible = "zhihe,a210-pd-peri1", .id = A210_PD_PERI1 },
	{ .compatible = "zhihe,a210-pd-peri2", .id = A210_PD_PERI2 },
	{ .compatible = "zhihe,a210-pd-peri3", .id = A210_PD_PERI3 },
	{ .compatible = "zhihe,a210-pd-can", .id = A210_PD_CAN },
	{ .compatible = "zhihe,a210-pd-spi", .id = A210_PD_SPI },
	{ .compatible = "zhihe,a210-pd-qspi", .id = A210_PD_QSPI },
	{ .compatible = "zhihe,a210-pd-uart", .id = A210_PD_UART },
	{ .compatible = "zhihe,a210-pd-pwm", .id = A210_PD_PWM },
	{ .compatible = "zhihe,a210-pd-emmc", .id = A210_PD_EMMC },
	{ .compatible = "zhihe,a210-pd-sdio", .id = A210_PD_SDIO },
	{ .compatible = "zhihe,a210-pd-dmac", .id = A210_PD_DMAC },
};

static int a210_domain_lookup(struct device_node *np)
{
	for (int i = 0; i < ARRAY_SIZE(a210_pd_matches); i++) {
		if (of_device_is_compatible(np, a210_pd_matches[i].compatible))
			return a210_pd_matches[i].id;
	}

	return -ENODEV;
}

static int a210_add_one_domain(struct platform_device *pdev,
			       struct device_node *np)
{
	struct device *dev = &pdev->dev;
	struct a210_pd_soc *pd_soc = dev_get_drvdata(dev);
	struct a210_pm_domain *a210_pd;
	const char *name;
	int id;
	int ret;

	a210_pd = devm_kzalloc(dev, sizeof(*a210_pd), GFP_KERNEL);
	if (!a210_pd)
		return -ENOMEM;

	id = a210_domain_lookup(np);
	if (id < 0)
		return -ENODEV;
	if (pd_soc->num_domains >= ARRAY_SIZE(pd_soc->domains))
		return -E2BIG;

	name = np->name;
	a210_pd->pd.name = devm_kstrdup(dev, name, GFP_KERNEL);
	if (!a210_pd->pd.name)
		return -ENOMEM;

	a210_pd->index = id;
	a210_pd->pd.power_off = a210_pd_power_off;
	a210_pd->pd.power_on = a210_pd_power_on;
	a210_pd->soc = pd_soc;
	a210_pd->np = np;

	if (id == A210_PD_TOP)
		a210_pd->pd.flags |= GENPD_FLAG_ALWAYS_ON;

	a210_pd->pca_base = a210_ioremap_by_name(dev, np, "pca");
	if (IS_ERR(a210_pd->pca_base))
		return PTR_ERR(a210_pd->pca_base);
	a210_pd->bpc_base = a210_ioremap_by_name(dev, np, "bpc");
	if (IS_ERR(a210_pd->bpc_base))
		return PTR_ERR(a210_pd->bpc_base);
	a210_pd->pcu_base = a210_ioremap_by_name(dev, np, "pcu");
	if (IS_ERR(a210_pd->pcu_base))
		return PTR_ERR(a210_pd->pcu_base);

	if (id == A210_PD_GPU) {
		a210_pd->iopmp_base = a210_ioremap_by_name(dev, np, "iopmp");
		if (IS_ERR(a210_pd->iopmp_base))
			return PTR_ERR(a210_pd->iopmp_base);
		if (!a210_pd->iopmp_base)
			return dev_err_probe(dev, -EINVAL, "missing GPU IOPMP bank\n");
	}

	if (id == A210_PD_VO) {
		void __iomem *base = a210_ioremap_by_name(dev, np, "syscon");

		if (IS_ERR(base))
			return PTR_ERR(base);
		/* Older firmware DTs may omit the VOSYS context window. */
		if (base) {
			a210_pd->vo = devm_kzalloc(dev, sizeof(*a210_pd->vo),
						   GFP_KERNEL);
			if (!a210_pd->vo)
				return -ENOMEM;
			a210_pd->vo->base = base;
			a210_pd->vo->iopmp =
				a210_ioremap_by_name(dev, np, "iopmp");
			if (IS_ERR(a210_pd->vo->iopmp))
				return PTR_ERR(a210_pd->vo->iopmp);
		}
	}

	a210_pd->reset = of_reset_control_array_get_optional_shared(np);
	if (IS_ERR(a210_pd->reset)) {
		ret = PTR_ERR(a210_pd->reset);
		dev_err_probe(dev, ret, "failed to get %s resets\n", np->name);
		return ret;
	}

	a210_pd->num_clks = of_clk_get_parent_count(np);
	if (a210_pd->num_clks) {
		a210_pd->clks = devm_kcalloc(dev, a210_pd->num_clks,
					     sizeof(*a210_pd->clks), GFP_KERNEL);
		if (!a210_pd->clks) {
			ret = -ENOMEM;
			goto put_reset;
		}

		for (int i = 0; i < a210_pd->num_clks; i++) {
			a210_pd->clks[i].clk = of_clk_get(np, i);
			if (IS_ERR(a210_pd->clks[i].clk)) {
				ret = PTR_ERR(a210_pd->clks[i].clk);
				dev_err_probe(dev, ret,
					      "failed to get %s clock %d\n",
					      np->name, i);
				clk_bulk_put(i, a210_pd->clks);
				goto put_reset;
			}
		}

		ret = clk_bulk_prepare(a210_pd->num_clks, a210_pd->clks);
		if (ret) {
			clk_bulk_put(a210_pd->num_clks, a210_pd->clks);
			goto put_reset;
		}
	}

	ret = a210_parse_ccu(dev, np, a210_pd);
	if (ret)
		goto unprepare_clocks;

	ret = pm_genpd_init(&a210_pd->pd, NULL,
			    !(a210_pd->pd.flags & GENPD_FLAG_ALWAYS_ON));
	if (ret) {
		dev_err_probe(dev, ret, "failed to initialize domain %s\n",
			      a210_pd->pd.name);
		goto unprepare_clocks;
	}

	ret = of_genpd_add_provider_simple(np, &a210_pd->pd);
	if (ret) {
		dev_err(dev, "failed to add PM domain provider for %pOFn: %d\n",
			np, ret);
		goto remove_genpd;
	}

	pd_soc->domains[pd_soc->num_domains++] = a210_pd;

	dev_dbg(dev, "added PM domain %s\n", a210_pd->pd.name);

	return 0;

remove_genpd:
	pm_genpd_remove(&a210_pd->pd);
unprepare_clocks:
	clk_bulk_unprepare(a210_pd->num_clks, a210_pd->clks);
	clk_bulk_put(a210_pd->num_clks, a210_pd->clks);
put_reset:
	reset_control_put(a210_pd->reset);
	return ret;
}

static int a210_init_pm_domains(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct of_phandle_args child_args, parent_args;
	int ret = 0;

	for_each_available_child_of_node_scoped(np, child) {
		ret = a210_add_one_domain(pdev, child);
		if (ret) {
			dev_err(dev, "failed to handle node %pOFn: %d\n",
				child, ret);
			return ret;
		}
	}

	/* Register every provider before linking child domains. */
	for_each_available_child_of_node_scoped(np, child) {
		if (of_parse_phandle_with_args(child, "power-domains",
					       "#power-domain-cells", 0,
					       &parent_args))
			continue;

		child_args.np = child;
		child_args.args_count = 0;

		ret = of_genpd_add_subdomain(&parent_args, &child_args);
		of_node_put(parent_args.np);
		if (ret) {
			dev_err(dev, "failed to handle subdomain node %pOFn: %d\n",
				child, ret);
			return ret;
		}
	}

	return ret;
}

static void a210_remove_pm_domains(struct a210_pd_soc *pd_soc)
{
	while (pd_soc->num_domains) {
		struct a210_pm_domain *a210_pd;

		a210_pd = pd_soc->domains[--pd_soc->num_domains];
		of_genpd_del_provider(a210_pd->np);
		clk_bulk_unprepare(a210_pd->num_clks, a210_pd->clks);
		clk_bulk_put(a210_pd->num_clks, a210_pd->clks);
		reset_control_put(a210_pd->reset);
		pm_genpd_remove(&a210_pd->pd);
	}
}

static int a210_pd_parse_regulators(struct device *dev)
{
	struct a210_pd_soc *pd_soc = dev_get_drvdata(dev);

	for_each_available_child_of_node_scoped(dev->of_node, child) {
		int id;
		int ret;

		id = a210_domain_lookup(child);
		if (id < 0)
			return -ENODEV;

		if (!of_property_present(child, "pmic-supply"))
			continue;

		pd_soc->regulators[id] =
			devm_of_regulator_get_optional(dev, child, "pmic");
		if (IS_ERR(pd_soc->regulators[id])) {
			ret = PTR_ERR(pd_soc->regulators[id]);
			return dev_err_probe(dev, ret,
					     "failed to get %s regulator\n",
					     child->name);
		}
	}

	return 0;
}

static int a210_pd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct a210_pd_soc *pd_soc;
	int ret;

	pd_soc = devm_kzalloc(dev, sizeof(*pd_soc), GFP_KERNEL);
	if (!pd_soc)
		return -ENOMEM;
	pd_soc->dev = dev;

	dev_set_drvdata(dev, pd_soc);

	ret = a210_pd_parse_regulators(dev);
	if (ret)
		return ret;

	ret = a210_init_pm_domains(pdev);
	if (ret) {
		a210_remove_pm_domains(pd_soc);
		return ret;
	}

	dev_info(dev, "registered A210 power domains\n");

	return ret;
}

static const struct of_device_id a210_pd_of_match[] = {
	{ .compatible = "zhihe,a210-power-domain" },
	{ /* Sentinel */ },
};
MODULE_DEVICE_TABLE(of, a210_pd_of_match);

static struct platform_driver a210_pd_driver = {
	.probe = a210_pd_probe,
	.driver = {
		.name = "a210-power-domain",
		.of_match_table = a210_pd_of_match,
		.suppress_bind_attrs = true,
	},
};

module_platform_driver(a210_pd_driver);

MODULE_AUTHOR("Dong Yan <yand@zhcomputing.com>");
MODULE_DESCRIPTION("Zhihe A210 power domain driver");
MODULE_LICENSE("GPL");
