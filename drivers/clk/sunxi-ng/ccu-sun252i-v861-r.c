// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2026 Han Gao <gaohan@iscas.ac.cn> */

#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <dt-bindings/clock/sun252i-v861-r-ccu.h>
#include <dt-bindings/clock/sun252i-v861-rtc-ccu.h>
#include <dt-bindings/reset/sun252i-v861-r-ccu.h>

#include "ccu_common.h"
#include "ccu_reset.h"
#include "ccu_div.h"
#include "ccu_gate.h"
#include "ccu_mux.h"

static const struct clk_parent_data r_bus_parents[] = {
	{ .fw_name = "hosc" }, { .fw_name = "losc" },
	{ .fw_name = "iosc" }, { .fw_name = "pll-periph" },
};
static const struct clk_parent_data hosc_parent[] = { { .fw_name = "hosc" } };

static SUNXI_CCU_M_DATA_WITH_MUX(r_ahb_clk, "r-ahb", r_bus_parents,
				 0x000, 0, 5, 24, 3, CLK_IS_CRITICAL);
static SUNXI_CCU_M_DATA_WITH_MUX(r_apb0_clk, "r-apb0", r_bus_parents,
				 0x00c, 0, 5, 24, 3, 0);
static SUNXI_CCU_GATE_DATA(bus_r_ppu_clk, "bus-r-ppu", hosc_parent,
			   0x1ac, BIT(0), CLK_IS_CRITICAL);
static SUNXI_CCU_GATE_DATA(bus_r_rtc_clk, "bus-r-rtc", hosc_parent,
			   0x20c, BIT(0), CLK_IS_CRITICAL);

static struct ccu_common *r_ccu_clks[] = {
	&r_ahb_clk.common, &r_apb0_clk.common,
	&bus_r_ppu_clk.common, &bus_r_rtc_clk.common,
};
static struct clk_hw_onecell_data r_hw_clks = {
	.num = CLK_BUS_R_RTC + 1,
	.hws = {
		[CLK_R_AHB] = &r_ahb_clk.common.hw,
		[CLK_R_APB0] = &r_apb0_clk.common.hw,
		[CLK_BUS_R_PPU] = &bus_r_ppu_clk.common.hw,
		[CLK_BUS_R_RTC] = &bus_r_rtc_clk.common.hw,
	},
};
static const struct ccu_reset_map r_resets[] = {
	[RST_BUS_R_PPU] = { 0x1ac, BIT(16) },
	[RST_BUS_R_RTC] = { 0x20c, BIT(16) },
};
static const struct sunxi_ccu_desc r_ccu_desc = {
	.ccu_clks = r_ccu_clks,
	.num_ccu_clks = ARRAY_SIZE(r_ccu_clks),
	.hw_clks = &r_hw_clks,
	.resets = r_resets,
	.num_resets = ARRAY_SIZE(r_resets),
};

static const struct clk_parent_data iosc_parent[] = { { .fw_name = "iosc" } };
static SUNXI_CCU_GATE_DATA(iosc_clk, "iosc", iosc_parent,
			   0x160, BIT(0), CLK_IS_CRITICAL);
static CLK_FIXED_FACTOR_HW(iosc_32k_clk, "iosc-32k", &iosc_clk.common.hw,
			   500, 1, 0);
static const struct clk_parent_data losc_parents[] = {
	{ .hw = &iosc_32k_clk.hw }, { .fw_name = "ext-osc32k" },
};
static struct ccu_mux losc_clk = {
	.mux = _SUNXI_CCU_MUX(0, 1),
	.common = {
		.reg = 0x000,
		.features = CCU_FEATURE_KEY_FIELD,
		.hw.init = CLK_HW_INIT_PARENTS_DATA("losc", losc_parents,
						 &ccu_mux_ops, CLK_IS_CRITICAL),
	},
};
static SUNXI_CCU_GATE_DATA(hosc_32k_gate_clk, "hosc-32k-gate", hosc_parent,
			   0x060, BIT(16), 0);
static CLK_FIXED_FACTOR_HW(hosc_32k_clk, "hosc-32k", &hosc_32k_gate_clk.common.hw,
			   375000, 512, 0);
static const struct clk_parent_data rtc32k_parents[] = {
	{ .hw = &losc_clk.common.hw }, { .hw = &hosc_32k_clk.hw },
};
static struct ccu_mux rtc32k_clk = {
	.mux = _SUNXI_CCU_MUX(1, 1),
	.common = {
		.reg = 0x000,
		.features = CCU_FEATURE_KEY_FIELD,
		.hw.init = CLK_HW_INIT_PARENTS_DATA("rtc32k", rtc32k_parents,
						 &ccu_mux_ops, CLK_IS_CRITICAL),
	},
};
static struct ccu_common *rtc_ccu_clks[] = {
	&iosc_clk.common, &losc_clk.common,
	&hosc_32k_gate_clk.common, &rtc32k_clk.common,
};
static struct clk_hw_onecell_data rtc_hw_clks = {
	.num = CLK_RTC_32K + 1,
	.hws = {
		[CLK_RTC_IOSC] = &iosc_clk.common.hw,
		[CLK_RTC_LOSC] = &losc_clk.common.hw,
		[CLK_RTC_IOSC_32K] = &iosc_32k_clk.hw,
		[CLK_RTC_HOSC_GATE] = &hosc_32k_gate_clk.common.hw,
		[CLK_RTC_HOSC_32K] = &hosc_32k_clk.hw,
		[CLK_RTC_32K] = &rtc32k_clk.common.hw,
	},
};
static const struct sunxi_ccu_desc rtc_ccu_desc = {
	.ccu_clks = rtc_ccu_clks,
	.num_ccu_clks = ARRAY_SIZE(rtc_ccu_clks),
	.hw_clks = &rtc_hw_clks,
};

static int sun252i_v861_r_ccu_probe(struct platform_device *pdev)
{
	const struct sunxi_ccu_desc *desc = device_get_match_data(&pdev->dev);
	void __iomem *reg;

	reg = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(reg))
		return PTR_ERR(reg);

	return devm_sunxi_ccu_probe(&pdev->dev, reg, desc);
}

static const struct of_device_id sun252i_v861_r_ccu_ids[] = {
	{ .compatible = "allwinner,sun252i-v861-r-ccu", .data = &r_ccu_desc },
	{ .compatible = "allwinner,sun252i-v861-rtc-ccu", .data = &rtc_ccu_desc },
	{ }
};
MODULE_DEVICE_TABLE(of, sun252i_v861_r_ccu_ids);

static struct platform_driver sun252i_v861_r_ccu_driver = {
	.probe = sun252i_v861_r_ccu_probe,
	.driver = {
		.name = "sun252i-v861-r-ccu",
		.suppress_bind_attrs = true,
		.of_match_table = sun252i_v861_r_ccu_ids,
	},
};
module_platform_driver(sun252i_v861_r_ccu_driver);

MODULE_IMPORT_NS("SUNXI_CCU");
MODULE_DESCRIPTION("Allwinner V861 always-on peripheral clocks");
MODULE_LICENSE("GPL");
