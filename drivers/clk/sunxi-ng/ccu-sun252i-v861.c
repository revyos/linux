// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2026 Han Gao <gaohan@iscas.ac.cn> */

#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <dt-bindings/clock/sun252i-v861-ccu.h>
#include <dt-bindings/reset/sun252i-v861-ccu.h>

#include "ccu_common.h"
#include "ccu_reset.h"
#include "ccu_div.h"
#include "ccu_gate.h"
#include "ccu_mp.h"
#include "ccu_nm.h"

static const struct clk_parent_data hosc_parent[] = {
	{ .fw_name = "hosc" },
};

static struct ccu_nm pll_periph_clk = {
	.enable = BIT(31) | BIT(30) | BIT(27),
	.lock = BIT(28),
	.n = _SUNXI_CCU_MULT_MIN(8, 8, 53),
	.common = {
		.reg = 0x020,
		.hw.init = CLK_HW_INIT_PARENTS_DATA("pll-periph", hosc_parent,
						 &ccu_nm_ops, CLK_IS_CRITICAL),
	},
};
static const struct clk_hw *pll_periph_hw = &pll_periph_clk.common.hw;

static SUNXI_CCU_M_HWS(pll_periph_2x_clk, "pll-periph-2x", &pll_periph_hw,
			0x020, 16, 3, 0);
static SUNXI_CCU_M_HWS(pll_periph_800m_clk, "pll-periph-800m", &pll_periph_hw,
			0x020, 20, 3, 0);
static SUNXI_CCU_M_HWS(pll_periph_480m_clk, "pll-periph-480m", &pll_periph_hw,
			0x020, 2, 3, 0);

static CLK_FIXED_FACTOR_HW(pll_periph_600m_clk, "pll-periph-600m",
			   &pll_periph_2x_clk.common.hw, 2, 1, 0);
static CLK_FIXED_FACTOR_HW(pll_periph_400m_clk, "pll-periph-400m",
			   &pll_periph_2x_clk.common.hw, 3, 1, 0);
static CLK_FIXED_FACTOR_HW(pll_periph_300m_clk, "pll-periph-300m",
			   &pll_periph_600m_clk.hw, 2, 1, 0);
static CLK_FIXED_FACTOR_HW(pll_periph_200m_clk, "pll-periph-200m",
			   &pll_periph_400m_clk.hw, 2, 1, 0);

static const struct clk_parent_data bus_parents[] = {
	{ .fw_name = "hosc" },
	{ .fw_name = "losc" },
	{ .fw_name = "iosc" },
	{ .hw = &pll_periph_600m_clk.hw },
};

static SUNXI_CCU_MP_DATA_WITH_MUX(ahb_clk, "ahb", bus_parents,
				  0x510, 0, 5, 8, 2, 24, 2, CLK_IS_CRITICAL);
static SUNXI_CCU_MP_DATA_WITH_MUX(apb0_clk, "apb0", bus_parents,
				  0x520, 0, 5, 8, 2, 24, 2, 0);
static SUNXI_CCU_MP_DATA_WITH_MUX(apb1_clk, "apb1", bus_parents,
				  0x524, 0, 5, 8, 2, 24, 2, 0);
static SUNXI_CCU_MP_DATA_WITH_MUX(apb_uart_clk, "apb-uart", bus_parents,
				  0x528, 0, 5, 8, 2, 24, 3, 0);
static const struct clk_hw *apb1_hw = &apb1_clk.common.hw;
static const struct clk_hw *apb_uart_hw = &apb_uart_clk.common.hw;

static const struct clk_parent_data mmc_parents[] = {
	{ .fw_name = "hosc" },
	{ .hw = &pll_periph_400m_clk.hw },
	{ .hw = &pll_periph_300m_clk.hw },
};
static const struct clk_parent_data mmc2_parents[] = {
	{ .fw_name = "hosc" },
	{ .hw = &pll_periph_600m_clk.hw },
	{ .hw = &pll_periph_800m_clk.common.hw },
};
static SUNXI_CCU_MP_DATA_WITH_MUX_GATE(mmc0_clk, "mmc0", mmc_parents,
				       0x830, 0, 4, 8, 2, 24, 3, BIT(31), 0);
static SUNXI_CCU_MP_DATA_WITH_MUX_GATE(mmc1_clk, "mmc1", mmc_parents,
				       0x834, 0, 4, 8, 2, 24, 3, BIT(31), 0);
static SUNXI_CCU_MP_DATA_WITH_MUX_GATE(mmc2_clk, "mmc2", mmc2_parents,
				       0x838, 0, 4, 8, 2, 24, 3, BIT(31), 0);

static SUNXI_CCU_GATE_DATA(bus_mmc0_clk, "bus-mmc0", hosc_parent, 0x84c, BIT(0), 0);
static SUNXI_CCU_GATE_DATA(bus_mmc1_clk, "bus-mmc1", hosc_parent, 0x84c, BIT(1), 0);
static SUNXI_CCU_GATE_DATA(bus_mmc2_clk, "bus-mmc2", hosc_parent, 0x84c, BIT(2), 0);
static SUNXI_CCU_GATE_HWS(bus_uart0_clk, "bus-uart0", &apb_uart_hw, 0x90c, BIT(0), 0);
static SUNXI_CCU_GATE_HWS(bus_uart1_clk, "bus-uart1", &apb_uart_hw, 0x90c, BIT(1), 0);
static SUNXI_CCU_GATE_HWS(bus_uart2_clk, "bus-uart2", &apb_uart_hw, 0x90c, BIT(2), 0);
static SUNXI_CCU_GATE_HWS(bus_uart3_clk, "bus-uart3", &apb_uart_hw, 0x90c, BIT(3), 0);
static SUNXI_CCU_GATE_HWS(bus_i2c0_clk, "bus-i2c0", &apb1_hw, 0x91c, BIT(0), 0);
static SUNXI_CCU_GATE_HWS(bus_i2c1_clk, "bus-i2c1", &apb1_hw, 0x91c, BIT(1), 0);
static SUNXI_CCU_GATE_HWS(bus_i2c2_clk, "bus-i2c2", &apb1_hw, 0x91c, BIT(2), 0);
static SUNXI_CCU_GATE_HWS(bus_i2c3_clk, "bus-i2c3", &apb1_hw, 0x91c, BIT(3), 0);

static const struct clk_parent_data spi_parents[] = {
	{ .fw_name = "hosc" },
	{ .hw = &pll_periph_300m_clk.hw },
	{ .hw = &pll_periph_200m_clk.hw },
};
static SUNXI_CCU_MP_DATA_WITH_MUX_GATE(spi0_clk, "spi0", spi_parents,
				       0x940, 0, 4, 8, 2, 24, 3, BIT(31), 0);
static SUNXI_CCU_MP_DATA_WITH_MUX_GATE(spi1_clk, "spi1", spi_parents,
				       0x944, 0, 4, 8, 2, 24, 3, BIT(31), 0);
static SUNXI_CCU_MP_DATA_WITH_MUX_GATE(spi2_clk, "spi2", spi_parents,
				       0x948, 0, 4, 8, 2, 24, 3, BIT(31), 0);
static SUNXI_CCU_MP_DATA_WITH_MUX_GATE(spif_clk, "spif", mmc_parents,
				       0x950, 0, 4, 8, 2, 24, 3, BIT(31), 0);
static SUNXI_CCU_GATE_DATA(bus_spi0_clk, "bus-spi0", hosc_parent, 0x96c, BIT(0), 0);
static SUNXI_CCU_GATE_DATA(bus_spi1_clk, "bus-spi1", hosc_parent, 0x96c, BIT(1), 0);
static SUNXI_CCU_GATE_DATA(bus_spi2_clk, "bus-spi2", hosc_parent, 0x96c, BIT(2), 0);
static SUNXI_CCU_GATE_DATA(bus_spif_clk, "bus-spif", hosc_parent, 0x96c, BIT(4), 0);

static struct ccu_common *sun252i_v861_ccu_clks[] = {
	&pll_periph_clk.common, &pll_periph_2x_clk.common,
	&pll_periph_800m_clk.common, &pll_periph_480m_clk.common,
	&ahb_clk.common, &apb0_clk.common, &apb1_clk.common, &apb_uart_clk.common,
	&mmc0_clk.common, &mmc1_clk.common, &mmc2_clk.common,
	&bus_mmc0_clk.common, &bus_mmc1_clk.common, &bus_mmc2_clk.common,
	&bus_uart0_clk.common, &bus_uart1_clk.common,
	&bus_uart2_clk.common, &bus_uart3_clk.common,
	&bus_i2c0_clk.common, &bus_i2c1_clk.common,
	&bus_i2c2_clk.common, &bus_i2c3_clk.common,
	&spi0_clk.common, &spi1_clk.common, &spi2_clk.common, &spif_clk.common,
	&bus_spi0_clk.common, &bus_spi1_clk.common,
	&bus_spi2_clk.common, &bus_spif_clk.common,
};

static struct clk_hw_onecell_data sun252i_v861_hw_clks = {
	.num = CLK_BUS_SPIF + 1,
	.hws = {
		[CLK_PLL_PERIPH] = &pll_periph_clk.common.hw,
		[CLK_PLL_PERIPH_2X] = &pll_periph_2x_clk.common.hw,
		[CLK_PLL_PERIPH_800M] = &pll_periph_800m_clk.common.hw,
		[CLK_PLL_PERIPH_480M] = &pll_periph_480m_clk.common.hw,
		[CLK_PLL_PERIPH_600M] = &pll_periph_600m_clk.hw,
		[CLK_PLL_PERIPH_400M] = &pll_periph_400m_clk.hw,
		[CLK_PLL_PERIPH_300M] = &pll_periph_300m_clk.hw,
		[CLK_PLL_PERIPH_200M] = &pll_periph_200m_clk.hw,
		[CLK_AHB] = &ahb_clk.common.hw,
		[CLK_APB0] = &apb0_clk.common.hw,
		[CLK_APB1] = &apb1_clk.common.hw,
		[CLK_APB_UART] = &apb_uart_clk.common.hw,
		[CLK_MMC0] = &mmc0_clk.common.hw,
		[CLK_MMC1] = &mmc1_clk.common.hw,
		[CLK_MMC2] = &mmc2_clk.common.hw,
		[CLK_BUS_MMC0] = &bus_mmc0_clk.common.hw,
		[CLK_BUS_MMC1] = &bus_mmc1_clk.common.hw,
		[CLK_BUS_MMC2] = &bus_mmc2_clk.common.hw,
		[CLK_BUS_UART0] = &bus_uart0_clk.common.hw,
		[CLK_BUS_UART1] = &bus_uart1_clk.common.hw,
		[CLK_BUS_UART2] = &bus_uart2_clk.common.hw,
		[CLK_BUS_UART3] = &bus_uart3_clk.common.hw,
		[CLK_BUS_I2C0] = &bus_i2c0_clk.common.hw,
		[CLK_BUS_I2C1] = &bus_i2c1_clk.common.hw,
		[CLK_BUS_I2C2] = &bus_i2c2_clk.common.hw,
		[CLK_BUS_I2C3] = &bus_i2c3_clk.common.hw,
		[CLK_SPI0] = &spi0_clk.common.hw,
		[CLK_SPI1] = &spi1_clk.common.hw,
		[CLK_SPI2] = &spi2_clk.common.hw,
		[CLK_SPIF] = &spif_clk.common.hw,
		[CLK_BUS_SPI0] = &bus_spi0_clk.common.hw,
		[CLK_BUS_SPI1] = &bus_spi1_clk.common.hw,
		[CLK_BUS_SPI2] = &bus_spi2_clk.common.hw,
		[CLK_BUS_SPIF] = &bus_spif_clk.common.hw,
	},
};

static const struct ccu_reset_map sun252i_v861_resets[] = {
	[RST_BUS_MMC0] = { 0x84c, BIT(16) },
	[RST_BUS_MMC1] = { 0x84c, BIT(17) },
	[RST_BUS_MMC2] = { 0x84c, BIT(18) },
	[RST_BUS_UART0] = { 0x90c, BIT(16) },
	[RST_BUS_UART1] = { 0x90c, BIT(17) },
	[RST_BUS_UART2] = { 0x90c, BIT(18) },
	[RST_BUS_UART3] = { 0x90c, BIT(19) },
	[RST_BUS_I2C0] = { 0x91c, BIT(16) },
	[RST_BUS_I2C1] = { 0x91c, BIT(17) },
	[RST_BUS_I2C2] = { 0x91c, BIT(18) },
	[RST_BUS_I2C3] = { 0x91c, BIT(19) },
	[RST_BUS_SPI0] = { 0x96c, BIT(16) },
	[RST_BUS_SPI1] = { 0x96c, BIT(17) },
	[RST_BUS_SPI2] = { 0x96c, BIT(18) },
	[RST_BUS_SPIF] = { 0x96c, BIT(20) },
};

static const struct sunxi_ccu_desc sun252i_v861_ccu_desc = {
	.ccu_clks = sun252i_v861_ccu_clks,
	.num_ccu_clks = ARRAY_SIZE(sun252i_v861_ccu_clks),
	.hw_clks = &sun252i_v861_hw_clks,
	.resets = sun252i_v861_resets,
	.num_resets = ARRAY_SIZE(sun252i_v861_resets),
};

static int sun252i_v861_ccu_probe(struct platform_device *pdev)
{
	void __iomem *reg;

	reg = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(reg))
		return PTR_ERR(reg);

	/* Enable lock detection without changing any PLL divisors. */
	writel(readl(reg + 0x020) | BIT(29), reg + 0x020);
	return devm_sunxi_ccu_probe(&pdev->dev, reg, &sun252i_v861_ccu_desc);
}

static const struct of_device_id sun252i_v861_ccu_ids[] = {
	{ .compatible = "allwinner,sun252i-v861-ccu" },
	{ }
};
MODULE_DEVICE_TABLE(of, sun252i_v861_ccu_ids);

static struct platform_driver sun252i_v861_ccu_driver = {
	.probe = sun252i_v861_ccu_probe,
	.driver = {
		.name = "sun252i-v861-ccu",
		.suppress_bind_attrs = true,
		.of_match_table = sun252i_v861_ccu_ids,
	},
};
module_platform_driver(sun252i_v861_ccu_driver);

MODULE_IMPORT_NS("SUNXI_CCU");
MODULE_DESCRIPTION("Allwinner V861 peripheral clock and reset controller");
MODULE_LICENSE("GPL");
