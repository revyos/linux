// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2026 Han Gao <gaohan@iscas.ac.cn> */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/pinctrl.h>

#include "pinctrl-sunxi.h"

static const u8 v861_nr_bank_pins[SUNXI_PINCTRL_MAX_BANKS] = {
	/* PA  PB  PC  PD  PE  PF  PG  PH */
	   22,  0, 12, 23, 18,  7,  8, 16,
};
static const u8 v861_irq_bank_muxes[SUNXI_PINCTRL_MAX_BANKS] = {
	14, 0, 14, 14, 14, 14, 14, 14,
};
static const unsigned int v861_irq_bank_map[] = { 0, 2, 3, 4, 5, 6, 7 };

static const u8 v861_r_nr_bank_pins[SUNXI_PINCTRL_MAX_BANKS] = { 7 };
static const u8 v861_r_irq_bank_muxes[SUNXI_PINCTRL_MAX_BANKS] = { 14 };

static int sun252i_v861_pinctrl_probe(struct platform_device *pdev)
{
	struct sunxi_pinctrl_desc *desc;
	bool rtc = of_device_is_compatible(pdev->dev.of_node,
					 "allwinner,sun252i-v861-r-pinctrl");

	desc = devm_kzalloc(&pdev->dev, sizeof(*desc), GFP_KERNEL);
	if (!desc)
		return -ENOMEM;

	if (rtc) {
		desc->pin_base = PL_BASE;
		desc->irq_banks = 1;
		desc->irq_reg_base = 0x40;
		return sunxi_pinctrl_dt_table_init(pdev, v861_r_nr_bank_pins,
					   v861_r_irq_bank_muxes, desc,
					   SUNXI_PINCTRL_NEW_REG_LAYOUT);
	}

	desc->irq_banks = ARRAY_SIZE(v861_irq_bank_map);
	desc->irq_bank_map = v861_irq_bank_map;
	return sunxi_pinctrl_dt_table_init(pdev, v861_nr_bank_pins,
					 v861_irq_bank_muxes, desc,
					 SUNXI_PINCTRL_NEW_REG_LAYOUT |
					 SUNXI_PINCTRL_ELEVEN_BANKS);
}

static const struct of_device_id sun252i_v861_pinctrl_match[] = {
	{ .compatible = "allwinner,sun252i-v861-pinctrl" },
	{ .compatible = "allwinner,sun252i-v861-r-pinctrl" },
	{ }
};
MODULE_DEVICE_TABLE(of, sun252i_v861_pinctrl_match);

static struct platform_driver sun252i_v861_pinctrl_driver = {
	.probe = sun252i_v861_pinctrl_probe,
	.driver = {
		.name = "sun252i-v861-pinctrl",
		.of_match_table = sun252i_v861_pinctrl_match,
	},
};
builtin_platform_driver(sun252i_v861_pinctrl_driver);

MODULE_DESCRIPTION("Allwinner V861 main and always-on pin controllers");
MODULE_LICENSE("GPL");
