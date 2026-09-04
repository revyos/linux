// SPDX-License-Identifier: GPL-2.0-only
/* Read the firmware-selected AON clock and gate the AP-owned I2C bus. */
#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#include <dt-bindings/clock/zhihe,a210-aon-clk.h>

#define AON_CLOCK_CONFIG	0x00
#define AON_CLOCK_GATE		0x04
#define AON_SOURCE_SHIFT	4
#define AON_SOURCE_WIDTH	2
#define AON_DIVIDER_WIDTH	3
#define AON_I2C1_IC_ENABLE	20
#define AON_I2C1_APB_ENABLE	21

struct a210_aon_clock {
	spinlock_t lock;
	struct clk_hw_onecell_data data;
};

static int a210_aon_clock_probe(struct platform_device *pdev)
{
	static const struct clk_parent_data parents[] = {
		{ .fw_name = "high" },
		{ .fw_name = "osc" },
		{ .fw_name = "rc" },
		{ .fw_name = "rtc" },
	};
	struct device *dev = &pdev->dev;
	struct a210_aon_clock *priv;
	struct clk_hw *mux, *sys, *hw;
	void __iomem *base;
	int i;

	priv = devm_kzalloc(dev, struct_size(priv, data.hws, 3), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	spin_lock_init(&priv->lock);
	priv->data.num = 3;
	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	/* E902 firmware owns source selection and the synchronized divider. */
	mux = devm_clk_hw_register_mux_parent_data_table(dev, "aon_mux",
		parents, ARRAY_SIZE(parents), CLK_GET_RATE_NOCACHE,
		base + AON_CLOCK_CONFIG, AON_SOURCE_SHIFT, AON_SOURCE_WIDTH,
		CLK_MUX_READ_ONLY, NULL, &priv->lock);
	if (IS_ERR(mux))
		return PTR_ERR(mux);
	sys = devm_clk_hw_register_divider_parent_hw(dev, "aon_sys", mux,
		CLK_IS_CRITICAL | CLK_GET_RATE_NOCACHE, base + AON_CLOCK_CONFIG,
		0, AON_DIVIDER_WIDTH, CLK_DIVIDER_READ_ONLY, &priv->lock);
	if (IS_ERR(sys))
		return PTR_ERR(sys);
	priv->data.hws[A210_AON_SYS_CLK] = sys;

	for (i = 0; i < 2; i++) {
		hw = devm_clk_hw_register_gate_parent_hw(dev,
			i ? "aon_i2c1_pclk" : "aon_i2c1_ic_clk", sys, 0,
			base + AON_CLOCK_GATE,
			i ? AON_I2C1_APB_ENABLE : AON_I2C1_IC_ENABLE,
			0, &priv->lock);
		if (IS_ERR(hw))
			return PTR_ERR(hw);
		priv->data.hws[A210_AON_I2C1_IC_CLK + i] = hw;
	}

	return devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get,
					  &priv->data);
}

static const struct of_device_id a210_aon_clock_match[] = {
	{ .compatible = "zhihe,a210-aon-clk" },
	{ }
};
MODULE_DEVICE_TABLE(of, a210_aon_clock_match);

static struct platform_driver a210_aon_clock_driver = {
	.probe = a210_aon_clock_probe,
	.driver = {
		.name = "a210-aon-clock",
		.of_match_table = a210_aon_clock_match,
	},
};
module_platform_driver(a210_aon_clock_driver);

MODULE_DESCRIPTION("Zhihe A210 AON peripheral clocks");
MODULE_LICENSE("GPL");
