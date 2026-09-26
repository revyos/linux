// SPDX-License-Identifier: GPL-2.0-only
/* A210 USB2 femtoPHY; register layout from the A210 USB20BLK manual. */
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/reset.h>
#include <linux/usb/of.h>
#include <linux/usb/otg.h>

#define A210_USB2_ANALOG		0x00
#define A210_USB2_CONFIG		0x04
#define A210_USB31_OTG_CONTROL	0x20
#define A210_USB31_VBUS_COMPARE	BIT(12)
#define A210_USB31_OTG_DISABLE	BIT(4)
#define A210_USB2_TX_VREF	GENMASK(20, 17)
#define A210_USB2_TX_LEVEL	11

struct a210_usb2_variant {
	u32 pulldown_mask;
};

static const struct a210_usb2_variant a210_usb20_phy0 = {
	.pulldown_mask = GENMASK(1, 0),
};

static const struct a210_usb2_variant a210_usb20_phy1 = {
	.pulldown_mask = GENMASK(3, 2),
};

static const struct a210_usb2_variant a210_usb31 = {};

struct a210_usb2_phy {
	const struct a210_usb2_variant *variant;
	void __iomem *base;
	struct clk *clock;
	struct reset_control *reset;
	enum usb_dr_mode mode;
};

static int a210_usb2_init(struct phy *phy)
{
	struct a210_usb2_phy *priv = phy_get_drvdata(phy);
	u32 val;
	int ret;

	ret = clk_prepare_enable(priv->clock);
	if (ret)
		return ret;
	ret = reset_control_assert(priv->reset);
	if (ret)
		goto disable_clock;
	usleep_range(100, 200);
	if (priv->variant->pulldown_mask) {
		val = readl(priv->base + A210_USB2_ANALOG);
		val &= ~A210_USB2_TX_VREF;
		val |= FIELD_PREP(A210_USB2_TX_VREF, A210_USB2_TX_LEVEL);
		writel(val, priv->base + A210_USB2_ANALOG);
		val = readl(priv->base + A210_USB2_CONFIG);
		val &= ~priv->variant->pulldown_mask;
		if (priv->mode == USB_DR_MODE_HOST)
			val |= priv->variant->pulldown_mask;
		writel(val, priv->base + A210_USB2_CONFIG);
	} else {
		/* Board VBUS power is controlled separately by the TCPC. */
		val = readl(priv->base + A210_USB31_OTG_CONTROL);
		val &= ~A210_USB31_OTG_DISABLE;
		val |= A210_USB31_VBUS_COMPARE;
		writel(val, priv->base + A210_USB31_OTG_CONTROL);
	}
	ret = reset_control_deassert(priv->reset);
	if (ret)
		goto disable_clock;
	usleep_range(1000, 1500);
	return 0;

disable_clock:
	clk_disable_unprepare(priv->clock);
	return ret;
}

static int a210_usb2_exit(struct phy *phy)
{
	struct a210_usb2_phy *priv = phy_get_drvdata(phy);

	reset_control_assert(priv->reset);
	clk_disable_unprepare(priv->clock);
	return 0;
}

static const struct phy_ops a210_usb2_ops = {
	.init = a210_usb2_init,
	.exit = a210_usb2_exit,
	.owner = THIS_MODULE,
};

static int a210_usb2_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct a210_usb2_phy *priv;
	struct phy_provider *provider;
	struct phy *phy;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	priv->variant = device_get_match_data(dev);
	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);
	priv->clock = devm_clk_get(dev, "apb");
	if (IS_ERR(priv->clock))
		return dev_err_probe(dev, PTR_ERR(priv->clock),
				     "failed to get register clock\n");
	priv->reset = devm_reset_control_get_exclusive(dev, "phy");
	if (IS_ERR(priv->reset))
		return dev_err_probe(dev, PTR_ERR(priv->reset),
				     "failed to get PHY reset\n");
	priv->mode = of_usb_get_dr_mode_by_phy(dev->of_node, -1);
	if (priv->variant->pulldown_mask &&
	    priv->mode != USB_DR_MODE_HOST &&
	    priv->mode != USB_DR_MODE_PERIPHERAL)
		return dev_err_probe(dev, -EINVAL,
				     "fixed host or device mode required\n");
	phy = devm_phy_create(dev, NULL, &a210_usb2_ops);
	if (IS_ERR(phy))
		return PTR_ERR(phy);
	phy_set_drvdata(phy, priv);
	provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	return PTR_ERR_OR_ZERO(provider);
}

static const struct of_device_id a210_usb2_match[] = {
	{ .compatible = "zhihe,a210-usb2-phy", .data = &a210_usb20_phy0 },
	{ .compatible = "zhihe,a210-usb2-phy1", .data = &a210_usb20_phy1 },
	{ .compatible = "zhihe,a210-usb31-usb2-phy", .data = &a210_usb31 },
	{ }
};
MODULE_DEVICE_TABLE(of, a210_usb2_match);

static struct platform_driver a210_usb2_driver = {
	.probe = a210_usb2_probe,
	.driver = {
		.name = "a210-usb2-phy",
		.of_match_table = a210_usb2_match,
	},
};
module_platform_driver(a210_usb2_driver);

MODULE_DESCRIPTION("Zhihe A210 USB2 femtoPHY driver");
MODULE_LICENSE("GPL");
