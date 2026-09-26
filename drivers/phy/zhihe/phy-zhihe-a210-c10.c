// SPDX-License-Identifier: GPL-2.0-only
/* A210 Synopsys C10 PHY USB path and Type-C orientation switch. */
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/reset.h>
#include <linux/usb/typec_mux.h>

#define TCA_CLOCK_RESET		0x00
#define TCA_RESET_RELEASE	BIT(9)
#define TCA_INTERRUPT_STATUS	0x08
#define TCA_INTERRUPT_CLEAR	GENMASK(15, 0)
#define TCA_GLOBAL_CONFIG	0x10
#define TCA_OPERATION_MODE	GENMASK(1, 0)
#define TCA_SYSTEM_MODE		0
#define TCA_SYNC_MODE		1
#define TCA_TCPC		0x14
#define TCA_TCPC_VALID		BIT(4)
#define TCA_TCPC_LOW_POWER	BIT(3)
#define TCA_TCPC_FLIP		BIT(2)
#define TCA_TCPC_USB		BIT(0)
#define TCA_SYSTEM_CONFIG	0x18
#define TCA_SYSTEM_DISABLE	BIT(3)
#define TCA_SYSTEM_FLIP		BIT(2)
#define TCA_SYSTEM_CONNECTION	GENMASK(1, 0)
#define TCA_SYSTEM_USB_DP	3

struct a210_c10_phy {
	void __iomem *tca;
	struct clk_bulk_data clocks[3];
	struct reset_control *apb_reset;
	struct reset_control *phy_reset;
	struct mutex lock; /* Protects orientation and PHY initialization. */
	enum typec_orientation orientation;
	bool initialized;
};

static void a210_c10_configure(struct a210_c10_phy *priv)
{
	u32 value, tcpc = TCA_TCPC_VALID;

	value = readl(priv->tca + TCA_GLOBAL_CONFIG);
	value &= ~TCA_OPERATION_MODE;
	writel(value | FIELD_PREP(TCA_OPERATION_MODE, TCA_SYSTEM_MODE),
	       priv->tca + TCA_GLOBAL_CONFIG);

	value = readl(priv->tca + TCA_SYSTEM_CONFIG);
	value |= TCA_SYSTEM_DISABLE;
	writel(value, priv->tca + TCA_SYSTEM_CONFIG);
	usleep_range(10, 20);
	value &= ~(TCA_SYSTEM_FLIP | TCA_SYSTEM_CONNECTION);
	value |= FIELD_PREP(TCA_SYSTEM_CONNECTION, TCA_SYSTEM_USB_DP);
	if (priv->orientation == TYPEC_ORIENTATION_REVERSE) {
		value |= TCA_SYSTEM_FLIP;
		tcpc |= TCA_TCPC_FLIP;
	}
	writel(value, priv->tca + TCA_SYSTEM_CONFIG);
	usleep_range(10, 20);
	writel(value & ~TCA_SYSTEM_DISABLE,
	       priv->tca + TCA_SYSTEM_CONFIG);

	value = readl(priv->tca + TCA_GLOBAL_CONFIG);
	value &= ~TCA_OPERATION_MODE;
	writel(value | FIELD_PREP(TCA_OPERATION_MODE, TCA_SYNC_MODE),
	       priv->tca + TCA_GLOBAL_CONFIG);
	writel(TCA_INTERRUPT_CLEAR, priv->tca + TCA_INTERRUPT_STATUS);
	if (priv->orientation == TYPEC_ORIENTATION_NONE)
		tcpc |= TCA_TCPC_LOW_POWER;
	else
		tcpc |= TCA_TCPC_USB;
	/* The controller completes the PHY handshake after leaving reset. */
	writel(tcpc, priv->tca + TCA_TCPC);
}

static int a210_c10_orientation(struct typec_switch_dev *sw,
				enum typec_orientation orientation)
{
	struct a210_c10_phy *priv = typec_switch_get_drvdata(sw);

	mutex_lock(&priv->lock);
	if (priv->orientation != orientation) {
		priv->orientation = orientation;
		if (priv->initialized)
			a210_c10_configure(priv);
	}
	mutex_unlock(&priv->lock);
	return 0;
}

static int a210_c10_init(struct phy *phy)
{
	struct a210_c10_phy *priv = phy_get_drvdata(phy);
	u32 value;
	int ret;

	mutex_lock(&priv->lock);
	ret = clk_bulk_prepare_enable(ARRAY_SIZE(priv->clocks), priv->clocks);
	if (ret)
		goto unlock;
	ret = reset_control_deassert(priv->apb_reset);
	if (ret)
		goto disable_clocks;
	ret = reset_control_assert(priv->phy_reset);
	if (ret)
		goto assert_apb;

	value = readl(priv->tca + TCA_CLOCK_RESET);
	writel(value & ~TCA_RESET_RELEASE, priv->tca + TCA_CLOCK_RESET);
	usleep_range(100, 200);
	writel(value | TCA_RESET_RELEASE, priv->tca + TCA_CLOCK_RESET);
	a210_c10_configure(priv);
	ret = reset_control_deassert(priv->phy_reset);
	if (ret)
		goto assert_apb;
	usleep_range(1000, 1500);
	priv->initialized = true;
	mutex_unlock(&priv->lock);
	return 0;

assert_apb:
	reset_control_assert(priv->apb_reset);
disable_clocks:
	clk_bulk_disable_unprepare(ARRAY_SIZE(priv->clocks), priv->clocks);
unlock:
	mutex_unlock(&priv->lock);
	return ret;
}

static int a210_c10_exit(struct phy *phy)
{
	struct a210_c10_phy *priv = phy_get_drvdata(phy);

	mutex_lock(&priv->lock);
	priv->initialized = false;
	reset_control_assert(priv->phy_reset);
	reset_control_assert(priv->apb_reset);
	clk_bulk_disable_unprepare(ARRAY_SIZE(priv->clocks), priv->clocks);
	mutex_unlock(&priv->lock);
	return 0;
}

static const struct phy_ops a210_c10_ops = {
	.init = a210_c10_init,
	.exit = a210_c10_exit,
	.owner = THIS_MODULE,
};

static void a210_c10_unregister_switch(void *data)
{
	typec_switch_unregister(data);
}

static int a210_c10_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct typec_switch_desc desc = {};
	struct typec_switch_dev *sw;
	struct phy_provider *provider;
	struct a210_c10_phy *priv;
	struct phy *phy;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	mutex_init(&priv->lock);
	priv->tca = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->tca))
		return PTR_ERR(priv->tca);
	priv->clocks[0].id = "apb";
	priv->clocks[1].id = "ref";
	priv->clocks[2].id = "suspend";
	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(priv->clocks), priv->clocks);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get PHY clocks\n");
	priv->apb_reset = devm_reset_control_get_exclusive(dev, "apb");
	if (IS_ERR(priv->apb_reset))
		return dev_err_probe(dev, PTR_ERR(priv->apb_reset),
				     "failed to get APB reset\n");
	priv->phy_reset = devm_reset_control_get_exclusive(dev, "phy");
	if (IS_ERR(priv->phy_reset))
		return dev_err_probe(dev, PTR_ERR(priv->phy_reset),
				     "failed to get PHY reset\n");

	if (device_property_present(dev, "orientation-switch")) {
		desc.drvdata = priv;
		desc.fwnode = dev_fwnode(dev);
		desc.set = a210_c10_orientation;
		sw = typec_switch_register(dev, &desc);
		if (IS_ERR(sw))
			return dev_err_probe(dev, PTR_ERR(sw),
					     "failed to register orientation switch\n");
		ret = devm_add_action_or_reset(dev, a210_c10_unregister_switch, sw);
		if (ret)
			return ret;
	} else {
		priv->orientation = TYPEC_ORIENTATION_NORMAL;
	}
	phy = devm_phy_create(dev, NULL, &a210_c10_ops);
	if (IS_ERR(phy))
		return PTR_ERR(phy);
	phy_set_drvdata(phy, priv);
	provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	return PTR_ERR_OR_ZERO(provider);
}

static const struct of_device_id a210_c10_match[] = {
	{ .compatible = "zhihe,a210-c10-phy" },
	{ }
};
MODULE_DEVICE_TABLE(of, a210_c10_match);

static struct platform_driver a210_c10_driver = {
	.probe = a210_c10_probe,
	.driver = {
		.name = "a210-c10-phy",
		.of_match_table = a210_c10_match,
	},
};
module_platform_driver(a210_c10_driver);

MODULE_DESCRIPTION("Zhihe A210 C10 USB3 PHY and orientation switch");
MODULE_LICENSE("GPL");
