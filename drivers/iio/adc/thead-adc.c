// SPDX-License-Identifier: GPL-2.0-only
/* Single-conversion driver for the T-Head ADC used in TH1520 and A210. */
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/iio/iio.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>

#define ADC_PHY_CONFIG		0x00
#define ADC_PHY_CONTROL		0x04
#define ADC_OPERATION		0x0c
#define ADC_START		0x10
#define ADC_CLOCK		0x14
#define ADC_START_TIME		0x18
#define ADC_SAMPLE_TIME		0x1c
#define ADC_DATA		0x20
#define ADC_IRQ_MASK		0x50
#define ADC_DELTA_IRQ_MASK	0x54
#define ADC_RESOLUTION_12	3
#define ADC_ENABLE		BIT(0)
#define ADC_RESET		BIT(4)
#define ADC_SINGLE		BIT(0)
#define ADC_CHANNEL(n)		BIT(12 + (n))
#define ADC_CLOCK_PHASE		BIT(16)
#define ADC_CLOCK_DIVIDER	GENMASK(6, 0)
#define ADC_MAX_CLOCK		15625000
#define ADC_SAMPLE_RATE		1000000
#define ADC_ALL_IRQS		GENMASK(3, 0)
#define ADC_VALUE		GENMASK(11, 0)
#define ADC_CHANNEL_ID		GENMASK(14, 12)
#define ADC_VALID		BIT(15)

struct thead_adc {
	void __iomem *base;
	struct clk *clk;
	struct reset_control *reset;
	struct regulator *vref;
	struct mutex lock; /* Serializes conversions and FIFO reads. */
};

static int thead_adc_resume(struct device *dev)
{
	struct thead_adc *adc = iio_priv(dev_get_drvdata(dev));
	unsigned long rate, divider;
	int ret;

	ret = regulator_enable(adc->vref);
	if (ret)
		return ret;
	ret = clk_prepare_enable(adc->clk);
	if (ret)
		goto disable_supply;
	ret = reset_control_deassert(adc->reset);
	if (ret)
		goto disable_clock;

	rate = clk_get_rate(adc->clk);
	/* Odd divisors other than one do not provide a valid duty cycle. */
	divider = max(2UL, round_up(DIV_ROUND_UP(rate, ADC_MAX_CLOCK), 2));
	if (!rate || divider > FIELD_MAX(ADC_CLOCK_DIVIDER)) {
		ret = -EINVAL;
		goto assert_reset;
	}
	rate /= divider;
	writel(ADC_RESET, adc->base + ADC_PHY_CONTROL);
	writel(0, adc->base + ADC_PHY_CONTROL);
	/* Single-ended, external reference, 12-bit conversions. */
	writel(ADC_RESOLUTION_12, adc->base + ADC_PHY_CONFIG);
	writel(ADC_CLOCK_PHASE | FIELD_PREP(ADC_CLOCK_DIVIDER, divider),
	       adc->base + ADC_CLOCK);
	writel(DIV_ROUND_UP(rate, 200000), adc->base + ADC_START_TIME);
	writel(DIV_ROUND_UP(rate, ADC_SAMPLE_RATE),
	       adc->base + ADC_SAMPLE_TIME);
	writel(ADC_ALL_IRQS, adc->base + ADC_IRQ_MASK);
	writel(ADC_ALL_IRQS, adc->base + ADC_DELTA_IRQ_MASK);
	return 0;

assert_reset:
	reset_control_assert(adc->reset);
disable_clock:
	clk_disable_unprepare(adc->clk);
disable_supply:
	regulator_disable(adc->vref);
	return ret;
}

static int thead_adc_suspend(struct device *dev)
{
	struct thead_adc *adc = iio_priv(dev_get_drvdata(dev));

	writel(ADC_RESET, adc->base + ADC_PHY_CONTROL);
	reset_control_assert(adc->reset);
	clk_disable_unprepare(adc->clk);
	regulator_disable(adc->vref);
	return 0;
}

static int thead_adc_read_raw(struct iio_dev *indio_dev,
			      const struct iio_chan_spec *channel,
			      int *val, int *val2, long mask)
{
	struct device *dev = indio_dev->dev.parent;
	struct thead_adc *adc = iio_priv(indio_dev);
	u32 data;
	int ret;

	if (mask == IIO_CHAN_INFO_SCALE) {
		ret = regulator_get_voltage(adc->vref);
		if (ret < 0)
			return ret;
		*val = ret / 1000;
		*val2 = 12;
		return IIO_VAL_FRACTIONAL_LOG2;
	}
	if (mask != IIO_CHAN_INFO_RAW)
		return -EINVAL;
	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0)
		return ret;

	mutex_lock(&adc->lock);
	writel(ADC_SINGLE | ADC_CHANNEL(channel->channel),
	       adc->base + ADC_OPERATION);
	writel(ADC_ENABLE, adc->base + ADC_PHY_CONTROL);
	writel(ADC_SINGLE, adc->base + ADC_START);
	ret = readl_poll_timeout(adc->base + ADC_DATA, data,
				 (data & ADC_VALID) || (data & (ADC_VALID << 16)),
				 10, 100000);
	writel(0, adc->base + ADC_PHY_CONTROL);
	if (!ret) {
		if (!(data & ADC_VALID) ||
		    FIELD_GET(ADC_CHANNEL_ID, data) != channel->channel)
			data >>= 16;
		if (!(data & ADC_VALID) ||
		    FIELD_GET(ADC_CHANNEL_ID, data) != channel->channel) {
			ret = -EIO;
		} else {
			*val = FIELD_GET(ADC_VALUE, data);
			ret = IIO_VAL_INT;
		}
	}
	mutex_unlock(&adc->lock);
	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);
	return ret;
}

#define THEAD_ADC_CHANNEL(n) { \
	.type = IIO_VOLTAGE, .indexed = 1, .channel = (n), \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE), \
}

static const struct iio_chan_spec thead_adc_channels[] = {
	THEAD_ADC_CHANNEL(0), THEAD_ADC_CHANNEL(1),
	THEAD_ADC_CHANNEL(2), THEAD_ADC_CHANNEL(3),
	THEAD_ADC_CHANNEL(4), THEAD_ADC_CHANNEL(5),
	THEAD_ADC_CHANNEL(6), THEAD_ADC_CHANNEL(7),
};

static const struct iio_info thead_adc_info = {
	.read_raw = thead_adc_read_raw,
};

static const unsigned int th1520_channels = 8;
static const unsigned int a210_channels = 4;

static void thead_adc_disable(void *data)
{
	struct device *dev = data;

	pm_runtime_disable(dev);
	if (!pm_runtime_status_suspended(dev))
		thead_adc_suspend(dev);
}

static int thead_adc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct iio_dev *indio_dev;
	struct thead_adc *adc;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*adc));
	if (!indio_dev)
		return -ENOMEM;
	adc = iio_priv(indio_dev);
	mutex_init(&adc->lock);
	adc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(adc->base))
		return PTR_ERR(adc->base);
	adc->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(adc->clk))
		return dev_err_probe(dev, PTR_ERR(adc->clk), "missing clock\n");
	adc->reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(adc->reset))
		return dev_err_probe(dev, PTR_ERR(adc->reset), "missing reset\n");
	adc->vref = devm_regulator_get(dev, "vref");
	if (IS_ERR(adc->vref))
		return dev_err_probe(dev, PTR_ERR(adc->vref), "missing reference\n");

	indio_dev->name = "thead-adc";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &thead_adc_info;
	indio_dev->channels = thead_adc_channels;
	indio_dev->num_channels = *(const unsigned int *)device_get_match_data(dev);
	platform_set_drvdata(pdev, indio_dev);
	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	ret = pm_runtime_set_suspended(dev);
	if (ret)
		return ret;
	pm_runtime_enable(dev);
	ret = devm_add_action_or_reset(dev, thead_adc_disable, dev);
	if (ret)
		return ret;
	return devm_iio_device_register(dev, indio_dev);
}

static const struct dev_pm_ops thead_adc_pm_ops = {
	RUNTIME_PM_OPS(thead_adc_suspend, thead_adc_resume, NULL)
	SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend, pm_runtime_force_resume)
};

static const struct of_device_id thead_adc_match[] = {
	{ .compatible = "thead,th1520-adc", .data = &th1520_channels },
	{ .compatible = "zhihe,a210-adc", .data = &a210_channels },
	{ }
};
MODULE_DEVICE_TABLE(of, thead_adc_match);

static struct platform_driver thead_adc_driver = {
	.probe = thead_adc_probe,
	.driver = {
		.name = "thead-adc",
		.of_match_table = thead_adc_match,
		.pm = pm_ptr(&thead_adc_pm_ops),
	},
};
module_platform_driver(thead_adc_driver);

MODULE_DESCRIPTION("T-Head TH1520 and Zhihe A210 ADC");
MODULE_LICENSE("GPL");
