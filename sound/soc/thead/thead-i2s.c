// SPDX-License-Identifier: GPL-2.0-only
/*
 * T-Head stereo I2S controller.
 * Register definitions derived from the Zhihe A210 driver,
 * Copyright (C) 2024 Zhihe Computing Technology (Shenzhen) Co., Ltd.
 */
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/spinlock.h>
#include <sound/dmaengine_pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#define I2S_ENABLE	0x00
#define I2S_MODE	0x04
#define I2S_RX_CONFIG	0x08
#define I2S_FORMAT	0x0c
#define I2S_TX_CONFIG	0x10
#define I2S_IRQ_MASK	0x30
#define I2S_IRQ_CLEAR	0x3c
#define I2S_DMA_ENABLE	0x40
#define I2S_TX_WATERMARK	0x44
#define I2S_RX_WATERMARK	0x48
#define I2S_DATA	0x4c
#define I2S_MCLK_DIV	0x50
#define I2S_REF_DIV	0x54
#define I2S_IRQ_ALL	GENMASK(20, 0)
#define I2S_TX_MODE	BIT(0)
#define I2S_TX_WRITE	BIT(1)
#define I2S_RX_MODE	BIT(4)
#define I2S_RX_WRITE	BIT(5)
#define I2S_CHANNEL0	BIT(8)
#define I2S_RX_MASTER	BIT(8)
#define I2S_DATA_WIDTH	GENMASK(11, 8)
#define I2S_WIDTH_32	0xa
#define I2S_BCLK_RATIO	GENMASK(13, 12)
#define I2S_BCLK_64	2
#define I2S_DMA_RX	BIT(0)
#define I2S_DMA_TX	BIT(1)
#define I2S_MCLK_RATIO	256
#define I2S_REF_48K	294912000UL
#define I2S_REF_44K	316108800UL

struct thead_i2s {
	void __iomem *base;
	struct regmap *syscon;
	u32 enable[2];
	struct clk_bulk_data clocks[2];
	struct reset_control *reset;
	struct snd_dmaengine_dai_dma_data tx, rx;
	spinlock_t lock; /* Protects shared DMA and enable registers. */
	unsigned int sysclk;
};

static int thead_i2s_runtime_resume(struct device *dev)
{
	struct thead_i2s *i2s = dev_get_drvdata(dev);
	int ret;

	ret = clk_bulk_prepare_enable(ARRAY_SIZE(i2s->clocks), i2s->clocks);
	if (ret)
		return ret;
	ret = reset_control_deassert(i2s->reset);
	if (ret) {
		clk_bulk_disable_unprepare(ARRAY_SIZE(i2s->clocks), i2s->clocks);
		return ret;
	}
	ret = regmap_update_bits(i2s->syscon, i2s->enable[0],
				 i2s->enable[1], i2s->enable[1]);
	if (ret) {
		reset_control_assert(i2s->reset);
		clk_bulk_disable_unprepare(ARRAY_SIZE(i2s->clocks), i2s->clocks);
		return ret;
	}
	writel(0, i2s->base + I2S_ENABLE);
	writel(0, i2s->base + I2S_DMA_ENABLE);
	writel(I2S_IRQ_ALL, i2s->base + I2S_IRQ_MASK);
	writel(~0U, i2s->base + I2S_IRQ_CLEAR);
	writel(I2S_CHANNEL0 | I2S_TX_WRITE | I2S_RX_WRITE,
	       i2s->base + I2S_MODE);
	writel(I2S_RX_MASTER, i2s->base + I2S_RX_CONFIG);
	writel(0, i2s->base + I2S_TX_CONFIG);
	writel(16, i2s->base + I2S_TX_WATERMARK);
	writel(4, i2s->base + I2S_RX_WATERMARK);
	return 0;
}

static int thead_i2s_runtime_suspend(struct device *dev)
{
	struct thead_i2s *i2s = dev_get_drvdata(dev);

	writel(0, i2s->base + I2S_ENABLE);
	regmap_update_bits(i2s->syscon, i2s->enable[0], i2s->enable[1], 0);
	reset_control_assert(i2s->reset);
	clk_bulk_disable_unprepare(ARRAY_SIZE(i2s->clocks), i2s->clocks);
	return 0;
}

static int thead_i2s_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	if ((fmt & SND_SOC_DAIFMT_FORMAT_MASK) != SND_SOC_DAIFMT_I2S ||
	    (fmt & SND_SOC_DAIFMT_INV_MASK) != SND_SOC_DAIFMT_NB_NF ||
	    (fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) != SND_SOC_DAIFMT_BP_FP)
		return -EINVAL;
	return 0;
}

static int thead_i2s_set_sysclk(struct snd_soc_dai *dai, int id,
				unsigned int rate, int direction)
{
	struct thead_i2s *i2s = snd_soc_dai_get_drvdata(dai);

	if (id || direction != SND_SOC_CLOCK_OUT)
		return -EINVAL;
	i2s->sysclk = rate;
	return 0;
}

static int thead_i2s_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *params,
			      struct snd_soc_dai *dai)
{
	struct thead_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	unsigned long mclk = params_rate(params) * I2S_MCLK_RATIO;
	unsigned long source, divider;
	int ret;

	if (params_format(params) != SNDRV_PCM_FORMAT_S32_LE ||
	    params_channels(params) != 2 ||
	    (i2s->sysclk && i2s->sysclk != mclk))
		return -EINVAL;
	source = params_rate(params) % 11025 ? I2S_REF_48K : I2S_REF_44K;
	ret = clk_set_rate(i2s->clocks[1].clk, source);
	if (ret)
		return ret;
	source = clk_get_rate(i2s->clocks[1].clk);
	if (!source)
		return -EINVAL;
	divider = DIV_ROUND_CLOSEST(source, mclk);
	if (!divider || divider > U8_MAX)
		return -EINVAL;

	writel(FIELD_PREP(I2S_DATA_WIDTH, I2S_WIDTH_32) |
	       FIELD_PREP(I2S_BCLK_RATIO, I2S_BCLK_64),
	       i2s->base + I2S_FORMAT);
	writel(divider, i2s->base + I2S_MCLK_DIV);
	writel(3, i2s->base + I2S_REF_DIV);
	return 0;
}

static int thead_i2s_startup(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	int ret = pm_runtime_resume_and_get(dai->dev);

	return ret < 0 ? ret : 0;
}

static void thead_i2s_shutdown(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *dai)
{
	pm_runtime_mark_last_busy(dai->dev);
	pm_runtime_put_autosuspend(dai->dev);
}

static int thead_i2s_trigger(struct snd_pcm_substream *substream,
			     int command, struct snd_soc_dai *dai)
{
	struct thead_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	unsigned int mask = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ?
			    I2S_DMA_TX : I2S_DMA_RX;
	unsigned long flags;
	bool start;
	u32 enabled;

	switch (command) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		start = true;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		start = false;
		break;
	default:
		return -EINVAL;
	}
	spin_lock_irqsave(&i2s->lock, flags);
	enabled = readl(i2s->base + I2S_DMA_ENABLE);
	enabled = start ? enabled | mask : enabled & ~mask;
	writel(I2S_CHANNEL0 | I2S_TX_WRITE | I2S_RX_WRITE |
	       (enabled ? I2S_TX_MODE | I2S_RX_MODE : 0),
	       i2s->base + I2S_MODE);
	writel(enabled, i2s->base + I2S_DMA_ENABLE);
	writel(!!enabled, i2s->base + I2S_ENABLE);
	spin_unlock_irqrestore(&i2s->lock, flags);
	return 0;
}

static int thead_i2s_dai_probe(struct snd_soc_dai *dai)
{
	struct thead_i2s *i2s = snd_soc_dai_get_drvdata(dai);

	snd_soc_dai_init_dma_data(dai, &i2s->tx, &i2s->rx);
	return 0;
}

static const struct snd_soc_dai_ops thead_i2s_ops = {
	.probe = thead_i2s_dai_probe,
	.set_fmt = thead_i2s_set_fmt,
	.set_sysclk = thead_i2s_set_sysclk,
	.hw_params = thead_i2s_hw_params,
	.startup = thead_i2s_startup,
	.shutdown = thead_i2s_shutdown,
	.trigger = thead_i2s_trigger,
};

static struct snd_soc_dai_driver thead_i2s_dai = {
	.name = "thead-i2s",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_96000,
		.formats = SNDRV_PCM_FMTBIT_S32_LE,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_96000,
		.formats = SNDRV_PCM_FMTBIT_S32_LE,
	},
	.symmetric_rate = 1,
	.ops = &thead_i2s_ops,
};

static const struct snd_soc_component_driver thead_i2s_component = {
	.name = "thead-i2s",
	.legacy_dai_naming = 1,
	/* Drain DMA while the peripheral still accepts transfers. */
	.trigger_stop = SND_SOC_TRIGGER_ORDER_LDC,
};

static void thead_i2s_disable(void *data)
{
	struct device *dev = data;

	pm_runtime_disable(dev);
	if (!pm_runtime_status_suspended(dev))
		thead_i2s_runtime_suspend(dev);
}

static int thead_i2s_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct thead_i2s *i2s;
	struct resource *res;
	int ret;

	i2s = devm_kzalloc(dev, sizeof(*i2s), GFP_KERNEL);
	if (!i2s)
		return -ENOMEM;
	i2s->base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(i2s->base))
		return PTR_ERR(i2s->base);
	i2s->syscon = syscon_regmap_lookup_by_phandle_args(dev->of_node,
							   "zhihe,syscon", 2,
							   i2s->enable);
	if (IS_ERR(i2s->syscon))
		return dev_err_probe(dev, PTR_ERR(i2s->syscon), "missing syscon\n");
	i2s->clocks[0].id = "pclk";
	i2s->clocks[1].id = "mclk";
	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(i2s->clocks), i2s->clocks);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get clocks\n");
	i2s->reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(i2s->reset))
		return dev_err_probe(dev, PTR_ERR(i2s->reset), "missing reset\n");
	i2s->tx.addr = res->start + I2S_DATA;
	i2s->tx.addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	i2s->tx.maxburst = 1;
	i2s->rx = i2s->tx;
	spin_lock_init(&i2s->lock);
	platform_set_drvdata(pdev, i2s);
	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_enable(dev);
	ret = devm_add_action_or_reset(dev, thead_i2s_disable, dev);
	if (ret)
		return ret;
	ret = devm_snd_dmaengine_pcm_register(dev, NULL, 0);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register PCM\n");
	return devm_snd_soc_register_component(dev, &thead_i2s_component,
					     &thead_i2s_dai, 1);
}

static const struct dev_pm_ops thead_i2s_pm_ops = {
	RUNTIME_PM_OPS(thead_i2s_runtime_suspend, thead_i2s_runtime_resume, NULL)
	SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend, pm_runtime_force_resume)
};

static const struct of_device_id thead_i2s_match[] = {
	{ .compatible = "zhihe,a210-i2s" },
	{ }
};
MODULE_DEVICE_TABLE(of, thead_i2s_match);

static struct platform_driver thead_i2s_driver = {
	.probe = thead_i2s_probe,
	.driver = {
		.name = "thead-i2s",
		.of_match_table = thead_i2s_match,
		.pm = pm_ptr(&thead_i2s_pm_ops),
	},
};
module_platform_driver(thead_i2s_driver);
MODULE_DESCRIPTION("T-Head stereo I2S controller");
MODULE_LICENSE("GPL");
