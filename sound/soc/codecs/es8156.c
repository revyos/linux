// SPDX-License-Identifier: GPL-2.0-only
/*
 * Everest Semiconductor ES8156 stereo DAC.
 * Register initialization adapted from the Everest Semiconductor driver.
 * Copyright Everest Semiconductor Co., Ltd.
 */
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#define ES8156_RESET		0x00
#define ES8156_CLOCK_MODE	0x02
#define ES8156_SERIAL_FORMAT	0x11
#define ES8156_MUTE		0x13
#define ES8156_VOLUME		0x14
#define ES8156_WORD_LENGTH	GENMASK(6, 4)
#define ES8156_WORD_32		0x40
#define ES8156_MUTE_CHANNELS	GENMASK(2, 1)

struct es8156 {
	struct regmap *regmap;
	unsigned int sysclk;
};

/* External MCLK, 256fs, I2S consumer, differential output at 3.3 V. */
static const struct reg_sequence es8156_init[] = {
	{ ES8156_RESET, 0x1c, 5000 },
	{ ES8156_RESET, 0x03 },
	{ ES8156_CLOCK_MODE, 0x04 },
	{ 0x19, 0x20 },
	{ 0x20, 0x2a }, { 0x21, 0x3c }, { 0x22, 0x02 },
	{ 0x24, 0x07 }, { 0x23, 0x40 },
	{ 0x0a, 0x01 }, { 0x0b, 0x01 },
	{ 0x01, 0x21 }, { 0x09, 0x00 },
	{ 0x03, 0x01 }, { 0x04, 0x00 }, { 0x05, 0x08 },
	{ ES8156_SERIAL_FORMAT, ES8156_WORD_32 },
	{ ES8156_MUTE, ES8156_MUTE_CHANNELS },
	{ ES8156_VOLUME, 143 }, /* -24 dB until userspace selects a level. */
	{ 0x0d, 0x14 }, { 0x18, 0x00 }, { 0x08, 0x3f },
	{ ES8156_RESET, 0x02 }, { ES8156_RESET, 0x03 },
	{ 0x25, 0x20 },
};

static int es8156_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	if ((fmt & SND_SOC_DAIFMT_FORMAT_MASK) != SND_SOC_DAIFMT_I2S ||
	    (fmt & SND_SOC_DAIFMT_INV_MASK) != SND_SOC_DAIFMT_NB_NF ||
	    (fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) != SND_SOC_DAIFMT_BC_FC)
		return -EINVAL;
	return 0;
}

static int es8156_set_sysclk(struct snd_soc_dai *dai, int id,
			     unsigned int rate, int direction)
{
	struct es8156 *es8156 = snd_soc_component_get_drvdata(dai->component);

	if (id || direction != SND_SOC_CLOCK_IN)
		return -EINVAL;
	es8156->sysclk = rate;
	return 0;
}

static int es8156_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	struct es8156 *es8156 = snd_soc_component_get_drvdata(dai->component);

	if (es8156->sysclk != params_rate(params) * 256 ||
	    params_format(params) != SNDRV_PCM_FORMAT_S32_LE)
		return -EINVAL;
	return regmap_update_bits(es8156->regmap, ES8156_SERIAL_FORMAT,
				  ES8156_WORD_LENGTH, ES8156_WORD_32);
}

static int es8156_mute(struct snd_soc_dai *dai, int mute, int stream)
{
	struct es8156 *es8156 = snd_soc_component_get_drvdata(dai->component);

	return regmap_update_bits(es8156->regmap, ES8156_MUTE,
				  ES8156_MUTE_CHANNELS,
				  mute ? ES8156_MUTE_CHANNELS : 0);
}

static const struct snd_soc_dai_ops es8156_dai_ops = {
	.set_fmt = es8156_set_fmt,
	.set_sysclk = es8156_set_sysclk,
	.hw_params = es8156_hw_params,
	.mute_stream = es8156_mute,
	.no_capture_mute = 1,
};

static struct snd_soc_dai_driver es8156_dai = {
	.name = "es8156-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_96000,
		.formats = SNDRV_PCM_FMTBIT_S32_LE,
	},
	.ops = &es8156_dai_ops,
};

static const DECLARE_TLV_DB_SCALE(es8156_volume_tlv, -9550, 50, 1);
static const struct snd_kcontrol_new es8156_controls[] = {
	SOC_SINGLE_TLV("DAC Playback Volume", ES8156_VOLUME, 0, 191, 0,
		       es8156_volume_tlv),
};

static const struct snd_soc_dapm_widget es8156_widgets[] = {
	SND_SOC_DAPM_DAC("DAC", "Playback", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_OUTPUT("LOUT"),
	SND_SOC_DAPM_OUTPUT("ROUT"),
};

static const struct snd_soc_dapm_route es8156_routes[] = {
	{ "LOUT", NULL, "DAC" },
	{ "ROUT", NULL, "DAC" },
};

static const struct snd_soc_component_driver es8156_component = {
	.controls = es8156_controls,
	.num_controls = ARRAY_SIZE(es8156_controls),
	.dapm_widgets = es8156_widgets,
	.num_dapm_widgets = ARRAY_SIZE(es8156_widgets),
	.dapm_routes = es8156_routes,
	.num_dapm_routes = ARRAY_SIZE(es8156_routes),
	.use_pmdown_time = 1,
	.endianness = 1,
};

static bool es8156_volatile(struct device *dev, unsigned int reg)
{
	return reg == ES8156_RESET || reg == 0x0c || reg >= 0xfc;
}

static const struct regmap_config es8156_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xff,
	.cache_type = REGCACHE_MAPLE,
	.volatile_reg = es8156_volatile,
};

static int es8156_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	static const char * const supplies[] = { "dvdd", "pvdd" };
	struct es8156 *es8156;
	int ret, avdd;

	es8156 = devm_kzalloc(dev, sizeof(*es8156), GFP_KERNEL);
	if (!es8156)
		return -ENOMEM;
	avdd = devm_regulator_get_enable_read_voltage(dev, "avdd");
	if (avdd < 0)
		return dev_err_probe(dev, avdd, "failed to enable analog supply\n");
	ret = devm_regulator_bulk_get_enable(dev, ARRAY_SIZE(supplies), supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable supplies\n");
	usleep_range(1000, 2000);
	es8156->regmap = devm_regmap_init_i2c(client, &es8156_regmap);
	if (IS_ERR(es8156->regmap))
		return dev_err_probe(dev, PTR_ERR(es8156->regmap),
				     "failed to create register map\n");
	i2c_set_clientdata(client, es8156);
	ret = regmap_multi_reg_write(es8156->regmap, es8156_init,
				     ARRAY_SIZE(es8156_init));
	if (ret)
		return dev_err_probe(dev, ret, "failed to initialize DAC\n");
	if (avdd <= 1800000) {
		ret = regmap_update_bits(es8156->regmap, 0x23, 0x30, 0x30);
		if (ret)
			return ret;
	}
	return devm_snd_soc_register_component(dev, &es8156_component,
					     &es8156_dai, 1);
}

static const struct of_device_id es8156_of_match[] = {
	{ .compatible = "everest,es8156" },
	{ }
};
MODULE_DEVICE_TABLE(of, es8156_of_match);

static struct i2c_driver es8156_driver = {
	.driver = {
		.name = "es8156",
		.of_match_table = es8156_of_match,
	},
	.probe = es8156_probe,
};
module_i2c_driver(es8156_driver);
MODULE_DESCRIPTION("Everest Semiconductor ES8156 stereo DAC");
MODULE_LICENSE("GPL");
