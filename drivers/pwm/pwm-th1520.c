// SPDX-License-Identifier: GPL-2.0
/*
 * T-HEAD TH1520 PWM driver
 *
 * Copyright (C) 2021 Alibaba Group Holding Limited.
 * Copyright (C) 2023 Jisheng Zhang <jszhang@kernel.org>
 * Copyright (C) 2026 Han Gao <gaohan@iscas.ac.cn>
 *
 * C implementation of the upstream Rust pwm_th1520 driver, for kernels
 * without the Rust PWM abstractions.
 *
 * Limitations:
 * - The period and duty cycle are controlled by 32-bit hardware registers,
 *   limiting the maximum resolution.
 * - Only continuous output mode is supported.
 * - The controller provides up to 6 PWM channels.
 * - Reconfiguration is glitch free: new period and duty cycle values are
 *   latched and take effect at the start of the next period.
 * - Polarity is handled by a simple hardware inversion bit.
 * - A channel is disabled by setting its duty cycle to zero, which produces
 *   a static low output. Clearing the START bit does not reliably force the
 *   inactive level, so it is not used.
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/limits.h>
#include <linux/math64.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/time64.h>

#define TH1520_PWM_MAX_NUM		6

#define TH1520_PWM_CHN_BASE(n)		((n) * 0x20)
#define TH1520_PWM_CTRL(n)		(TH1520_PWM_CHN_BASE(n) + 0x00)
#define TH1520_PWM_PER(n)		(TH1520_PWM_CHN_BASE(n) + 0x08)
#define TH1520_PWM_FP(n)		(TH1520_PWM_CHN_BASE(n) + 0x0c)

#define TH1520_PWM_START		BIT(0)
#define TH1520_PWM_CFG_UPDATE		BIT(2)
#define TH1520_PWM_CONTINUOUS_MODE	BIT(5)
#define TH1520_PWM_FPOUT		BIT(8)

struct th1520_pwm {
	void __iomem *base;
	struct clk *clk;
	u64 rate;
};

static inline struct th1520_pwm *to_th1520_pwm(struct pwm_chip *chip)
{
	return pwmchip_get_drvdata(chip);
}

static u32 th1520_pwm_ns_to_cycles(u64 ns, u64 rate)
{
	return min_t(u64, mul_u64_u64_div_u64(ns, rate, NSEC_PER_SEC), U32_MAX);
}

static u64 th1520_pwm_cycles_to_ns(u32 cycles, u64 rate)
{
	return DIV64_U64_ROUND_UP((u64)cycles * NSEC_PER_SEC, rate);
}

static int th1520_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			    const struct pwm_state *state)
{
	struct th1520_pwm *priv = to_th1520_pwm(chip);
	unsigned int ch = pwm->hwpwm;
	u32 period, duty, ctrl;
	bool was_enabled;

	was_enabled = readl(priv->base + TH1520_PWM_FP(ch)) != 0;

	if (!state->enabled || !state->period) {
		if (was_enabled) {
			writel(0, priv->base + TH1520_PWM_CTRL(ch));
			writel(0, priv->base + TH1520_PWM_FP(ch));
			writel(TH1520_PWM_CFG_UPDATE, priv->base + TH1520_PWM_CTRL(ch));
		}
		return 0;
	}

	period = th1520_pwm_ns_to_cycles(state->period, priv->rate);
	if (!period)
		period = 1;

	duty = th1520_pwm_ns_to_cycles(state->duty_cycle, priv->rate);
	if (duty > period)
		duty = period;

	ctrl = TH1520_PWM_CONTINUOUS_MODE;
	if (state->polarity == PWM_POLARITY_NORMAL)
		ctrl |= TH1520_PWM_FPOUT;

	writel(ctrl, priv->base + TH1520_PWM_CTRL(ch));
	writel(period, priv->base + TH1520_PWM_PER(ch));
	writel(duty, priv->base + TH1520_PWM_FP(ch));
	writel(ctrl | TH1520_PWM_CFG_UPDATE, priv->base + TH1520_PWM_CTRL(ch));

	/*
	 * The START bit must be written in a separate, final transaction, and
	 * only when enabling the channel from a disabled state.
	 */
	if (!was_enabled)
		writel(ctrl | TH1520_PWM_START, priv->base + TH1520_PWM_CTRL(ch));

	return 0;
}

static int th1520_pwm_get_state(struct pwm_chip *chip, struct pwm_device *pwm,
				struct pwm_state *state)
{
	struct th1520_pwm *priv = to_th1520_pwm(chip);
	unsigned int ch = pwm->hwpwm;
	u32 ctrl, period, duty;

	ctrl = readl(priv->base + TH1520_PWM_CTRL(ch));
	period = readl(priv->base + TH1520_PWM_PER(ch));
	duty = readl(priv->base + TH1520_PWM_FP(ch));

	state->polarity = (ctrl & TH1520_PWM_FPOUT) ? PWM_POLARITY_NORMAL :
						       PWM_POLARITY_INVERSED;

	if (!period) {
		state->enabled = false;
		state->period = 0;
		state->duty_cycle = 0;
		return 0;
	}

	state->enabled = duty != 0;
	state->period = th1520_pwm_cycles_to_ns(period, priv->rate);
	state->duty_cycle = th1520_pwm_cycles_to_ns(min(duty, period), priv->rate);

	return 0;
}

static const struct pwm_ops th1520_pwm_ops = {
	.apply = th1520_pwm_apply,
	.get_state = th1520_pwm_get_state,
};

static int th1520_pwm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct th1520_pwm *priv;
	struct pwm_chip *chip;
	int ret;

	chip = devm_pwmchip_alloc(dev, TH1520_PWM_MAX_NUM, sizeof(*priv));
	if (IS_ERR(chip))
		return PTR_ERR(chip);
	priv = to_th1520_pwm(chip);

	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	priv->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(priv->clk))
		return dev_err_probe(dev, PTR_ERR(priv->clk),
				     "failed to get the clock\n");

	ret = devm_clk_rate_exclusive_get(dev, priv->clk);
	if (ret)
		return dev_err_probe(dev, ret, "failed to lock the clock rate\n");

	priv->rate = clk_get_rate(priv->clk);
	if (!priv->rate)
		return dev_err_probe(dev, -EINVAL, "clock rate is zero\n");
	if (priv->rate > NSEC_PER_SEC)
		return dev_err_probe(dev, -EINVAL,
				     "clock rate %llu Hz is too high\n", priv->rate);

	chip->ops = &th1520_pwm_ops;

	ret = devm_pwmchip_add(dev, chip);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add PWM chip\n");

	return 0;
}

static const struct of_device_id th1520_pwm_of_match[] = {
	{ .compatible = "thead,th1520-pwm" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, th1520_pwm_of_match);

static struct platform_driver th1520_pwm_driver = {
	.driver = {
		.name = "pwm-th1520",
		.of_match_table = th1520_pwm_of_match,
	},
	.probe = th1520_pwm_probe,
};
module_platform_driver(th1520_pwm_driver);

MODULE_AUTHOR("Wei Liu <lw312886@linux.alibaba.com>");
MODULE_AUTHOR("Jisheng Zhang <jszhang@kernel.org>");
MODULE_AUTHOR("Han Gao <gaohan@iscas.ac.cn>");
MODULE_DESCRIPTION("T-HEAD TH1520 PWM driver");
MODULE_LICENSE("GPL");
