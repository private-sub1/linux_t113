// SPDX-License-Identifier: GPL-2.0
/*
 * PWM Controller Driver for sunxi platforms (D1, T113-S3 and R329)
 *
 * Limitations:
 * - When the parameters change, current running period will not be completed
 *   and run new settings immediately.
 * - It output HIGH-Z state when PWM channel disabled.
 *
 * Copyright (c) 2023 Aleksandr Shubin <privatesub2@gmail.com>
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/reset.h>

#define SUN20I_PWM_CLK_CFG(chan)		(0x20 + ((chan) * 0x4))
#define SUN20I_PWM_CLK_CFG_SRC			GENMASK(8, 7)
#define SUN20I_PWM_CLK_CFG_DIV_M		GENMASK(3, 0)
#define SUN20I_PWM_CLK_DIV_M_MAX		8

#define SUN20I_PWM_CLK_GATE			0x40
#define SUN20I_PWM_CLK_GATE_BYPASS(chan)	BIT((chan) + 16)
#define SUN20I_PWM_CLK_GATE_GATING(chan)	BIT(chan)

#define SUN20I_PWM_ENABLE			0x80
#define SUN20I_PWM_ENABLE_EN(chan)		BIT(chan)

#define SUN20I_PWM_CTL(chan)			(0x100 + (chan) * 0x20)
#define SUN20I_PWM_CTL_ACT_STA			BIT(8)
#define SUN20I_PWM_CTL_PRESCAL_K		GENMASK(7, 0)
#define SUN20I_PWM_CTL_PRESCAL_K_MAX		field_max(SUN20I_PWM_CTL_PRESCAL_K)

#define SUN20I_PWM_PERIOD(chan)			(0x104 + (chan) * 0x20)
#define SUN20I_PWM_PERIOD_ENTIRE_CYCLE		GENMASK(31, 16)
#define SUN20I_PWM_PERIOD_ACT_CYCLE		GENMASK(15, 0)

#define SUN20I_PWM_PCNTR_SIZE			BIT(16)

/*
 * SUN20I_PWM_MAGIC is used to quickly compute the values of the clock dividers
 * div_m (SUN20I_PWM_CLK_CFG_DIV_M) & prescale_k (SUN20I_PWM_CTL_PRESCAL_K)
 * without using a loop. These dividers limit the # of cycles in a period
 * to SUN20I_PWM_PCNTR_SIZE by applying a scaling factor of
 * 1/(div_m * (prescale_k + 1)) to the clock source.
 *
 * SUN20I_PWM_MAGIC is derived by solving for div_m and prescale_k
 * such that for a given requested period,
 *
 * i) div_m is minimized for any prescale_k ≤ SUN20I_PWM_CTL_PRESCAL_K_MAX,
 * ii) prescale_k is minimized.
 *
 * The derivation proceeds as follows, with val = # of cycles for requested
 * period:
 *
 * for a given value of div_m we want the smallest prescale_k such that
 *
 * (val >> div_m) // (prescale_k + 1) ≤ 65536 (SUN20I_PWM_PCNTR_SIZE)
 *
 * This is equivalent to:
 *
 * (val >> div_m) ≤ 65536 * (prescale_k + 1) + prescale_k
 * ⟺ (val >> div_m) ≤ 65537 * prescale_k + 65536
 * ⟺ (val >> div_m) - 65536 ≤ 65537 * prescale_k
 * ⟺ ((val >> div_m) - 65536) / 65537 ≤ prescale_k
 *
 * As prescale_k is integer, this becomes
 *
 * ((val >> div_m) - 65536) // 65537 ≤ prescale_k
 *
 * And is minimized at
 *
 * ((val >> div_m) - 65536) // 65537
 *
 * Now we pick the smallest div_m that satifies prescale_k ≤ 255
 * (i.e SUN20I_PWM_CTL_PRESCAL_K_MAX),
 *
 * ((val >> div_m) - 65536) // 65537 ≤ 255
 * ⟺ (val >> div_m) - 65536 ≤ 255 * 65537 + 65536
 * ⟺ val >> div_m ≤ 255 * 65537 + 2 * 65536
 * ⟺ val >> div_m < (255 * 65537 + 2 * 65536 + 1)
 * ⟺ div_m = fls((val) / (255 * 65537 + 2 * 65536 + 1))
 *
 * Suggested by Uwe Kleine-König
 */
#define SUN20I_PWM_MAGIC			(255 * 65537 + 2 * 65536 + 1)

struct sun20i_pwm_chip {
	struct clk *clk_bus, *clk_hosc, *clk_apb;
	struct reset_control *rst;
	void __iomem *base;
	struct mutex mutex; /* Protect PWM apply state */
};

static inline struct sun20i_pwm_chip *to_sun20i_pwm_chip(struct pwm_chip *chip)
{
	return pwmchip_get_drvdata(chip);
}

static inline u32 sun20i_pwm_readl(struct sun20i_pwm_chip *chip,
				   unsigned long offset)
{
	return readl(chip->base + offset);
}

static inline void sun20i_pwm_writel(struct sun20i_pwm_chip *chip,
				     u32 val, unsigned long offset)
{
	writel(val, chip->base + offset);
}

static int sun20i_pwm_get_state(struct pwm_chip *chip,
				struct pwm_device *pwm,
				struct pwm_state *state)
{
	struct sun20i_pwm_chip *sun20i_chip = to_sun20i_pwm_chip(chip);
	u16 ent_cycle, act_cycle, prescale_k;
	u64 clk_rate, tmp;
	u8 div_m;
	u32 val;

	mutex_lock(&sun20i_chip->mutex);

	val = sun20i_pwm_readl(sun20i_chip, SUN20I_PWM_CLK_CFG(pwm->hwpwm / 2));
	div_m = FIELD_GET(SUN20I_PWM_CLK_CFG_DIV_M, val);
	if (div_m > SUN20I_PWM_CLK_DIV_M_MAX)
		div_m = SUN20I_PWM_CLK_DIV_M_MAX;

	if (FIELD_GET(SUN20I_PWM_CLK_CFG_SRC, val) == 0)
		clk_rate = clk_get_rate(sun20i_chip->clk_hosc);
	else
		clk_rate = clk_get_rate(sun20i_chip->clk_apb);

	val = sun20i_pwm_readl(sun20i_chip, SUN20I_PWM_CTL(pwm->hwpwm));
	state->polarity = (SUN20I_PWM_CTL_ACT_STA & val) ?
			   PWM_POLARITY_NORMAL : PWM_POLARITY_INVERSED;

	prescale_k = FIELD_GET(SUN20I_PWM_CTL_PRESCAL_K, val) + 1;

	val = sun20i_pwm_readl(sun20i_chip, SUN20I_PWM_ENABLE);
	state->enabled = (SUN20I_PWM_ENABLE_EN(pwm->hwpwm) & val) ? true : false;

	val = sun20i_pwm_readl(sun20i_chip, SUN20I_PWM_PERIOD(pwm->hwpwm));

	mutex_unlock(&sun20i_chip->mutex);

	act_cycle = FIELD_GET(SUN20I_PWM_PERIOD_ACT_CYCLE, val);
	ent_cycle = FIELD_GET(SUN20I_PWM_PERIOD_ENTIRE_CYCLE, val);

	/*
	 * The duration of the active phase should not be longer
	 * than the duration of the period
	 */
	if (act_cycle > ent_cycle)
		act_cycle = ent_cycle;

	/*
	 * We have act_cycle <= ent_cycle <= 0xffff, prescale_k <= 0x100,
	 * div_m <= 8. So the multiplication fits into an u64 without
	 * overflow.
	 */
	tmp = ((u64)(act_cycle) * prescale_k << div_m) * NSEC_PER_SEC;
	state->duty_cycle = DIV_ROUND_UP_ULL(tmp, clk_rate);
	tmp = ((u64)(ent_cycle) * prescale_k << div_m) * NSEC_PER_SEC;
	state->period = DIV_ROUND_UP_ULL(tmp, clk_rate);

	return 0;
}

static int sun20i_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			    const struct pwm_state *state)
{
	struct sun20i_pwm_chip *sun20i_chip = to_sun20i_pwm_chip(chip);
	u64 bus_rate, hosc_rate, val, ent_cycle, act_cycle;
	u32 clk_gate, clk_cfg, pwm_en, ctl, reg_period;
	u32 prescale_k, div_m;
	bool use_bus_clk;

	guard(mutex)(&sun20i_chip->mutex);

	pwm_en = sun20i_pwm_readl(sun20i_chip, SUN20I_PWM_ENABLE);

	if (state->enabled != pwm->state.enabled) {
		clk_gate = sun20i_pwm_readl(sun20i_chip, SUN20I_PWM_CLK_GATE);

		if (!state->enabled) {
			clk_gate &= ~SUN20I_PWM_CLK_GATE_GATING(pwm->hwpwm);
			pwm_en &= ~SUN20I_PWM_ENABLE_EN(pwm->hwpwm);
			sun20i_pwm_writel(sun20i_chip, pwm_en, SUN20I_PWM_ENABLE);
			sun20i_pwm_writel(sun20i_chip, clk_gate, SUN20I_PWM_CLK_GATE);

			return 0;
		}
	}

	ctl = sun20i_pwm_readl(sun20i_chip, SUN20I_PWM_CTL(pwm->hwpwm));
	clk_cfg = sun20i_pwm_readl(sun20i_chip, SUN20I_PWM_CLK_CFG(pwm->hwpwm / 2));
	hosc_rate = clk_get_rate(sun20i_chip->clk_hosc);
	bus_rate = clk_get_rate(sun20i_chip->clk_apb);
	if (pwm_en & SUN20I_PWM_ENABLE_EN(pwm->hwpwm ^ 1)) {
		/* if the neighbor channel is enabled, check period only */
		use_bus_clk = FIELD_GET(SUN20I_PWM_CLK_CFG_SRC, clk_cfg) != 0;
		val = mul_u64_u64_div_u64(state->period,
					  (use_bus_clk ? bus_rate : hosc_rate),
					  NSEC_PER_SEC);

		div_m = FIELD_GET(SUN20I_PWM_CLK_CFG_DIV_M, clk_cfg);
	} else {
		/*
		 * Select the clock source based on the period,
		 * since bus_rate > hosc_rate, which means bus_rate
		 * can provide a higher frequency than hosc_rate.
		 */
		use_bus_clk = false;
		val = mul_u64_u64_div_u64(state->period, hosc_rate, NSEC_PER_SEC);
		if (val <= 1) {
			use_bus_clk = true;
			val = mul_u64_u64_div_u64(state->period, bus_rate, NSEC_PER_SEC);
			if (val <= 1)
				return -EINVAL;
		}
		div_m = fls(DIV_ROUND_DOWN_ULL(val, SUN20I_PWM_MAGIC));
		if (div_m > SUN20I_PWM_CLK_DIV_M_MAX)
			return -EINVAL;

		/* set up the CLK_DIV_M and clock CLK_SRC */
		clk_cfg = FIELD_PREP(SUN20I_PWM_CLK_CFG_DIV_M, div_m);
		clk_cfg |= FIELD_PREP(SUN20I_PWM_CLK_CFG_SRC, use_bus_clk);

		sun20i_pwm_writel(sun20i_chip, clk_cfg, SUN20I_PWM_CLK_CFG(pwm->hwpwm / 2));
	}

	/* calculate prescale_k, PWM entire cycle */
	ent_cycle = val >> div_m;
	prescale_k = DIV_ROUND_DOWN_ULL(ent_cycle, 65537);
	if (prescale_k > SUN20I_PWM_CTL_PRESCAL_K_MAX)
		prescale_k = SUN20I_PWM_CTL_PRESCAL_K_MAX;

	do_div(ent_cycle, prescale_k + 1);

	/* for N cycles, PPRx.PWM_ENTIRE_CYCLE = (N-1) */
	reg_period = FIELD_PREP(SUN20I_PWM_PERIOD_ENTIRE_CYCLE, ent_cycle - 1);

	/* set duty cycle */
	val = mul_u64_u64_div_u64(state->duty_cycle,
				  (use_bus_clk ? bus_rate : hosc_rate),
				  NSEC_PER_SEC);
	act_cycle = val >> div_m;
	do_div(act_cycle, prescale_k + 1);

	/*
	 * The formula of the output period and the duty-cycle for PWM are as follows.
	 * T period = PWM0_PRESCALE_K / PWM01_CLK * (PPR0.PWM_ENTIRE_CYCLE + 1)
	 * T high-level = PWM0_PRESCALE_K / PWM01_CLK * PPR0.PWM_ACT_CYCLE
	 * Duty-cycle = T high-level / T period
	 */
	reg_period |= FIELD_PREP(SUN20I_PWM_PERIOD_ACT_CYCLE, act_cycle);
	sun20i_pwm_writel(sun20i_chip, reg_period, SUN20I_PWM_PERIOD(pwm->hwpwm));

	ctl = FIELD_PREP(SUN20I_PWM_CTL_PRESCAL_K, prescale_k);
	if (state->polarity == PWM_POLARITY_NORMAL)
		ctl |= SUN20I_PWM_CTL_ACT_STA;

	sun20i_pwm_writel(sun20i_chip, ctl, SUN20I_PWM_CTL(pwm->hwpwm));

	if (state->enabled != pwm->state.enabled && state->enabled) {
		clk_gate &= ~SUN20I_PWM_CLK_GATE_BYPASS(pwm->hwpwm);
		clk_gate |= SUN20I_PWM_CLK_GATE_GATING(pwm->hwpwm);
		pwm_en |= SUN20I_PWM_ENABLE_EN(pwm->hwpwm);
		sun20i_pwm_writel(sun20i_chip, pwm_en, SUN20I_PWM_ENABLE);
		sun20i_pwm_writel(sun20i_chip, clk_gate, SUN20I_PWM_CLK_GATE);
	}

	return 0;
}

static const struct pwm_ops sun20i_pwm_ops = {
	.apply = sun20i_pwm_apply,
	.get_state = sun20i_pwm_get_state,
};

static const struct of_device_id sun20i_pwm_dt_ids[] = {
	{ .compatible = "allwinner,sun20i-d1-pwm" },
	{ },
};
MODULE_DEVICE_TABLE(of, sun20i_pwm_dt_ids);

static void sun20i_pwm_reset_ctrl_release(void *data)
{
	struct reset_control *rst = data;

	reset_control_assert(rst);
}

static int sun20i_pwm_probe(struct platform_device *pdev)
{
	struct pwm_chip *chip;
	struct sun20i_pwm_chip *sun20i_chip;
	int ret;

	chip = devm_pwmchip_alloc(&pdev->dev, 8, sizeof(*sun20i_chip));
	if (IS_ERR(chip))
		return PTR_ERR(chip);
	sun20i_chip = to_sun20i_pwm_chip(chip);

	sun20i_chip->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(sun20i_chip->base))
		return PTR_ERR(sun20i_chip->base);

	sun20i_chip->clk_bus = devm_clk_get_enabled(&pdev->dev, "bus");
	if (IS_ERR(sun20i_chip->clk_bus))
		return dev_err_probe(&pdev->dev, PTR_ERR(sun20i_chip->clk_bus),
				     "failed to get bus clock\n");

	sun20i_chip->clk_hosc = devm_clk_get_enabled(&pdev->dev, "hosc");
	if (IS_ERR(sun20i_chip->clk_hosc))
		return dev_err_probe(&pdev->dev, PTR_ERR(sun20i_chip->clk_hosc),
				     "failed to get hosc clock\n");

	sun20i_chip->clk_apb = devm_clk_get_enabled(&pdev->dev, "apb");
	if (IS_ERR(sun20i_chip->clk_apb))
		return dev_err_probe(&pdev->dev, PTR_ERR(sun20i_chip->clk_apb),
				     "failed to get apb clock\n");

	if (clk_get_rate(sun20i_chip->clk_apb) > clk_get_rate(sun20i_chip->clk_hosc))
		dev_info(&pdev->dev, "apb clock must be greater than hosc clock");

	sun20i_chip->rst = devm_reset_control_get_exclusive(&pdev->dev, NULL);
	if (IS_ERR(sun20i_chip->rst))
		return dev_err_probe(&pdev->dev, PTR_ERR(sun20i_chip->rst),
				     "failed to get bus reset\n");

	ret = of_property_read_u32(pdev->dev.of_node, "allwinner,pwm-channels",
				   &chip->npwm);

	if (chip->npwm > 16) {
		dev_info(&pdev->dev, "limiting number of PWM lines from %u to 16",
			 chip->npwm);
		chip->npwm = 16;
	}

	/* Deassert reset */
	ret = reset_control_deassert(sun20i_chip->rst);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "failed to deassert reset\n");

	ret = devm_add_action_or_reset(&pdev->dev, sun20i_pwm_reset_ctrl_release, sun20i_chip->rst);
	if (ret)
		return ret;

	chip->ops = &sun20i_pwm_ops;

	mutex_init(&sun20i_chip->mutex);

	ret = devm_pwmchip_add(&pdev->dev, chip);
	if (ret < 0)
		return dev_err_probe(&pdev->dev, ret, "failed to add PWM chip\n");

	return 0;
}

static struct platform_driver sun20i_pwm_driver = {
	.driver = {
		.name = "sun20i-pwm",
		.of_match_table = sun20i_pwm_dt_ids,
	},
	.probe = sun20i_pwm_probe,
};
module_platform_driver(sun20i_pwm_driver);

MODULE_AUTHOR("Aleksandr Shubin <privatesub2@gmail.com>");
MODULE_DESCRIPTION("Allwinner sun20i PWM driver");
MODULE_LICENSE("GPL");
