// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * MCHP Fireant SoC Clock driver.
 *
 * Copyright (c) 2019 Microchip Inc.
 *
 * Author: Lars Povlsen <lars.povlsen@microchip.com>
 */

#include <linux/io.h>
#include <linux/clk-provider.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <dt-bindings/clock/mchp,fireant.h>

#define PLL_DIV_MASK		GENMASK(7, 0)
#define PLL_PRE_DIV_MASK	GENMASK(10, 8)
#define PLL_PRE_DIV_SHIFT	8
#define PLL_ROT_DIR		BIT(11)
#define PLL_ROT_SEL_MASK	GENMASK(13, 12)
#define PLL_ROT_SEL_SHIFT	12
#define PLL_ROT_ENA		BIT(14)
#define PLL_CLK_ENA		BIT(15)

#define MAX_SEL 4
#define MAX_PRE BIT(3)

#define KHZ 1000
#define MHZ (KHZ*KHZ)

#define BASE_CLOCK (2500UL*MHZ)

static u8 sel_rates[MAX_SEL] = { 0, 2*8, 2*4, 2*2 };

static const char *clk_names[N_CLOCKS] = {
	"core", "ddr", "cpu2", "arm2",
	"aux1", "aux2", "aux3", "aux4",
	"synce",
};

struct fa_hw_clk {
	struct clk_hw hw;
	void __iomem *reg;
	int index;
};

struct fa_clk_data {
	void __iomem *base;
	struct fa_hw_clk fa_hw[N_CLOCKS];
};

struct pll_conf {
	int freq;
	u8 div;
	bool rot_ena;
	u8 rot_sel;
	u8 rot_dir;
	u8 pre_div;
};

#define to_clk_pll(hw) container_of(hw, struct fa_hw_clk, hw)

unsigned long calc_freq(const struct pll_conf *pdata)
{
	unsigned long rate = BASE_CLOCK / pdata->div;

	if (pdata->rot_ena) {
		unsigned long base = BASE_CLOCK / pdata->div;
		int sign = pdata->rot_dir ? -1 : 1;
		int divt = sel_rates[pdata->rot_sel] * (1 + pdata->pre_div);
		int divb = divt + sign;

		rate = mult_frac(base, divt, divb);
		rate = roundup(rate, 1000);
	}

	return rate;
}

static unsigned long clk_calc_params(unsigned long rate,
				     struct pll_conf *conf)
{
	memset(conf, 0, sizeof(*conf));

	conf->div = DIV_ROUND_CLOSEST_ULL(BASE_CLOCK, rate);

	if (BASE_CLOCK % rate) {
		struct pll_conf best;
		ulong cur_offset, best_offset = rate;
		int i, j;

		/* Enable fractional rotation */
		conf->rot_ena = true;

		if ((BASE_CLOCK / rate) != conf->div) {
			/* Overshoot, adjust other direction */
			conf->rot_dir = 1;
		}

		/* Brute force search over MAX_PRE * (MAX_SEL - 1) = 24 */
		for (i = 0; i < MAX_PRE; i++) {
			conf->pre_div = i;
			for (j = 1; j < MAX_SEL; j++) {
				conf->rot_sel = j;
				conf->freq = calc_freq(conf);
				cur_offset = abs(rate - conf->freq);
				if (cur_offset == 0)
					/* Perfect fit */
					goto done;
				if (cur_offset < best_offset) {
					/* Better fit found */
					best_offset = cur_offset;
					best = *conf;
				}
			}
		}
		/* Best match */
		*conf = best;
	}

done:
	return conf->freq;
}

static int clk_pll_enable(struct clk_hw *hw)
{
	struct fa_hw_clk *pll = to_clk_pll(hw);
	u32 val = readl(pll->reg);

	val |= PLL_CLK_ENA;
	writel(val, pll->reg);
	pr_debug("%s: Enable val %04x\n", clk_names[pll->index], val);
	return 0;
}

static void clk_pll_disable(struct clk_hw *hw)
{
	struct fa_hw_clk *pll = to_clk_pll(hw);
	u32 val = readl(pll->reg);

	val &= ~PLL_CLK_ENA;
	writel(val, pll->reg);
	pr_debug("%s: Disable val %04x\n", clk_names[pll->index], val);
}

static int clk_pll_set_rate(struct clk_hw *hw,
			    unsigned long rate,
			    unsigned long parent_rate)
{
	struct fa_hw_clk *pll = to_clk_pll(hw);
	struct pll_conf conf;
	unsigned long eff_rate;
	int ret = 0;

	eff_rate = clk_calc_params(rate, &conf);
	if (eff_rate == rate) {
		u32 val;

		val = readl(pll->reg) & PLL_CLK_ENA;
		val |= PLL_DIV_MASK & conf.div;
		if (conf.rot_ena) {
			val |= (PLL_ROT_ENA |
				(PLL_ROT_SEL_MASK &
				 (conf.rot_sel << PLL_ROT_SEL_SHIFT)) |
				(PLL_PRE_DIV_MASK &
				 (conf.pre_div << PLL_PRE_DIV_SHIFT)));
			if (conf.rot_dir)
				val |= PLL_ROT_DIR;
		}
		pr_debug("%s: Rate %ld >= 0x%04x\n",
			 clk_names[pll->index], rate, val);
		writel(val, pll->reg);
	} else {
		pr_err("%s: freq unsupported: %ld paren %ld\n",
		       clk_names[pll->index], rate, parent_rate);
		ret = -ENOTSUPP;
	}

	return ret;
}

static unsigned long clk_pll_recalc_rate(struct clk_hw *hw,
					 unsigned long parent_rate)
{
	/* Don't care */
	return 0;
}

static long clk_pll_round_rate(struct clk_hw *hw, unsigned long rate,
			       unsigned long *parent_rate)
{
	struct pll_conf conf;
	unsigned long eff_rate;

	eff_rate = clk_calc_params(rate, &conf);
	pr_debug("%s: Rate %ld rounded to %ld\n", __func__, rate, eff_rate);

	return eff_rate;
}

static const struct clk_ops fa_pll_ops = {
	.enable		= clk_pll_enable,
	.disable	= clk_pll_disable,
	.set_rate	= clk_pll_set_rate,
	.round_rate	= clk_pll_round_rate,
	.recalc_rate	= clk_pll_recalc_rate,
};

static struct fa_clk_data *fa_clk_alloc(struct device_node *np)
{
	struct fa_clk_data *clk_data;

	clk_data = kzalloc(sizeof(*clk_data), GFP_KERNEL);
	if (WARN_ON(!clk_data))
		return NULL;

	clk_data->base = of_iomap(np, 0);
	if (WARN_ON(!clk_data->base))
		return NULL;

	return clk_data;
}

static struct clk_hw *fa_clk_hw_get(struct of_phandle_args *clkspec, void *data)
{
	struct fa_clk_data *pll_clk = data;
	unsigned int idx = clkspec->args[0];

	if (idx >= N_CLOCKS) {
		pr_err("%s: invalid index %u\n", __func__, idx);
		return ERR_PTR(-EINVAL);
	}

	return &pll_clk->fa_hw[idx].hw;
}

static void __init fa_pll_init(struct device_node *np)
{
	int i, ret;
	struct fa_clk_data *pll_clk;
	struct clk_init_data init = { 0 };

	pll_clk = fa_clk_alloc(np);
	if (!pll_clk)
		return;

	init.ops = &fa_pll_ops;
	init.parent_names = NULL;
	init.num_parents = 0;

	for (i = 0; i < N_CLOCKS; i++) {
		struct fa_hw_clk *fa_hw = &pll_clk->fa_hw[i];

		init.name = clk_names[i];
		fa_hw->index = i;
		fa_hw->reg = pll_clk->base + (i * sizeof(u32));
		fa_hw->hw.init = &init;
		ret = of_clk_hw_register(np, &fa_hw->hw);
		if (ret) {
			pr_err("failed to register %s clock\n", init.name);
			return;
		}
	}

	of_clk_add_hw_provider(np, fa_clk_hw_get, pll_clk);
}
CLK_OF_DECLARE_DRIVER(mchp_fa, "mchp,fireant-clock", fa_pll_init);
