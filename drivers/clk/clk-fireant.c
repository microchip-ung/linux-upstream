// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * MCHP Fireant SoC Clock driver.
 *
 * Copyright (c) 2019 Microchip Inc.
 *
 * Author: Lars Povlsen <lars.povlsen@microchip.com>
 */

#define DEBUG
#include <linux/io.h>
#include <linux/clk-provider.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <dt-bindings/clock/mchp,fireant.h>

#define PLL_DIV_MASK		GENMASK(7,0)
#define PLL_PRE_DIV_MASK	GENMASK(10,8)
#define PLL_PRE_DIV_SHIFT	8
#define PLL_ROT_DIR		BIT(11)
#define PLL_ROT_SEL_MASK	GENMASK(13,12)
#define PLL_ROT_SEL_SHIFT	12
#define PLL_ROT_ENA		BIT(14)
#define PLL_CLK_ENA		BIT(15)

#define KHZ 1000
#define MHZ (KHZ*KHZ)

#define BASE_CLOCK (2500*MHZ)

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

#define to_clk_pll(hw) container_of(hw, struct fa_hw_clk, hw)

static int clk_calc_params(unsigned long rate,
			   unsigned long parent_rate,
			   int *div,
			   int *pre_div,
			   int *rot_sel)
{
	/* Cheat */
	if (rate == 100*MHZ) {
		*div = 3;
		*pre_div = 2;
		*rot_sel = 2;
		return 0;
	}
	return -ENOTSUPP;
}

static int clk_pll_set_rate(struct clk_hw *hw,
			    unsigned long rate,
			    unsigned long parent_rate)
{
	struct fa_hw_clk *pll = to_clk_pll(hw);
	int ret;
	int div, pre_div, rot_sel;

	ret = clk_calc_params(rate, parent_rate,
			      &div, &pre_div, &rot_sel);
	if (!ret) {
		u32 val = PLL_CLK_ENA | PLL_ROT_ENA |
			(PLL_ROT_SEL_MASK & (rot_sel << PLL_ROT_SEL_SHIFT)) |
			(PLL_PRE_DIV_MASK & (pre_div << PLL_PRE_DIV_SHIFT)) |
			(PLL_DIV_MASK & div);
		writel(val, pll->reg);
	} else {
		pr_err("%s: clock %d unsupported: %ld paren %ld\n",
		       clk_names[pll->index], pll->index, rate, parent_rate);
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
	/* 1:1 */
	return rate;
}

static const struct clk_ops fa_pll_ops = {
	.set_rate    = clk_pll_set_rate,
	.round_rate  = clk_pll_round_rate,
	.recalc_rate = clk_pll_recalc_rate,
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
