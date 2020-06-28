/*
 * Marvell Pegmatite SoC clock handling.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/of_address.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/module.h>

#define CLKOUT_MASK 0x1
#define CLKOUT_SHIFT 31
#define HIDIV_MASK 0xff
#define HIDIV_SHIFT 8
#define LODIV_MASK 0xff
#define LODIV_SHIFT 0


#define to_pegmatite_clklvdsafe(_hw) container_of(_hw, struct pegmatite_clklvdsafe, hw)
struct pegmatite_clklvdsafe {
	struct clk_hw       hw;
	void __iomem        *base;
	struct clk          *parent_clk;
};

static int pegmatite_clklvdsafe_is_enabled(struct clk_hw *hw)
{
	struct pegmatite_clklvdsafe *lvdsafe = to_pegmatite_clklvdsafe(hw);
	unsigned int val = 0;
	if (__clk_is_enabled(lvdsafe->parent_clk))
	{
		val = readl(lvdsafe->base);
		val = (val >> CLKOUT_SHIFT) & CLKOUT_MASK;
	}
	return (val);
}

static int pegmatite_clklvdsafe_enable(struct clk_hw *hw)
{
	struct pegmatite_clklvdsafe *lvdsafe = to_pegmatite_clklvdsafe(hw);
	unsigned int val = readl(lvdsafe->base);
	val |= (CLKOUT_MASK << CLKOUT_SHIFT);
	writel(val, lvdsafe->base);
	return 0;
}

static unsigned long pegmatite_clklvdsafe_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct pegmatite_clklvdsafe *lvdsafe = to_pegmatite_clklvdsafe(hw);
	unsigned long rate = parent_rate;
	unsigned int hidiv, lodiv;
	u32 val = 0;
	if (parent_rate == 0 || !__clk_is_enabled(lvdsafe->parent_clk)) {
	return 0;
	}

	val = readl(lvdsafe->base);
	hidiv = (val >> HIDIV_SHIFT) & HIDIV_MASK;
	lodiv = (val >> LODIV_SHIFT) & LODIV_MASK;
	rate /= (hidiv + 1 + lodiv + 1);
	return rate;
}

static int pegmatite_clklvdsafe_set_rate(struct clk_hw *hw, unsigned long rate, unsigned long parent_rate)
{
	struct pegmatite_clklvdsafe *lvdsafe = to_pegmatite_clklvdsafe(hw);
	unsigned int totaldiv, hidiv, lodiv;
	u32 val;
	if (parent_rate == 0 || rate == 0 || !__clk_is_enabled(lvdsafe->parent_clk)) {
	return 0;
	}
	totaldiv = parent_rate / rate;
	if (totaldiv < 1 + 1)
		totaldiv = 1 + 1;
	if (totaldiv > HIDIV_MASK + 1 + LODIV_MASK + 1)
		totaldiv = HIDIV_MASK + 1 + LODIV_MASK + 1;
	/* if we aren't going over, check if the next divider is closer */
	if ((totaldiv < HIDIV_MASK + 1 + LODIV_MASK + 1) &&
		(abs(rate - parent_rate / totaldiv) > abs(rate - parent_rate / (totaldiv + 1))))
	{
		totaldiv++;
	}
	/* make sure we are in a valid range */
	if (totaldiv < 1 + 1)
		totaldiv = 1 + 1;
	if (totaldiv > HIDIV_MASK + 1 + LODIV_MASK + 1)
		totaldiv = HIDIV_MASK + 1 + LODIV_MASK + 1;
	/* split total divider into low/high */
	lodiv = totaldiv / 2;
	hidiv = totaldiv - lodiv;

	/* Values saved are - 1 what they are representing*/
	hidiv--;
	lodiv--;

	val = readl(lvdsafe->base);
	val &= ~(HIDIV_MASK << HIDIV_SHIFT);
	val &= ~(LODIV_MASK << LODIV_SHIFT);
	val |= (hidiv & HIDIV_MASK) << HIDIV_SHIFT;
	val |= (lodiv & LODIV_MASK) << LODIV_SHIFT;
	writel(val, lvdsafe->base);

	return 0;
}

static long pegmatite_clklvdsafe_round_rate(struct clk_hw *hw, unsigned long rate, unsigned long *parent_rate)
{
	struct pegmatite_clklvdsafe *lvdsafe = to_pegmatite_clklvdsafe(hw);
	unsigned int totaldiv;

	if (rate == 0 || *parent_rate == 0 || !__clk_is_enabled(lvdsafe->parent_clk)) {
		return 0;
	}
	totaldiv = *parent_rate / rate;
	/* if we aren't going over, check if the next divider is closer */
	if ((totaldiv < HIDIV_MASK + 1 + LODIV_MASK + 1) &&
		(abs(rate - *parent_rate / totaldiv) > abs(rate - *parent_rate / (totaldiv + 1))))
	{
		totaldiv++;
	}
	/* make sure we are in a valid range */
	if (totaldiv < 1 + 1)
		totaldiv = 1 + 1;
	if (totaldiv > HIDIV_MASK + 1 + LODIV_MASK + 1)
		totaldiv = HIDIV_MASK + 1 + LODIV_MASK + 1;

	return *parent_rate / totaldiv;
}

static void pegmatite_clkgate_disable(struct clk_hw *hw)
{
	struct pegmatite_clklvdsafe *lvdsafe = to_pegmatite_clklvdsafe(hw);
	unsigned int val = 0;
	if (!__clk_is_enabled(lvdsafe->parent_clk)) {
		return;
	}
	val = readl(lvdsafe->base);
	val &= ~(CLKOUT_MASK << CLKOUT_SHIFT);
	writel(val, lvdsafe->base);
}

const struct clk_ops pegmatite_clklvdsafe_ops = {
	.enable = pegmatite_clklvdsafe_enable,
	.disable = pegmatite_clkgate_disable,
	.is_enabled = pegmatite_clklvdsafe_is_enabled,
	.recalc_rate = pegmatite_clklvdsafe_recalc_rate,
	.round_rate = pegmatite_clklvdsafe_round_rate,
	.set_rate = pegmatite_clklvdsafe_set_rate,
};

static void __init of_pegmatite_clklvdsafe_setup(struct device_node *node)
{
	struct pegmatite_clklvdsafe *lvdsafe;
	struct clk *clk;
	struct clk_init_data *init;
	const char *parent_name;

	lvdsafe = kzalloc(sizeof(*lvdsafe), GFP_KERNEL);
	if (!lvdsafe) {
		pr_err("%s: could not allocate clklvdsafe clk\n", __func__);
		return;
	}

	init = kzalloc(sizeof(*init), GFP_KERNEL);
	if (!init) {
		pr_err("%s: could not allocate clklvdsafe init\n", __func__);
		goto free_out;
	}

	lvdsafe->base = of_iomap(node, 0);
	if(WARN_ON(!lvdsafe->base))
		goto free_out2;

	init->name = kasprintf(GFP_KERNEL, "%s", node->name);
	init->ops = &pegmatite_clklvdsafe_ops;

	/* we want this to always check if the parent is gated. */
	init->flags = CLK_GET_RATE_NOCACHE;
	lvdsafe->parent_clk = of_clk_get(node, 0);
	parent_name = __clk_get_name(lvdsafe->parent_clk);
	init->parent_names = &parent_name;
	init->num_parents = 1;

	lvdsafe->hw.init = init;

	clk = clk_register(NULL, &lvdsafe->hw);
	if(WARN_ON(IS_ERR(clk)))
		goto map_out;

	of_clk_add_provider(node, of_clk_src_simple_get, clk);

	return;
map_out:
	iounmap(lvdsafe->base);
free_out2:
	kfree(init);
free_out:
	kfree(lvdsafe);
}

CLK_OF_DECLARE(pegmatite_clklvdsafe, "marvell,pegmatite-clklvdsafe", of_pegmatite_clklvdsafe_setup);
