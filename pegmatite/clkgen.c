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

#define SRCSEL_MASK 0x3
#define SRCSEL_SHIFT 24
#define HIDIV_MASK 0xff
#define HIDIV_SHIFT 16
#define LODIV_MASK 0xff
#define LODIV_SHIFT 8
#define DIV_SEL_SHIFT 20
#define PRE_DIV_ENB_SHIFT 2
#define PRE_DIV_VAL_SHIFT 27
#define PRE_DIV_VAL_MASK 0xff

#define to_pegmatite_clkgen(_hw) container_of(_hw, struct pegmatite_clkgen, hw)
struct pegmatite_clkgen {
	struct clk_hw		hw;
	void __iomem		*config;
	int			clock_source;
	int			max_divide;
	int			use_div_select;
	bool			use_prediv;
	int			prediv_shift;
};

static unsigned long pegmatite_clkgen_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct pegmatite_clkgen *gen = to_pegmatite_clkgen(hw);
	unsigned long rate = parent_rate;
	u32 val;

	if (gen->use_div_select) {
		val = readl(gen->config);

		if (val & (1 << DIV_SEL_SHIFT))
			rate = parent_rate / 4;
		else
			rate = parent_rate / 2;
	}
	else {
		/*
		 * If this clock has a predivider, apply that first
		 */
		unsigned int hidiv, lodiv;

		val = readl(gen->config + 4);

		if (gen->use_prediv) {
			if (val & (1 << PRE_DIV_ENB_SHIFT))
				rate /= ((val >> gen->prediv_shift) & PRE_DIV_VAL_MASK);
		}

		/*
		 * A value of zero for either div means no divider is applied,
		 * so only if there is a value of both to we apply lodiv + hidiv
		 */
		hidiv = (val >> HIDIV_SHIFT) & HIDIV_MASK;
		lodiv = (val >> LODIV_SHIFT) & LODIV_MASK;
		if (lodiv && hidiv) {
			rate /= (lodiv + hidiv);
		}
	}

	return rate;
}

static int pegmatite_clkgen_set_rate(struct clk_hw *hw, unsigned long rate, unsigned long parent_rate)
{
	struct pegmatite_clkgen *gen = to_pegmatite_clkgen(hw);
	u32 val = readl(gen->config);

	if (gen->use_div_select) {
		u32 div_sel = 0;

		if ((parent_rate / 2) > rate)
			div_sel = 1;

		val &= ~(1 << DIV_SEL_SHIFT);
		val |= (div_sel << DIV_SEL_SHIFT);
	}
	else {
		unsigned int hidiv = 0, lodiv = 0, prediv = 0;

		/*
		 * If the parent_rate (w/ or w/o predivider) matches the requested rate
		 * then no further dividers are needed
		 */
		if (parent_rate > rate) {
			unsigned int div;

			/*
			 * If the max hi-lo divide cannot get the clock rate slow enough
			 * use the predivider
			 */
			if (gen->use_prediv) {
				if ((parent_rate / gen->max_divide) > rate) {
					prediv = readl(gen->config + 4);
					prediv >>= gen->prediv_shift;
					prediv &= PRE_DIV_VAL_MASK;

					if (prediv)
						parent_rate /= prediv;
				}
			}

			/*
			 * Caclulate the divider required to divide the parent_rate (w/ or w/o predivider) down to the requested rate
			 */
			if (parent_rate > rate) {
				div = parent_rate / rate;
				if (abs(rate - parent_rate / div) > abs(rate - parent_rate / (div + 1)))
					++div;

				if (div > gen->max_divide) {
					div = gen->max_divide;
					pr_err("%s: %s divider %d greater than max %d!!\n", __func__, hw->init->name, div, gen->max_divide);
				}

				hidiv = div / 2;
				lodiv = div - hidiv;
			}
		}

		/*
		 * Clear any old lodiv and/or hidiv values, and set the calculated values
		 */
		if (prediv)
			val |= (1 << PRE_DIV_ENB_SHIFT);
		else
			val &= ~(1 << PRE_DIV_ENB_SHIFT);

		val &= ~(HIDIV_MASK << HIDIV_SHIFT);
		val &= ~(LODIV_MASK << LODIV_SHIFT);
		val |= (hidiv & HIDIV_MASK) << HIDIV_SHIFT;
		val |= (lodiv & LODIV_MASK) << LODIV_SHIFT;
	}

	writel(val, gen->config);

	return 0;
}

static long pegmatite_clkgen_round_rate(struct clk_hw *hw, unsigned long rate, unsigned long *prate)
{
	struct pegmatite_clkgen *gen = to_pegmatite_clkgen(hw);
	unsigned int calc_rate = *prate;

	if (calc_rate <= rate)
		return calc_rate;

	if (gen->use_div_select) {
		if ((calc_rate / 2) <= rate)
			calc_rate /= 2;
		else
			calc_rate /= 4;
	}
	else {
		unsigned int div;

		/*
		 * If the max hi-lo divide cannot get the clock rate slow enough
		 * use the predivider
		 */
		if (gen->use_prediv) {
			if ((calc_rate / gen->max_divide) > rate) {
				unsigned int prediv = readl(gen->config + 4);
				prediv >>= gen->prediv_shift;
				prediv &= PRE_DIV_VAL_MASK;

				if (prediv)
					calc_rate /= prediv;

				if (calc_rate <= rate)
					return calc_rate;
			}
		}

		div = calc_rate / rate;
		if (abs(rate - calc_rate / div) > abs(rate - calc_rate / (div + 1)))
			++div;

		if (div > gen->max_divide) {
			div = gen->max_divide;
			pr_err("%s: %s divider %d greater than max %d!!\n", __func__, hw->init->name, div, gen->max_divide);
		}

		calc_rate /= div;
	}

	return calc_rate;
}

const struct clk_ops pegmatite_clkgen_ops = {
	.recalc_rate = pegmatite_clkgen_recalc_rate,
	.set_rate = pegmatite_clkgen_set_rate,
	.round_rate = pegmatite_clkgen_round_rate,
};

static void __init of_pegmatite_clkgen_setup(struct device_node *node)
{
	struct pegmatite_clkgen *gen;
	struct clk *clk;
	void __iomem *clk_base;
	struct clk *parent_clk;
	struct clk_init_data *init;
	const char *parent_name;
	u32 default_rate;
	int val;

	gen = kzalloc(sizeof(*gen), GFP_KERNEL);
	if (!gen) {
		pr_err("%s: could not allocate clkgen clk\n", __func__);
		return;
	}

	init = kzalloc(sizeof(*init), GFP_KERNEL);
	if (!init) {
		pr_err("%s: could not allocate clkgen init\n", __func__);
		goto free_out;
	}

	/*
	 * Some clocks have multiple possible clock sources
	 */
	if (of_property_read_u32(node, "clock-source", &gen->clock_source)) {
		gen->clock_source = 0;
	}

	gen->use_prediv = !of_property_read_bool(node, "no-prediv");

	/*
	 * The DDR clock only has one bit to controll the divider
	 */
	gen->use_div_select = of_property_read_bool(node, "use-div-sel");

	/*
	 * Some clocks have different maximum divide values
	 */
	if (of_property_read_u32(node, "max-divide", &gen->max_divide)) {
		gen->max_divide = 16;
	}

	/*
	 * Rev B changed the predivider setting to a variable width field
	 */
	if (of_property_read_u32(node, "prediv-shift", &gen->prediv_shift)) {
		gen->prediv_shift = PRE_DIV_VAL_SHIFT;
	}

	/*
	 * clock-frequency can be set to enable a default clock rate for the clock
	 */
	if (of_property_read_u32(node, "clock-frequency", &default_rate)) {
		default_rate = 0;
	}

	clk_base = of_iomap(node, 0);
	if(WARN_ON(!clk_base))
		goto free_out2;

	init->name = kasprintf(GFP_KERNEL, "%s", node->name);
	init->ops = &pegmatite_clkgen_ops;
	init->flags = 0;
	parent_clk = of_clk_get(node, gen->clock_source);
	parent_name = __clk_get_name(parent_clk);
	init->parent_names = &parent_name;
	init->num_parents = 1;

	gen->hw.init = init;
	gen->config = clk_base;
	val = readl(gen->config);
	val &= ~(SRCSEL_MASK << SRCSEL_SHIFT);
	val |= (gen->clock_source & SRCSEL_MASK) << SRCSEL_SHIFT;
	writel(val, gen->config);

	clk = clk_register(NULL, &gen->hw);
	if(WARN_ON(IS_ERR(clk)))
		goto map_out;

	of_clk_add_provider(node, of_clk_src_simple_get, clk);

	/*
	 * If a default rate was specified in the device tree, set it here
	 * If this clock can be gated, setting the default rate does not ungate it
	 */
	if(default_rate > 0)
		clk_set_rate(clk, default_rate);

	return;
map_out:
	iounmap(clk_base);
free_out2:
	kfree(init);
free_out:
	kfree(gen);
}

CLK_OF_DECLARE(pegmatite_clkgen, "marvell,pegmatite-clkgen", of_pegmatite_clkgen_setup);
