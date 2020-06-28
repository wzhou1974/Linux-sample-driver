#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/of_address.h>
#include <linux/of.h>
#include <linux/io.h>

#define to_pegmatite_oc_factor(_hw) container_of(_hw, struct pegmatite_oc_factor, hw)
struct pegmatite_oc_factor {
	struct clk_hw		hw;
	unsigned int		mult;
};

static unsigned long pegmatite_oc_factor_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct pegmatite_oc_factor *oc_factor = to_pegmatite_oc_factor(hw);
	unsigned long calc_rate;

	calc_rate = parent_rate * oc_factor->mult;
	return calc_rate;
}

static int pegmatite_oc_factor_set_rate(struct clk_hw *hw, unsigned long rate, unsigned long parent_rate)
{
	struct pegmatite_oc_factor *oc_factor = to_pegmatite_oc_factor(hw);
	unsigned int mult = 1;

	if (rate < parent_rate) {
		rate = parent_rate;
	}

	mult = rate / parent_rate;
	if (rate % parent_rate)
	{
		if (abs(rate - parent_rate * mult) >
			abs(rate - parent_rate * (mult + 1))) {
			mult++;
		}
	}

	oc_factor->mult = mult;

	return 0;
}

static long pegmatite_oc_factor_round_rate(struct clk_hw *hw, unsigned long rate, unsigned long *parent_rate)
{
	unsigned int mult = 1;

	if (rate < *parent_rate) {
		return *parent_rate;
	}

	mult = rate / *parent_rate;
	if (rate % *parent_rate)
	{
		if (abs(rate - *parent_rate * mult) >
			abs(rate - *parent_rate * (mult + 1))) {
			mult++;
		}
	}

	return *parent_rate * mult;
}

const struct clk_ops pegmatite_oc_factor_ops = {
	.recalc_rate = pegmatite_oc_factor_recalc_rate,
	.set_rate = pegmatite_oc_factor_set_rate,
	.round_rate = pegmatite_oc_factor_round_rate,
};

/**
 * of_oc_factor_clk_setup - Setup function for off chip factor clock
 * @node: device node for this clock
 *
 * Sets up a simple factor clock based on device tree info. It can be set
 * at runtime.
 */
static void __init of_oc_factor_clk_setup(struct device_node *node)
{
	struct pegmatite_oc_factor *oc_factor;
	struct clk *clk;
	struct clk *parent_clk;
	struct clk_init_data *init;
	const char *parent_name;
	unsigned int default_rate;

	oc_factor = kzalloc(sizeof(*oc_factor), GFP_KERNEL);
	if (!oc_factor) {
		pr_err("%s: could not allocate oc_factor clk\n", __func__);
		return;
	}

	init = kzalloc(sizeof(*init), GFP_KERNEL);
	if (!init) {
		pr_err("%s: could not allocate oc_factor init\n", __func__);
		goto free_out;
	}

	 /* get a default clock rate for the oc_factor_clk if provided */
	if (of_property_read_u32(node, "clock-frequency", &default_rate)) {
		default_rate = 0;
	}

	init->name = kasprintf(GFP_KERNEL, "%s", node->name);
	init->ops = &pegmatite_oc_factor_ops;
	init->flags = 0;
	parent_clk = of_clk_get(node, 0);
	parent_name = __clk_get_name(parent_clk);
	init->parent_names = &parent_name;
	init->num_parents = 1;

	oc_factor->hw.init = init;

	/* init mult to 1 */
	oc_factor->mult = 1;

	clk = clk_register(NULL, &oc_factor->hw);
	if(WARN_ON(IS_ERR(clk)))
		goto free_out2;

	of_clk_add_provider(node, of_clk_src_simple_get, clk);

	/* If a default rate was specified in the device tree, set it here */
	if(default_rate > 0)
		clk_set_rate(clk, default_rate);

	return;
free_out2:
	kfree(init);
free_out:
	kfree(oc_factor);
}

CLK_OF_DECLARE(pegmatite_oc_factor, "marvell,pegmatite-oc-factor-clk", of_oc_factor_clk_setup);
