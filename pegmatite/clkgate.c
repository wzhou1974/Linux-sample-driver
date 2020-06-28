/*
 * Marvell Pegmatite SoC clock handling.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/of_address.h>
#include <linux/of.h>
#include <linux/io.h>

#define CLK_EN_MASK 2
#define CLK_RESET_MASK 1

#define CLK_STATUS_OFFSET		0x4
#define CLK_STATUS_SUCCESS_MASK		0x8
#define CLK_STATUS_FAIL_MASK		0x10

#define to_pegmatite_clkgate(_hw) container_of(_hw, struct pegmatite_clkgate, hw)
struct pegmatite_clkgate {
	struct clk_hw	hw;
	void __iomem	*config;
	int		reset;
};

static bool debug_clkdisable;
module_param(debug_clkdisable, bool, 0644);

static int pegmatite_clkgate_is_enabled(struct clk_hw *hw)
{
	struct pegmatite_clkgate *gate = to_pegmatite_clkgate(hw);
	int reset;
	int val = readl(gate->config);

	/*
	 * If this is a clock with a reset, read the reset value
	 */
	if(gate->reset) {
		reset = !!(val & CLK_RESET_MASK);
	} else {
		reset = 1;
	}

	/*
	 * Read the enable bit
	 */
	return (val & CLK_EN_MASK) ? reset : 0;
}

static int pegmatite_clkgate_enable(struct clk_hw *hw)
{
	struct pegmatite_clkgate *gate = to_pegmatite_clkgate(hw);
	int val = readl(gate->config);

	/*
	 * Set the enable bit
	 */
	val |= CLK_EN_MASK;
	writel(val, gate->config);

	/*
	 * If this is a clock with a reset, set that too
	 */
	if(gate->reset) {
		val |= CLK_RESET_MASK;
		writel(val, gate->config);
	}

	return 0;
}

static void pegmatite_clkgate_disable(struct clk_hw *hw)
{
	struct pegmatite_clkgate *gate = to_pegmatite_clkgate(hw);
	int val = readl(gate->config);

	/*
	 * If this is a clock with a reset, clear it first
	 */
	if(gate->reset) {
		val &= ~CLK_RESET_MASK;
		writel(val, gate->config);
	}

	/*
	 * Clear the enable bit
	 */
	val &= ~CLK_EN_MASK;
	writel(val, gate->config);
	if (debug_clkdisable) {
		val = readl(gate->config + CLK_STATUS_OFFSET);
		WARN(!(val & CLK_STATUS_SUCCESS_MASK),
		     "pegmatite clock %s@%pK disable failed. Status = 0x%x",
		     __clk_get_name(hw->clk), gate->config, val);
	}
}

const struct clk_ops pegmatite_clkgate_ops = {
	.is_enabled = pegmatite_clkgate_is_enabled,
	.enable = pegmatite_clkgate_enable,
	.disable = pegmatite_clkgate_disable,
};

static void __init of_pegmatite_clkgate_setup(struct device_node *node)
{
	struct pegmatite_clkgate *gate;
	struct clk *clk;
	void __iomem *clk_base;
	struct clk *parent_clk;
	struct clk_init_data *init;
	const char *parent_name;
	int always_used;

	gate = kzalloc(sizeof(*gate), GFP_KERNEL);
	if (!gate) {
		pr_err("%s: could not allocate clkgate clk\n", __func__);
		return;
	}

	init = kzalloc(sizeof(*init), GFP_KERNEL);
	if (!init) {
		pr_err("%s: could not allocate clkgate init\n", __func__);
		goto free_out;
	}

	/*
	 * If the reset property is set, this is as clock with a sw reset bit
	 */
	if (of_property_read_u32(node, "reset", &gate->reset)) {
		gate->reset = 0;
	}

	always_used  = of_property_read_bool(node, "always-used");

	clk_base = of_iomap(node, 0);
	if(WARN_ON(!clk_base))
		goto free_out2;

	init->name = node->name;
	init->ops = &pegmatite_clkgate_ops;
	init->flags = CLK_SET_RATE_PARENT;
	parent_clk = of_clk_get(node, 0);
	parent_name = __clk_get_name(parent_clk);
	init->parent_names = &parent_name;
	init->num_parents = 1;

	gate->hw.init = init;
	gate->config = clk_base;

	clk = clk_register(NULL, &gate->hw);
	if(WARN_ON(IS_ERR(clk)))
		goto map_out;

	if (always_used) {
		clk_prepare_enable(clk);
	}

	of_clk_add_provider(node, of_clk_src_simple_get, clk);

	return;
map_out:
	iounmap(clk_base);
free_out2:
	kfree(init);
free_out:
	kfree(gate);
}

CLK_OF_DECLARE(pegmatite_clkgate, "marvell,pegmatite-clkgate", of_pegmatite_clkgate_setup);
