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
#include <linux/device.h>
#include <linux/of_address.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/delay.h>


#define FIXED_MODE_SSC_MODE_OFFSET 0x18
#define BYPASS_EN_MASK 0x1
#define BYPASS_EN_SHIFT 16

#define CSSCG_MODE_CONFIG_OFFSET 0x810
#define MAX_ADDR_MASK 0x3f
#define MAX_ADDR_SHIFT 24

#define INTERP_POINTS_MASK 0xff
#define INTERP_POINTS_SHIFT 12

#define APPLY_CORRECTION_MASK 0x1
#define APPLY_CORRECTION_SHIFT 0x1

#define DOWN_SPREAD_MASK 0x1
#define DOWN_SPREAD_SHIFT 0x0

#define CSSCG_CONTROL_OFFSET 0x81c
#define CSSCG_ENABLED 0x5 /* csscg_external_mux_sel and csscg_enabled */

#define CSSCG_RAM(x) (0xc00 + (x * 4))

#define to_pegmatite_sscg(_hw) container_of(_hw, struct pegmatite_sscg, hw)
struct pegmatite_sscg {
	struct clk_hw		hw;
	void __iomem		*base;
	int			sscg_disabled;
	unsigned int		down_spread_offset;
	unsigned int		interp_points;
};

static unsigned long pegmatite_sscg_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct pegmatite_sscg *sscg = to_pegmatite_sscg(hw);
	unsigned long calc_rate;
	int val;

	/*
	 * If sscg is disabled return the parent_rate
	 */
	if(sscg->sscg_disabled) {
		return parent_rate;
	}

	/*
	 * If the pll is in bypass, then return the parent_rate
	 */
	val = readl(sscg->base + FIXED_MODE_SSC_MODE_OFFSET);
	if(val & (BYPASS_EN_MASK << BYPASS_EN_SHIFT)) {
		return parent_rate;
	}

	/*
	 * If down-spread is enabled, we need to apply an offset to our reported frequency
	 */
	val = readl(sscg->base + CSSCG_CONTROL_OFFSET);
	if(val == CSSCG_ENABLED && sscg->down_spread_offset != 0) {
		unsigned long offset;

		/*
		 * The down-spread-offset value read from the device tree is the offset percentage (to
		 * three decimal places) multipled by 1000.  So we need to start by dividing the parent rate
		 * by 100,000
		 */
		offset = parent_rate / (100000);

		/*
		 * Since we are finding the new center frequency we need to divide the offset in half, and
		 * and then multiply it by our divided parent_rate
		 */
		offset = (offset * (sscg->down_spread_offset / 2));

		/*
		 * Now that we have the offset, we can subtract it from the parent_rate
		 */
		calc_rate = parent_rate - offset;
	} else {
		calc_rate = parent_rate;
	}

	return calc_rate;
}

const struct clk_ops pegmatite_sscg_ops = {
	.recalc_rate = pegmatite_sscg_recalc_rate,
};

static void __init of_pegmatite_sscg_setup(struct device_node *node)
{
	struct pegmatite_sscg *sscg;
	struct clk *clk;
	struct clk *parent_clk;
	struct clk_init_data *init;
	const char *parent_name;
	int table_count = 0, i;
	u32 val;

	sscg = kzalloc(sizeof(*sscg), GFP_KERNEL);
	if (!sscg) {
		pr_err("%s: could not allocate sscg clk\n", __func__);
		return;
	}

	init = kzalloc(sizeof(*init), GFP_KERNEL);
	if (!init) {
		pr_err("%s: could not allocate sscg init\n", __func__);
		goto free_out;
	}

	sscg->base = of_iomap(node, 0);
	if(WARN_ON(!sscg->base))
		goto free_out2;

	/*
	 * Check if sscg is marked as disabled
	 */
	if (of_find_property(node, "sscg-disabled", NULL))
		sscg->sscg_disabled = 1;

	/*
	 * If we find an sscg-table, we enable spread
	 */
	table_count = of_property_count_u32_elems(node, "sscg-table");
	if (table_count > 0 && !sscg->sscg_disabled) {
		val = 0;

		/*
		 * If we are doing down-spread, then we need to know the offset percent to apply
		 * to our reported clock frequency.  If this property is not populated, or 0, we assume
		 * center-spread
		 */
		if (of_property_read_u32(node, "down-spread-offset", &sscg->down_spread_offset)) {
			sscg->down_spread_offset = 0;
		} else {
			val |= DOWN_SPREAD_MASK << DOWN_SPREAD_SHIFT;
		}

		/*
		 * Set the max_addr field to the number of table count minus one
		 */
		val |= ((table_count - 1) & MAX_ADDR_MASK) << MAX_ADDR_SHIFT;

		/*
		 * Get interpolation points
		 */
		if (of_property_read_u32(node, "interp-points", &sscg->interp_points)) {
			sscg->interp_points = 0;
		}

		/*
		 * Set interpolation points
		 */
		val |= (sscg->interp_points & INTERP_POINTS_MASK) << INTERP_POINTS_SHIFT;

		/*
		 * Always apply correction
		 */
		val |= APPLY_CORRECTION_MASK << APPLY_CORRECTION_SHIFT;
		writel(val, sscg->base + CSSCG_MODE_CONFIG_OFFSET);

		for(i = 0; i < table_count; i++) {
			of_property_read_u32_index(node, "sscg-table", i, &val);
			writel(val, sscg->base + CSSCG_RAM(i));
		}

		/*
		 * Always apply correction
		 */
		writel(CSSCG_ENABLED, sscg->base + CSSCG_CONTROL_OFFSET);
	}

	init->name = kasprintf(GFP_KERNEL, "%s", node->name);
	init->ops = &pegmatite_sscg_ops;
	init->flags = 0;
	parent_clk = of_clk_get(node, 0);
	parent_name = __clk_get_name(parent_clk);
	init->parent_names = &parent_name;
	init->num_parents = 1;

	sscg->hw.init = init;

	clk = clk_register(NULL, &sscg->hw);
	if(WARN_ON(IS_ERR(clk)))
		goto map_out;

	of_clk_add_provider(node, of_clk_src_simple_get, clk);

	return;
map_out:
	iounmap(sscg->base);
free_out2:
	kfree(init);
free_out:
	kfree(sscg);
}

CLK_OF_DECLARE(pegmatite_sscg, "marvell,pegmatite-sscg", of_pegmatite_sscg_setup);
