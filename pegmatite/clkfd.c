/*
 * Marvell Pegmatite SoC clock handling.
 *
 * Pegmatite Fractional Divider
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
#include <linux/math64.h>

/*
 * The fraction divider applies only to the UART clocks.  It allows the user
 * to create an arbitrary numerator and denominator to generate very specific
 * output clock frequencies.
 */
#define FD_NUM_SHIFT 16
#define FD_MASK 0xffff

#define to_pegmatite_clkfd(_hw) container_of(_hw, struct pegmatite_clkfd, hw)
struct pegmatite_clkfd {
	struct clk_hw		hw;
	void __iomem		*config;
	unsigned int		num;
	unsigned int		denom;
};

static unsigned long pegmatite_clkfd_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct pegmatite_clkfd *gen = to_pegmatite_clkfd(hw);
	unsigned long rate;
	u32 num, denom;
	u32 val = readl(gen->config + 8);

	num = (val >> FD_NUM_SHIFT) & FD_MASK;
	denom = val & FD_MASK;

	rate = (unsigned long)div64_ul(((u64)parent_rate * denom), (2 * num));

	return rate;
}

static int pegmatite_clkfd_set_rate(struct clk_hw *hw, unsigned long rate, unsigned long parent_rate)
{
	struct pegmatite_clkfd *gen = to_pegmatite_clkfd(hw);
	int val = 0x80008000;

	if (parent_rate > rate) {
		uint64_t a = parent_rate;
		unsigned int num, tmp, denom = 1;

		/*
		 * outfreq = infreq * D / (2 * N)
		 *
		 * This approximates a 16 bit numerator and denominator from below by
		 * proportionally multiplying the infreq by 2 and the denominator by
		 * 2 until numerator/outfreq uses all 16 bits.
		 */
		num = tmp = (unsigned int)div64_ul(a, rate);
		while ((tmp & ~0xffff) == 0) {
			num = tmp;
			a <<= 1;
			denom <<= 1;
			tmp = (unsigned int)div64_ul(a, rate);
		}

		/*
		 * The calculated divisor is an approximation from below so the
		 * resulting outfreq is too fast.  Reign it in by incrementing the
		 * numerator until the outfreq is <= the requested rate.
		 */
		tmp = (unsigned int)div64_ul(((u64)parent_rate * denom), (2 * num));
		while (tmp > rate) {
			if (num>= 0xffff)
				break;
			++num;
			tmp = (unsigned int)div64_ul(((u64)parent_rate * denom), (2 * num));
		}

		val = num << FD_NUM_SHIFT;
		val |= (denom & FD_MASK);
	}

	writel(val, gen->config + 8);

	return 0;
}

static long pegmatite_clkfd_round_rate(struct clk_hw *hw, unsigned long rate, unsigned long *prate)
{
	unsigned long calc_rate = *prate;
	uint64_t a = *prate;
	unsigned int num, tmp, denom = 1;

	/*
	 * See comments in set_rate for questions
	 */
	num = tmp = (unsigned int)div64_ul(a, rate);
	while ((tmp & ~0xffff) == 0) {
		num = tmp;
		a <<= 1;
		denom <<= 1;
		tmp = (unsigned int)div64_ul(a, rate);
	}

	calc_rate = div64_ul((*prate * (u64)denom), (2 * num));
	while (calc_rate > rate) {
		if (num>= 0xffff)
			break;
		++num;
		calc_rate = div64_ul((*prate * (u64)denom), (2 * num));
	}

	return calc_rate;
}

const struct clk_ops pegmatite_clkfd_ops = {
	.recalc_rate = pegmatite_clkfd_recalc_rate,
	.set_rate = pegmatite_clkfd_set_rate,
	.round_rate = pegmatite_clkfd_round_rate,
};

static void __init of_pegmatite_clkfd_setup(struct device_node *node)
{
	struct pegmatite_clkfd *gen;
	struct clk *clk;
	void __iomem *clk_base;
	struct clk *parent_clk;
	struct clk_init_data *init;
	const char *parent_name;
	u32 default_rate;

	gen = kzalloc(sizeof(*gen), GFP_KERNEL);
	if (!gen) {
		pr_err("%s: could not allocate clkfd clk\n", __func__);
		return;
	}

	init = kzalloc(sizeof(*init), GFP_KERNEL);
	if (!init) {
		pr_err("%s: could not allocate clkfd init\n", __func__);
		goto free_out;
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
	init->ops = &pegmatite_clkfd_ops;
	init->flags = 0;
	parent_clk = of_clk_get(node, 0);
	parent_name = __clk_get_name(parent_clk);
	init->parent_names = &parent_name;
	init->num_parents = 1;

	gen->hw.init = init;
	gen->config = clk_base;

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

CLK_OF_DECLARE(pegmatite_clkfd, "marvell,pegmatite-clkfd", of_pegmatite_clkfd_setup);
