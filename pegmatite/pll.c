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
#include <linux/delay.h>

#define REFDIV_MASK 0x1ff
#define REFDIV_SHIFT 0
#define PLL_BW_SEL_MASK 0x1
#define PLL_BW_SEL_SHIFT 15
#define ICP_MASK 0xf
#define ICP_SHIFT 5
#define CLKOUT_SE_DIV_SEL_MASK 0x7
#define CLKOUT_SE_DIV_SEL_SHIFT 0
#define CLKOUT_DIFF_DIV_SEL_MASK 0x7
#define CLKOUT_DIFF_DIV_SEL_SHIFT 8
#define FBDIV_MASK 0x1ff
#define FBDIV_SHIFT 16
#define CLKOUT_SOURCE_SEL_MASK 0x1
#define CLKOUT_SOURCE_SEL_SHIFT 16
#define CLKOUT_DIF_EN_MASK 0x1
#define CLKOUT_DIF_EN_SHIFT 17
#define PI_LOOP_MODE_MASK 0x1
#define PI_LOOP_MODE_SHIFT 20
#define FREQ_OFFSET_MASK 0x1ffff
#define FREQ_OFFSET_SHIFT 3
#define FREQ_OFFSET_VALID_MASK 0x1
#define FREQ_OFFSET_VALID_SHIFT 1
#define FREQ_OFFSET_EN_MASK 0x1
#define FREQ_OFFSET_EN_SHIFT 0
#define FREQ_OFFSET_INTPR_MASK    0x7
#define FREQ_OFFSET_INTPR_SHIFT   24
#define FREQ_OFFSET_FD_MASK       0x3
#define FREQ_OFFSET_FD_SHIFT      22
#define KVCO_MASK 0xf
#define KVCO_SHIFT 0
#define FBCLK_EXT_MSK 0x1
#define FBCLK_EXT_SHIFT 16
#define PU_MASK 0x1
#define PU_SHIFT 17
#define PI_EN_MASK 0x1
#define PI_EN_SHIFT 1
#define CLK_DET_MASK 0x4
#define BYPASS_EN_MASK 0x1
#define BYPASS_EN_SHIFT 16
#define RESET_MASK 0x1
#define RESET_SHIFT 16
#define RESET_SSC_MASK 0x1
#define RESET_SSC_SHIFT 17
#define RESET_PI_MASK 0x1
#define RESET_PI_SHIFT 18
#define PLL_LOCK_MASK 0x1
#define PLL_LOCK_SHIFT 0
#define VDDL_DESKEW_MASK 0x5
#define VDDL_SHIFT 0

struct pll_regs {
	volatile uint32_t rst_prediv;
	volatile uint32_t mult_postdiv;
	volatile uint32_t kvco;
	volatile uint32_t misc;
	volatile uint32_t feedback_mode_deskew;
	volatile uint32_t offset_mode;
	volatile uint32_t fixed_mode_ssc_mode;
	volatile uint32_t ssc_freq_ssc_range;
	volatile uint32_t clk_control_marvell_test;
	volatile uint32_t lock_state;
	volatile uint32_t reserve_out;
};

#define to_pegmatite_pll(_hw) container_of(_hw, struct pegmatite_pll, hw)
struct pegmatite_pll {
	struct clk_hw		hw;
	struct pll_regs		*regs;
	int			predivider;
	unsigned int		deskew;
};

static unsigned long pegmatite_pll_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct pegmatite_pll *pll = to_pegmatite_pll(hw);
	unsigned int refdiv;
	unsigned int vcodiv;
	unsigned int clkout_div_sel;
	unsigned int clkout_source_sel;
	unsigned int freq_offset = 0;
	unsigned int freq_offset_en;
	unsigned int fbdiv;
	unsigned int calc_rate;
	u64 calc_rate_64;
	int val;

	/*
	 * If the pll is in bypass, then return the parent_rate
	 */
	val = readl(&pll->regs->fixed_mode_ssc_mode);
	if(val & (BYPASS_EN_MASK << BYPASS_EN_SHIFT)) {
		pr_err("%s: %s is in bypass!\n", __func__, __clk_get_name(hw->clk));
		return parent_rate;
	}

	/*
	 * If the pll is not powered up, then return zero
	 */
	if((val & (PU_MASK << PU_SHIFT)) != (PU_MASK << PU_SHIFT)) {
		pr_err("%s: %s is not powered up!\n", __func__, __clk_get_name(hw->clk));
		return 0;
	}

	/*
	 * If the pll is in reset, then return zero
	 */
	val = readl(&pll->regs->rst_prediv);
	if(val & (RESET_MASK << RESET_SHIFT)) {
		pr_err("%s: %s is in reset!\n", __func__, __clk_get_name(hw->clk));
		return 0;
	}

	/*
	 * Get the reference divider
	 */
	refdiv =  (val >> REFDIV_SHIFT) & REFDIV_MASK;

	/*
	 * Get the Post Divider For Single-ended Output and the Feedback Divider
	 */
	val = readl(&pll->regs->mult_postdiv);
	clkout_div_sel = (val >> CLKOUT_SE_DIV_SEL_SHIFT) & CLKOUT_SE_DIV_SEL_MASK;
	fbdiv = (val >> FBDIV_SHIFT) & FBDIV_MASK;

	/*
	 * Get the Source Select
	 */
	val = readl(&pll->regs->clk_control_marvell_test);
	clkout_source_sel = (val >> CLKOUT_SOURCE_SEL_SHIFT) & CLKOUT_SOURCE_SEL_MASK;

	/*
	 * Get the Frequency Offset Enable and (maybe) the Frequency Offset
	 */
	val = readl(&pll->regs->offset_mode);
	freq_offset_en = (val >> FREQ_OFFSET_EN_SHIFT) & FREQ_OFFSET_EN_MASK;
	if(freq_offset_en) {
		freq_offset = (val >> FREQ_OFFSET_SHIFT) & FREQ_OFFSET_MASK;
	}

	/*
	 * Use the Post Divider For Single-ended Ouput value to determine vcodiv
	 */
	vcodiv = 1 << clkout_div_sel;

	/*
	 * Calculate the rate (w/o frequency offset)
	 */
	if(pll->deskew) {
		calc_rate_64 = (u64)parent_rate * fbdiv;
		do_div(calc_rate_64, refdiv * vcodiv);
		calc_rate = (unsigned int)calc_rate_64;
		return calc_rate;
	} else {
		calc_rate_64 = (u64)parent_rate * 4 * fbdiv;
		do_div(calc_rate_64, (refdiv * vcodiv));
		calc_rate = (unsigned int)calc_rate_64;
	}

	/*
	 * If there is a Frequency Offset value, determine the offset percent to apply to the calculated clock rate
	 * The formula to calculate freq_offset is freq_offset[15:0] = 2^20 * (abs(offset_percent) / (1 + offset_percent))
	 * We are basically doing the opposite here to determine the offset_percent from freq_offset
	 */
	if(freq_offset) {
		s64 offset_percent;
		s64 divider;
		s64 freq_bump;

		/*
		 * Use the first 16 bits of freq_offset to calculate the offset percentage
		 */
		offset_percent = (s64) freq_offset & 0xffff;

		/*
		 * The divider is 2^20 + freq_offset
		 */
		divider = 1048576 + offset_percent;

		/*
		 * Since we can't do floating point math in the kernel we multiple by 100000000
		 */
		offset_percent *= 100000000;

		/*
		 * Divide our offset_percent (so far) by the divider
		 */
		offset_percent = div_s64(offset_percent, divider);

		/*
		 * Multiple the offset percentage by the calculated rate and then divide by 100000000 (to undo the multiply we did earlier)
		 */
		freq_bump = div_s64((calc_rate * offset_percent), 100000000);

		/*
		 * Bit 17 of freq_offset is the sign of the offset percentage
		 */
		if(freq_offset & 0x10000) {
			calc_rate += freq_bump;
		} else {
			calc_rate -= freq_bump;
		}
	}
	return calc_rate;
}

static int pegmatite_pll_set_rate(struct clk_hw *hw, unsigned long rate, unsigned long parent_rate)
{
	struct pegmatite_pll *pll = to_pegmatite_pll(hw);
	unsigned int pll_bw_sel = 0;
	unsigned int refdiv = 1;
	unsigned int best_refdiv;
	unsigned int icp = 0;
	unsigned int vcodiv;
	unsigned int clkout_div_sel = 0;
	unsigned int kvco = 0;
	unsigned int fbdiv;
	unsigned int calc_rate;
	unsigned int best_calc_rate;
	unsigned int freq_offset = 0;
	unsigned int fvco;
	unsigned int frefdiv;
	unsigned int timeout = 1000;
	int val;
	u64 calc_rate_64;

	vcodiv = 1;
	if (pll->deskew) {
		/*
		 * Default vcodiv to 1 and then increment until the vco frequency
		 * is closest to 3GHz without going over
		 */
		if (rate > 0)
		{
			while((unsigned long long)rate * vcodiv <= 3000000000ull) {
				vcodiv *= 2;
			}
			vcodiv /= 2;
		}
	}
	else {
		/*
		 * Default vcodiv to 1 and then increment until the vco frequency
		 * is greater than 1GHz and less than 4GHz
		 */
		vcodiv = 1;
		while(((unsigned long long)rate * vcodiv < 1000000000ull) ||
			  ((unsigned long long)rate * vcodiv > 4000000000ull)) {
			vcodiv++;
		}
	}

	/*
	 * Set the Post Divider Single-ended Divide based on vcodiv
	 * These values come from the PLL spec
	 */
	switch(vcodiv) {
		case 1:
			clkout_div_sel=0;
			break;
		case 2:
			clkout_div_sel=1;
			break;
		case 4:
			clkout_div_sel=2;
			break;
		case 8:
			clkout_div_sel=3;
			break;
		case 16:
			clkout_div_sel=4;
			break;
		case 32:
			clkout_div_sel=5;
			break;
		case 64:
			clkout_div_sel=6;
			break;
		case 128:
			clkout_div_sel=7;
			break;
	}

	/*
	 * fvco is the target rate multiplied by vcodiv
	 */
	fvco = rate * vcodiv;

	/*
	 * Use fvco to set kvco Frequency Range
	 * These values come from the PLL spec
	 */
	if(fvco >= 1200000000 && fvco < 1350000000) {
		kvco=8;
	} else if(fvco >= 1350000000 && fvco < 1500000000) {
		kvco=9;
	} else if(fvco >= 1500000000 && fvco < 1750000000) {
		kvco=10;
	} else if(fvco >= 1750000000 && fvco < 2000000000) {
		kvco=11;
	} else if(fvco >= 2000000000u && fvco < 2200000000u) {
		kvco=12;
	} else if(fvco >= 2200000000u && fvco < 2400000000u) {
		kvco=13;
	} else if(fvco >= 2400000000u && fvco < 2600000000u) {
		kvco=14;
	} else if(fvco >= 2600000000u && fvco < 3000000000u) {
		kvco=15;
	}

	/*
	 * Increment the reference divider until parent_rate divided by the
	 * reference divider is greater than or equal to 8Mhz and less that
	 * or equal to 32MHz
	 */
	while(parent_rate/refdiv > 32000000) {
		refdiv++;
	}
	best_refdiv = refdiv;

	/*
	 * frefdiv is our divided reference rate
	 */
	frefdiv = parent_rate / refdiv;

	/*
	 * Calculate the Feedback Divider
	 */
	if(pll->deskew) {
		fbdiv = (fvco / (frefdiv * vcodiv));
	} else {
		fbdiv = (fvco / 4) / frefdiv;
	}

	/*
	 * Calculate the output rate (w/o any Frequency Offset)
	 */
	if(pll->deskew) {
		calc_rate_64 = (u64)parent_rate * fbdiv;
	} else {
		calc_rate_64 = (u64)parent_rate * 4 * fbdiv;
	}
	do_div(calc_rate_64, refdiv * vcodiv);
	best_calc_rate = (unsigned int)calc_rate_64;

	/*
	 * Test calc_rate at refdivs within the pll's range.
	 */
	refdiv++;

	while(parent_rate/refdiv >= 8000000) {
		/*
		 * frefdiv is our divided reference rate
		 */
		frefdiv = parent_rate / refdiv;

		/*
		 * Calculate the Feedback Divider
		 */
		if(pll->deskew) {
			fbdiv = (fvco / (frefdiv * vcodiv));
		} else {
			fbdiv = (fvco / 4) / frefdiv;
		}


		/*
		 * Calculate the output rate (w/o any Frequency Offset)
		 */
		if(pll->deskew) {
			calc_rate_64 = (u64)parent_rate * fbdiv;
		} else {
			calc_rate_64 = (u64)parent_rate * 4 * fbdiv;
		}
		do_div(calc_rate_64, refdiv * vcodiv);
		calc_rate = (unsigned int)calc_rate_64;
		if(abs(best_calc_rate - rate) > abs(calc_rate - rate))
		{
			best_refdiv = refdiv;
			best_calc_rate = calc_rate;
		}
		refdiv++;
	}
	refdiv = best_refdiv;

	/*
	 * frefdiv is our divided reference rate
	 */
	frefdiv = parent_rate / refdiv;

	/*
	 * Calculate the Feedback Divider
	 */
	if(pll->deskew) {
		fbdiv = (fvco / (frefdiv * vcodiv));
	} else {
		fbdiv = (fvco / 4) / frefdiv;
	}

	/*
	 * Calculate the output rate (w/o any Frequency Offset)
	 */
	if(pll->deskew) {
		calc_rate_64 = (u64)parent_rate * fbdiv;
	} else {
		calc_rate_64 = (u64)parent_rate * 4 * fbdiv;
	}
	do_div(calc_rate_64, refdiv * vcodiv);
	calc_rate = (unsigned int)calc_rate_64;

	/*
	 * Set the Charge Pump Current based on Pll Bandwidth Select and frefdiv
	 * These values come from the PLL spec
	 * If the parent rate is greater than 16Mhz, set pll Bandwidth Select to 1
	 */
	if(frefdiv >= 8000000 && frefdiv < 9000000) {
		pll_bw_sel = 0;
		icp = 3;
	} else if (frefdiv >= 9000000 && frefdiv < 11000000) {
		pll_bw_sel = 0;
		icp = 2;
	} else if (frefdiv >= 11000000 && frefdiv < 13000000) {
		pll_bw_sel = 0;
		icp = 1;
	} else if (frefdiv >= 13000000 && frefdiv < 16000000) {
		pll_bw_sel = 0;
		icp = 0;
	} else if(frefdiv >= 16000000 && frefdiv < 18000000) {
		pll_bw_sel = 1;
		icp = 7;
	} else if (frefdiv >= 18000000 && frefdiv < 22000000) {
		pll_bw_sel = 1;
		icp = 6;
	} else if (frefdiv >= 22000000 && frefdiv < 26000000) {
		pll_bw_sel = 1;
		icp = 5;
	} else if (frefdiv >= 26000000 && frefdiv < 32000000) {
		pll_bw_sel = 1;
		icp = 4;
	}

	/*
	 * If the calculated rate doesn't match the requested rate, apply
	 * a Frequency Offset to make up the difference
	 * Offset percent is calculated by (calc_rate - fvco) / fvco
	 * The formula to calculate freq_offset is freq_offset[15:0] = 2^20 * (abs(offset_percent) / (1 + offset_percent))
	 */
	if(calc_rate != rate && !pll->deskew) {
		s64 offset_percent;
		s64 divider;

		/*
		 * Start with our calculated rate
		 */
		offset_percent = (s64)(calc_rate);

		/*
		 * Subtract fvco
		 */
		offset_percent -= fvco;

		/*
		 * Since we can't do any floating point math in the kernel, multiply by 100000000
		 */
		offset_percent *= 100000000;

		/*
		 * Divide by fvco
		 */
		offset_percent = div_s64(offset_percent, fvco);

		/*
		 * Bit 17 of freq_offset is the sign of the offset percentage
		 */
		if(offset_percent > 0) {
			freq_offset = 0;
		} else {
			freq_offset = 0x10000;
		}

		/*
		 * The divider is supposed to be (1 + offset_percent), but since we multiplied by 100000000
		 * it needs to be (100000000 + offset_percent)
		 */
		divider = 100000000 + offset_percent;

		/*
		 * Get the absolute value of offset_percent
		 */
		offset_percent = abs64(offset_percent);

		/*
		 * Multiply by 2^20
		 */
		offset_percent *= 1048576;

		/*
		 * Divide by the divider to get freq_offset
		 */
		offset_percent = div_s64(offset_percent, divider);

		freq_offset |= 0xffff & (unsigned int)offset_percent;
	}

	/*
	 * Enable bypass while we set up the pll
	 */
	val = readl(&pll->regs->fixed_mode_ssc_mode);
	val |= (BYPASS_EN_MASK << BYPASS_EN_SHIFT);
	writel(val, &pll->regs->fixed_mode_ssc_mode);

	/*
	 * Put the pll in reset
	 */
	val = readl(&pll->regs->rst_prediv);
	val |= (RESET_MASK << RESET_SHIFT);
	val |= (RESET_PI_MASK << RESET_PI_SHIFT);
	val |= (RESET_SSC_MASK << RESET_SSC_SHIFT);
	writel(val, &pll->regs->rst_prediv);

	/*
	 * Power up and pi_en
	 */
	val = readl(&pll->regs->fixed_mode_ssc_mode);
	val |= (PU_MASK << PU_SHIFT);
	val |= (PI_EN_MASK << PI_EN_SHIFT);
	writel(val, &pll->regs->fixed_mode_ssc_mode);

	/*
	 * Set the reference divider
	 */
	val = readl(&pll->regs->rst_prediv);
	val &= ~(REFDIV_MASK << REFDIV_SHIFT);
	val |= ((refdiv & REFDIV_MASK) << REFDIV_SHIFT);
	writel(val, &pll->regs->rst_prediv);

	/*
	 * Set ICP and Pll Bandwidth Select
	 */
	val = readl(&pll->regs->misc);
	val &= ~(PLL_BW_SEL_MASK << PLL_BW_SEL_SHIFT);
	val &= ~(ICP_MASK << ICP_SHIFT);
	val |= ((pll_bw_sel & PLL_BW_SEL_MASK) << PLL_BW_SEL_SHIFT);
	val |= ((icp & ICP_MASK) << ICP_SHIFT);
	if(pll->deskew) {
		val |= ((VDDL_DESKEW_MASK) << VDDL_SHIFT);
	}
	writel(val, &pll->regs->misc);

	/*
	 * Set Post Divider For Single-ended Ouput and Feedback Divider
	 */
	val = readl(&pll->regs->mult_postdiv);
	val &= ~(CLKOUT_SE_DIV_SEL_MASK << CLKOUT_SE_DIV_SEL_SHIFT);
	val &= ~(CLKOUT_DIFF_DIV_SEL_MASK << CLKOUT_DIFF_DIV_SEL_SHIFT);
	val &= ~(FBDIV_MASK << FBDIV_SHIFT);
	val |= ((clkout_div_sel & CLKOUT_SE_DIV_SEL_MASK) << CLKOUT_SE_DIV_SEL_SHIFT);
	val |= ((clkout_div_sel & CLKOUT_DIFF_DIV_SEL_MASK) << CLKOUT_DIFF_DIV_SEL_SHIFT);
	val |= ((fbdiv & FBDIV_MASK) << FBDIV_SHIFT);
	writel(val, &pll->regs->mult_postdiv);

	/*
	 * Set Source Select
	 */
	val = readl(&pll->regs->clk_control_marvell_test);
	val |= (CLKOUT_SOURCE_SEL_MASK << CLKOUT_SOURCE_SEL_SHIFT);
	if(pll->deskew) {
		val |= (CLKOUT_DIF_EN_MASK << CLKOUT_DIF_EN_SHIFT);
	}
	writel(val, &pll->regs->clk_control_marvell_test);

	/*
	 * Set Frequency Offset Enable, Frequency Offset Valid, Frequency Offset (maybe)
	 * and Phase Interpolator Loop Control
	 */
	val = readl(&pll->regs->offset_mode);
	if(!pll->deskew) {
		val &= ~(FREQ_OFFSET_MASK << FREQ_OFFSET_SHIFT);
		val &= ~(FREQ_OFFSET_VALID_MASK << FREQ_OFFSET_VALID_SHIFT);
		val |= (PI_LOOP_MODE_MASK << PI_LOOP_MODE_SHIFT);
		val |= (FREQ_OFFSET_EN_MASK << FREQ_OFFSET_EN_SHIFT);
		if(freq_offset) {
			val |= ((freq_offset & FREQ_OFFSET_MASK) << FREQ_OFFSET_SHIFT);
			val |= (FREQ_OFFSET_VALID_MASK << FREQ_OFFSET_VALID_SHIFT);
		}
	} else {
		/* Disable Frequency Offset Enable and Frequency Offset Valid */
		val &= ~(FREQ_OFFSET_INTPR_MASK << FREQ_OFFSET_INTPR_SHIFT);
		val &= ~(FREQ_OFFSET_FD_MASK << FREQ_OFFSET_FD_SHIFT);
	}
	writel(val, &pll->regs->offset_mode);

	/*
	 * Clear ssc_freq_ssc_range
	 */
	writel(0, &pll->regs->ssc_freq_ssc_range);

	/*
	 * Set KVCO
	 */
	val = readl(&pll->regs->kvco);
	val &= ~(KVCO_MASK << KVCO_SHIFT);
	val |= ((kvco & KVCO_MASK) << KVCO_SHIFT);
	writel(val, &pll->regs->kvco);

	/*
	 * Enable External Feedback Clock if applicable
	 */
	if(pll->deskew) {
		val = readl(&pll->regs->feedback_mode_deskew);
		val |= (FBCLK_EXT_MSK << FBCLK_EXT_SHIFT);
		writel(val, &pll->regs->feedback_mode_deskew);
	}

	/*
	 * Clear reset
	 */
	val = readl(&pll->regs->rst_prediv);
	val &= ~(RESET_MASK << RESET_SHIFT);
	val &= ~(RESET_PI_MASK << RESET_PI_SHIFT);
	val &= ~(RESET_SSC_MASK << RESET_SSC_SHIFT);
	writel(val, &pll->regs->rst_prediv);

	/*
	 * Wait for lock
	 */
	while(!(readl(&pll->regs->lock_state) & PLL_LOCK_MASK)) {
		if(timeout-- == 0) {
			break;
		}
		udelay(10);
	}

	/*
	 * Take the pll out of bypass and disable phase interpolator if in deskew mode
	 */
	val = readl(&pll->regs->fixed_mode_ssc_mode);
	val &= ~(BYPASS_EN_MASK << BYPASS_EN_SHIFT);
	if(pll->deskew) {
		val &= ~(PI_EN_MASK << PI_EN_SHIFT);
		val &= ~CLK_DET_MASK;
	}
	writel(val, &pll->regs->fixed_mode_ssc_mode);

	return 0;
}

static long pegmatite_pll_round_rate(struct clk_hw *hw, unsigned long rate, unsigned long *prate)
{
	struct pegmatite_pll *pll = to_pegmatite_pll(hw);
	unsigned int refdiv;
	unsigned int best_refdiv;
	int vcodiv;
	unsigned int fbdiv;
	unsigned int calc_rate;
	unsigned int best_calc_rate;
	unsigned int fvco;
	unsigned int frefdiv;
	unsigned int parent_rate = *prate;
	u64 calc_rate_64;

	/*
	 * Default the Reference Divider to 1
	 */
	refdiv = 1;
	/*
	 * Default vcodiv to 1
	 */
	vcodiv = 1;

	if (pll->deskew) {
		/*
		 * Default vcodiv to 1 and then increment until the vco frequency
		 * is closest to 3GHz without going over
		 */
		if (rate > 0)
		{
			while((unsigned long long)rate * vcodiv <= 3000000000ull) {
				vcodiv *= 2;
			}
			vcodiv /= 2;
		}
	}
	else {
		/*
		 * Increment until the vco frequency
		 * is greater than 1GHz and less than 4GHz
		 */
		while(((unsigned long long)rate * vcodiv < 1000000000ull) ||
			  ((unsigned long long)rate * vcodiv > 4000000000ull)) {
			vcodiv++;
		}
	}

	/*
	 * fvco is the target rate multiplied by vcodiv
	 */
	fvco = rate * vcodiv;

	/*
	 * Increment the reference divider until parent_rate divided by the
	 * reference divider is greater than or equal to 8Mhz and less that
	 * or equal to 32MHz
	 */
	while(parent_rate/refdiv > 32000000) {
		refdiv++;
	}
	best_refdiv = refdiv;

	/*
	 * frefdiv is our divided reference rate
	 */
	frefdiv = parent_rate / refdiv;

	/*
	 * Calculate the Feedback Divider
	 */
	if(pll->deskew) {
		fbdiv = (fvco / (frefdiv * vcodiv));
	} else {
		fbdiv = (fvco / 4) / frefdiv;
	}

	/*
	 * Calculate the output rate (w/o any Frequency Offset)
	 */
	if(pll->deskew) {
		calc_rate_64 = (u64)parent_rate * fbdiv;
	} else {
		calc_rate_64 = (u64)parent_rate * 4 * fbdiv;
	}
	do_div(calc_rate_64, refdiv * vcodiv);
	best_calc_rate = (unsigned int)calc_rate_64;

	/*
	 * Test calc_rate at refdivs within the pll's range.
	 */
	refdiv++;
	while(parent_rate/refdiv >= 8000000) {
		/*
		 * frefdiv is our divided reference rate
		 */
		frefdiv = parent_rate / refdiv;

		/*
		 * Calculate the Feedback Divider
		 */
		if(pll->deskew) {
			fbdiv = (fvco / (frefdiv * vcodiv));
		} else {
			fbdiv = (fvco / 4) / frefdiv;
		}

		/*
		 * Calculate the output rate (w/o any Frequency Offset)
		 */
		if(pll->deskew) {
			calc_rate_64 = (u64)parent_rate * fbdiv;
		} else {
			calc_rate_64 = (u64)parent_rate * 4 * fbdiv;
		}
		do_div(calc_rate_64, refdiv * vcodiv);
		calc_rate = (unsigned int)calc_rate_64;
		if(abs(best_calc_rate - rate) > abs(calc_rate - rate))
		{
			best_refdiv = refdiv;
			best_calc_rate = calc_rate;
		}
		refdiv++;
	}
	refdiv = best_refdiv;

	/*
	 * frefdiv is our divided reference rate
	 */
	frefdiv = parent_rate / refdiv;

	/*
	 * Calculate the Feedback Divider
	 */
	if(pll->deskew) {
		fbdiv = (fvco / (frefdiv * vcodiv));
	} else {
		fbdiv = (fvco / 4) / frefdiv;
	}

	/*
	 * Calculate the output rate (w/o any Frequency Offset)
	 */
	if(pll->deskew) {
		calc_rate_64 = (u64)parent_rate * fbdiv;
		do_div(calc_rate_64, refdiv);
	} else {
		calc_rate_64 = (uint64_t)parent_rate * 4 * fbdiv;
		do_div(calc_rate_64, (refdiv * vcodiv));
	}
	calc_rate = (unsigned int)calc_rate_64;

	/*
	 * If the calculated rate doesn't match the requested rate,
	 * see if the calculated rate is within +/- 5% of the requested
	 * rate.  If thats the case the pll's frequency offset
	 * capability can make up the difference
	 */
	if(calc_rate != rate && !pll->deskew) {
		s64 offset_percent;

		/*
		 * Start with our calculated rate
		 */
		offset_percent = (s64)(calc_rate);

		/*
		 * Subtract fvco
		 */
		offset_percent -= fvco;

		/*
		 * Since we can't do any floating point math in the kernel, multiply by 100000000
		 */
		offset_percent *= 100000000;

		/*
		 * Divide by fvco
		 */
		offset_percent = div_s64(offset_percent, fvco);

		/*
		 * If offset_percent is <= 5%, we can acheive the requested rate
		 * Return the rate
		 */
		if(abs64(offset_percent) <= 5000000) {
			return rate;
		}
	}
	return calc_rate;
}

const struct clk_ops pegmatite_pll_ops = {
	.recalc_rate = pegmatite_pll_recalc_rate,
	.set_rate = pegmatite_pll_set_rate,
	.round_rate = pegmatite_pll_round_rate,
};

static void __init of_pegmatite_pll_setup(struct device_node *node)
{
	struct pegmatite_pll *pll;
	struct clk *clk;
	void __iomem *pll_base;
	struct clk *parent_clk;
	struct clk_init_data *init;
	const char *parent_name;
	unsigned int default_rate;

	pll = kzalloc(sizeof(*pll), GFP_KERNEL);
	if (!pll) {
		pr_err("%s: could not allocate pll clk\n", __func__);
		return;
	}

	init = kzalloc(sizeof(*init), GFP_KERNEL);
	if (!init) {
		pr_err("%s: could not allocate pll init\n", __func__);
		goto free_out;
	}

	/*
	 * clock-frequency can be set to enable a default clock rate for the pll
	 */
	if (of_property_read_u32(node, "clock-frequency", &default_rate)) {
		default_rate = 0;
	}

	/*
	 * Determine whether or not Pll needs to use deskew mode (i.e. lvdspll)
	 */
	pll->deskew = of_property_read_bool(node, "deskew");

	pll_base = of_iomap(node, 0);
	if(WARN_ON(!pll_base))
		goto free_out2;

	init->name = kasprintf(GFP_KERNEL, "%s", node->name);
	init->ops = &pegmatite_pll_ops;
	init->flags = 0;
	parent_clk = of_clk_get(node, 0);
	parent_name = __clk_get_name(parent_clk);
	init->parent_names = &parent_name;
	init->num_parents = 1;

	pll->hw.init = init;
	pll->regs = pll_base;

	clk = clk_register(NULL, &pll->hw);
	if(WARN_ON(IS_ERR(clk)))
		goto map_out;

	of_clk_add_provider(node, of_clk_src_simple_get, clk);

	/*
	 * If a default rate was specified in the device tree, set it here
	 */
	if(default_rate > 0)
		clk_set_rate(clk, default_rate);

	return;
map_out:
	iounmap(pll_base);
free_out2:
	kfree(init);
free_out:
	kfree(pll);
}

CLK_OF_DECLARE(pegmatite_pll, "marvell,pegmatite-pll", of_pegmatite_pll_setup);
