/*
 * drivers/watchdog/pegmatite_wdt.c
 *
 * Watchdog driver for Pegmatite processors
 *
 * This file is licensed under  the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/watchdog.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/interrupt.h>
#include <linux/smp.h>
#include <linux/irq.h>
#include <linux/irqdesc.h>
#include <linux/reboot.h>
#include <trace/events/irq.h>
#include <linux/irqchip/arm-gic.h>
#include "watchdog_pretimeout.h"

/*
 * MPMU Watchdog timer block registers.
 */
#define TWR	0x0
#define TTCR	0x4
#define TCR	0x8
#define TSR	0xc
#define TISR	0x10
#define TIAR	0x20

#define WDT_ENABLE		0x80000000
#define WDT_SERVICE_MASK	0xffff
#define WDT_TIMEBASE_MASK	0x70
#define WDT_TIMEBASE_SHIFT      4
#define WDT_ONE_US		0
#define WDT_TEN_US		1
#define WDT_100_US		2
#define WDT_ONE_MS		3
#define WDT_TEN_MS		4
#define WDT_100_MS		5
#define WDT_BUS_CLK		6
#define WDT_EXTERNAL		7
#define WDT_CONTINUOUS_MODE	0x2
#define WDT_TIMER_ENABLE	0x1
#define WDT_MAX_CYCLE_COUNT	0xffffffff
#define WDT_MAX_DURATION	0xffffffff / 1000

/*
 * APS timers_mv Watchdog timer block registers.
 */
#define APS_TMR_WMER	0x64 // Timers Watchdog Match Enable Register
#define APS_TMR_WMR	0x68 // Timers Watchdog Match Register
#define APS_TMR_WVR	0x6c // Timers Watchdog Value Register
#define APS_TMR_WSR	0x70 // Timers Watchdog Status Register
#define APS_TMR_WICR	0x80 // Timers Watchdog Interrupt Clear Register
#define APS_TMR_WCR	0x98 // Timers Watchdog Counter Reset Register
#define APS_TMR_WFAR	0x9c // Timers Watchdog First Access Register
#define APS_TMR_WSAR	0xa0 // Timers Watchdog Second Access Register

#define APS_TMR_HZ	256
#define APS_TMR_MAX	0xffff
#define APS_TMR_MAGIC1	0xbaba
#define APS_TMR_MAGIC2	0xeb10

static bool nowayout = WATCHDOG_NOWAYOUT;
static int heartbeat = -1;		/* module parameter (seconds) */

struct pegmatite_wdt_data {
	struct watchdog_device wdt;
	void __iomem *reg;	/* Regs in MPMU for TIMERS watchdog */
	void __iomem *aps_reg;	/* Regs in APS for timers_mv watchdog */
	int irq;
	int irq_enabled;
	spinlock_t lock;
};

static int timeout_on_panic = 30;
static DEVICE_INT_ATTR(timeout_on_panic, 0644, timeout_on_panic);

static struct device_attribute *pegmatite_wdt_attrs[] = {
	&dev_attr_timeout_on_panic.attr,
};

static void aps_watchdog_writel(struct pegmatite_wdt_data *wdt, u32 v, int off)
{
	void __iomem *aps_reg = wdt->aps_reg;
	unsigned long flags;
	static DEFINE_SPINLOCK(__aps_writel_lock);

	/* These 3 register operations need to be atomic */
	spin_lock_irqsave(&__aps_writel_lock, flags);
	/* Magic value sequence has to be written before every write */
	writel(APS_TMR_MAGIC1, aps_reg + APS_TMR_WFAR);
	writel(APS_TMR_MAGIC2, aps_reg + APS_TMR_WSAR);
	writel(v, aps_reg + off);
	spin_unlock_irqrestore(&__aps_writel_lock, flags);
}

static int pegmatite_wdt_ping(struct watchdog_device *wdt_dev)
{
	struct pegmatite_wdt_data *wdt = watchdog_get_drvdata(wdt_dev);
	u32 val;

	spin_lock(&wdt->lock);

	/* Reset the timer back to 0 by disabling/enabling it.
	 * We do this so the IRQ only fires as a pre-watchdog (indicating we
	 * never serviced it again in the entire window). */
	val = readl(wdt->reg + TCR);
	writel(val & ~(WDT_TIMER_ENABLE), wdt->reg + TCR);
	writel(val, wdt->reg + TCR);

	/* Read the service field of TWR */
	val = (readl(wdt->reg + TWR) & WDT_SERVICE_MASK);

	/* xor it with 0xf */
	val ^= 0xf;

	/* Write it back */
	writel(WDT_ENABLE | val, wdt->reg + TWR);

	/* Service the APS timers_mv watchdog block */
	aps_watchdog_writel(wdt, 1, APS_TMR_WCR);

	spin_unlock(&wdt->lock);
	return 0;
}

static void  __set_hw_pretimeout(struct watchdog_device *wdt_dev)
{
	struct pegmatite_wdt_data *wdt = watchdog_get_drvdata(wdt_dev);
	u32 expiry;
	int percent;

	/* This does not overflow since the clock rate is so low */
	percent = watchdog_pretimeout_percent();
	expiry = (percent * wdt_dev->timeout * APS_TMR_HZ) / 100;
	if (expiry > APS_TMR_MAX)
		expiry = APS_TMR_MAX;

	aps_watchdog_writel(wdt, expiry, APS_TMR_WMR);
}

static void  __set_hw_timeout(struct watchdog_device *wdt_dev)
{
	struct pegmatite_wdt_data *wdt = watchdog_get_drvdata(wdt_dev);

	/*
	 * Set watchdog duration in milliseconds. Note, this hardware wraps
	 * twice before it actually resets the system, so we need to set it for
	 * half the requested timeout.
	 */
	writel(wdt_dev->timeout * 1000 / 2, wdt->reg + TTCR);

	__set_hw_pretimeout(wdt_dev);
}

static int pegmatite_wdt_stop_unlocked(struct watchdog_device *wdt_dev)
{
	struct pegmatite_wdt_data *wdt = watchdog_get_drvdata(wdt_dev);
	u32 val;

	/* Read the service field of TWR */
	val = (readl(wdt->reg + TWR) & WDT_SERVICE_MASK);

	/* xor it with 0xf */
	val ^= 0xf;

	writel((~WDT_ENABLE) & val, wdt->reg + TWR);

	/* Disable timer */
	writel(~WDT_TIMER_ENABLE, wdt->reg + TCR);

	return 0;
}

static int pegmatite_wdt_disable_pretimeout_irq(struct watchdog_device *wdt_dev)
{
	struct pegmatite_wdt_data *wdt = watchdog_get_drvdata(wdt_dev);

	if (wdt->irq_enabled) {
		disable_irq_nosync(wdt->irq);
		wdt->irq_enabled = 0;
	}
	/* Disable APS watchdog block */
	aps_watchdog_writel(wdt, 0, APS_TMR_WMER);
	/* Clear any pending interrupt */
	aps_watchdog_writel(wdt, 1, APS_TMR_WICR);
	/* Clear the WDT timer count */
	aps_watchdog_writel(wdt, 1, APS_TMR_WCR);
	return 0;
}

static int pegmatite_wdt_stop(struct watchdog_device *wdt_dev)
{
	struct pegmatite_wdt_data *wdt = watchdog_get_drvdata(wdt_dev);

	spin_lock(&wdt->lock);
	pegmatite_wdt_stop_unlocked(wdt_dev);
	if (watchdog_pretimeout_enabled())
		pegmatite_wdt_disable_pretimeout_irq(wdt_dev);
	spin_unlock(&wdt->lock);
	return 0;
}

static int pegmatite_wdt_enable_pretimeout_irq(struct watchdog_device *wdt_dev)
{
	struct pegmatite_wdt_data *wdt = watchdog_get_drvdata(wdt_dev);

	/* Clear any old IRQ that may be latched */
	aps_watchdog_writel(wdt, 1, APS_TMR_WICR);
	/* Clear the WDT timer count */
	aps_watchdog_writel(wdt, 1, APS_TMR_WCR);
	/* Enable the APS timers_mv watchdog block in IRQ only mode */
	aps_watchdog_writel(wdt, 1, APS_TMR_WMER);
	if (!wdt->irq_enabled) {
		enable_irq(wdt->irq);
		wdt->irq_enabled = 1;
	}
	return 0;
}

static int pegmatite_wdt_start(struct watchdog_device *wdt_dev)
{
	struct pegmatite_wdt_data *wdt = watchdog_get_drvdata(wdt_dev);

	spin_lock(&wdt->lock);

	/*
	 * If wdog was already armed (e.g. if we just booted into a crash
	 * kernel and it was still running from a prev. session), then the
	 * below will look like an invalid servicing of the wdog, typically
	 * causing an immediate system reset. Simple solution is to disarm the
	 * wdog first.
	 */
	pegmatite_wdt_stop_unlocked(wdt_dev);

	__set_hw_timeout(wdt_dev);

	/* Set timer enable, continuous mode, and 1ms timebase */
	writel(WDT_TIMER_ENABLE | WDT_CONTINUOUS_MODE | (WDT_ONE_MS << WDT_TIMEBASE_SHIFT), wdt->reg + TCR);

	writel(WDT_ENABLE, wdt->reg + TWR);

	if (watchdog_pretimeout_enabled())
		pegmatite_wdt_enable_pretimeout_irq(wdt_dev);

	spin_unlock(&wdt->lock);
	return 0;
}

static unsigned int pegmatite_wdt_get_timeleft(struct watchdog_device *wdt_dev)
{
	struct pegmatite_wdt_data *wdt = watchdog_get_drvdata(wdt_dev);
	unsigned int time_left;
	unsigned int terminal_count;

	spin_lock(&wdt->lock);

	/* Read terminal count. This watchdog block only resets the system when
	 * the termincal count in TTCR is reached *and* no service ping has
	 * occurred since the last time the continuous timer wrapped. So, it
	 * takes two terminal counts to expire. */
	terminal_count = readl(wdt->reg + TTCR);

	/* Read the current count. Subtract from twice the terminal count to
	 * figure up the remaining time. There is no way to be sure we have
	 * actually serviced it in this interval. */
	time_left = (2*terminal_count - readl(wdt->reg + TSR)) / 1000;

	spin_unlock(&wdt->lock);

	return time_left;
}

/*
 * We don't need any locks here, and need to avoid them anyways because this
 * can be called when we're panicking from any context
 */
static int pegmatite_wdt_set_timeout(struct watchdog_device *wdt_dev,
				 unsigned int timeout)
{
	wdt_dev->timeout = timeout;
	__set_hw_timeout(wdt_dev);

	return 0;
}

static irqreturn_t pegmatite_wdt_irq(int irq, void *dev_id)
{
	struct watchdog_device *wdt_dev = dev_id;

	watchdog_pretimeout_handle();

	/* The above only returns if the pretimeout is disabled.
	 * So, disable the pretimeout irq and return. */
	pegmatite_wdt_disable_pretimeout_irq(wdt_dev);

	return IRQ_HANDLED;
}

struct pegmatite_wdt_notifier_block {
	struct notifier_block nblock;
	struct watchdog_device *wdt_dev;
};

/*
 * When a panic occurs, we may want to change (e.g. increase) the timeout in
 * order to extend watchdog protection into a crash kernel, while giving it
 * enough time to boot and start servicing the watchdog itself.  This is mostly
 * important in an embedded system, when the normal timeout may have to be very
 * short in order to protect equipment under software control. Note that this
 * will not enable the watchdog if it wasn't already enabled.
 *
 * Configured by timeout_on_panic dev attr:
 * If positive, change the timeout to this value.
 * If zero, stop the watchdog.
 * If negative, do nothing (leave the timeout alone).
 */
static int pegmatite_wdt_panic_notification(struct notifier_block *n,
	unsigned long val, void *v)
{
	struct pegmatite_wdt_notifier_block *pnb =
		(struct pegmatite_wdt_notifier_block *)n;

	if (timeout_on_panic > 0) {
		pegmatite_wdt_set_timeout(pnb->wdt_dev, timeout_on_panic);
	} else if (timeout_on_panic == 0) {
		/*
		 * Racy if panic occurred while our spinlock was held. But if
		 * we were to get stuck waiting on our lock while we're
		 * panicking, it won't help anything, so take a chance.
		 */
		pegmatite_wdt_stop_unlocked(pnb->wdt_dev);
	}

	return 0;
}
static struct pegmatite_wdt_notifier_block pegmatite_wdt_panic_notifier = {
	.nblock = {
		.notifier_call = pegmatite_wdt_panic_notification,
		.priority = -1000000,  // Want to be "very late"
	},
};

static const struct watchdog_info pegmatite_wdt_info = {
	.options = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE,
	.identity = "Pegmatite Watchdog",
};

static const struct watchdog_ops pegmatite_wdt_ops = {
	.owner = THIS_MODULE,
	.start = pegmatite_wdt_start,
	.stop = pegmatite_wdt_stop,
	.ping = pegmatite_wdt_ping,
	.set_timeout = pegmatite_wdt_set_timeout,
	.get_timeleft = pegmatite_wdt_get_timeleft,
};

/* We want our level interrupt to stay set in hardware, allowing all
 * cores to handle the interrupt simultaneously. The chip layer will interfere
 * by marking one of the cores as handling it, so use our own irq handler. */
static void handle_remain_latched_and_eoi_irq(unsigned int irq, struct irq_desc *desc)
{
	struct irqaction *action = desc->action;
	struct irq_chip *chip = irq_desc_get_chip(desc);
	irqreturn_t res;

	if (chip->irq_ack)
		chip->irq_ack(&desc->irq_data);

	trace_irq_handler_entry(irq, action);

	/* This flavor of interrupt is very odd. We end it before handling it
	 * on purpose so other cores can also get it. The interrupt action
	 * usually leaves the interrupt latched in hardware and never returns.
	 * This is only really useful for watchdog pretimeout */
	if (chip->irq_eoi)
		chip->irq_eoi(&desc->irq_data);

	res = action->handler(irq, action->dev_id);

	trace_irq_handler_exit(irq, action, res);
}

/* Global reference only for the restart handler */
static struct watchdog_device *reboot_watchdog_dev;

static int pegmatite_wdt_system_restart(struct notifier_block *nb, unsigned long mode, void *cmd)
{
	/* Set timeout to a small value */
	reboot_watchdog_dev->timeout = 1;
	pegmatite_wdt_start(reboot_watchdog_dev);

	while (true)
		;

	return NOTIFY_DONE;
}

static struct notifier_block pegmatite_wdt_restart_handler = {
	.notifier_call = pegmatite_wdt_system_restart,
	.priority = 128,
};

static int pegmatite_wdt_probe(struct platform_device *pdev)
{
	int ret, irq;
	struct watchdog_device *wdt_dev;
	struct pegmatite_wdt_data *wdt;
	int i;

	wdt_dev = devm_kzalloc(&pdev->dev, sizeof(*wdt_dev), GFP_KERNEL);
	if (!wdt_dev) {
		dev_err(&pdev->dev, "Failed to allocate memory for watchdog device");
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, wdt_dev);

	wdt = devm_kzalloc(&pdev->dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt) {
		dev_err(&pdev->dev, "Failed to allocate memory for watchdog data");
		return -ENOMEM;
	}
	wdt_dev->info = &pegmatite_wdt_info;
	wdt_dev->ops = &pegmatite_wdt_ops;
	wdt_dev->min_timeout = 1;
	wdt_dev->max_timeout = WDT_MAX_DURATION;
	wdt_dev->timeout = WDT_MAX_DURATION;
	watchdog_set_drvdata(wdt_dev, wdt);

	spin_lock_init(&wdt->lock);

	wdt->reg = devm_ioremap_resource(&pdev->dev,
			platform_get_resource(pdev, IORESOURCE_MEM, 0));
	if (!wdt->reg) {
		dev_err(&pdev->dev, "Failed to request MPMU I/O mem\n");
		return -ENOMEM;
	}

	wdt->aps_reg = devm_ioremap_resource(&pdev->dev,
				platform_get_resource(pdev, IORESOURCE_MEM, 1));
	if (!wdt->aps_reg) {
		dev_err(&pdev->dev, "Failed to request APS I/O mem\n");
		return -ENOMEM;
	}

	pegmatite_wdt_disable_pretimeout_irq(wdt_dev);

	irq = platform_get_irq(pdev, 0);
	if (irq > 0) {
		wdt->irq = irq;
		ret = devm_request_irq(&pdev->dev, irq, pegmatite_wdt_irq, IRQF_TRIGGER_HIGH,
				pdev->name, wdt_dev);
		if (ret < 0) {
			dev_err(&pdev->dev, "Failed to request IRQ\n");
			return -ENOMEM;
		}
		disable_irq(irq);
		gic_set_use_fiq(irq, 1);
		irq_set_handler(irq, handle_remain_latched_and_eoi_irq);
	}

	watchdog_init_timeout(wdt_dev, heartbeat, &pdev->dev);
	watchdog_set_nowayout(wdt_dev, nowayout);
	ret = watchdog_register_device(wdt_dev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register watchdog device");
		return ret;
	}

	for (i = 0; i < ARRAY_SIZE(pegmatite_wdt_attrs); i++) {
		ret = device_create_file(&pdev->dev, pegmatite_wdt_attrs[i]);
		if (ret)
			goto out_attrs;
	}
	pegmatite_wdt_panic_notifier.wdt_dev = wdt_dev;
	atomic_notifier_chain_register(&panic_notifier_list,
		&pegmatite_wdt_panic_notifier.nblock);

	pr_info("pegmatite_wdt: Initial timeout %d sec%s\n",
		wdt_dev->timeout, nowayout ? ", nowayout" : "");

	reboot_watchdog_dev = wdt_dev;
	ret = register_restart_handler(&pegmatite_wdt_restart_handler);
	if (ret) {
		dev_err(&pdev->dev, "failed to register restart handler %d\n", ret);
		goto out_attrs;
	}

	return 0;

out_attrs:
	while (--i >= 0)
		device_remove_file(&pdev->dev, pegmatite_wdt_attrs[i]);
	watchdog_unregister_device(wdt_dev);
	return ret;
}

static int pegmatite_wdt_remove(struct platform_device *pdev)
{
	struct watchdog_device *wdt_dev = platform_get_drvdata(pdev);
	int i;

	unregister_restart_handler(&pegmatite_wdt_restart_handler);
	reboot_watchdog_dev = NULL;
	atomic_notifier_chain_unregister(&panic_notifier_list,
		&pegmatite_wdt_panic_notifier.nblock);
	for (i = 0; i < ARRAY_SIZE(pegmatite_wdt_attrs); i++) {
		device_remove_file(&pdev->dev, pegmatite_wdt_attrs[i]);
	}
	pegmatite_wdt_stop(wdt_dev);
	watchdog_unregister_device(wdt_dev);

	return 0;
}

static void pegmatite_wdt_shutdown(struct platform_device *pdev)
{
	struct watchdog_device *wdt_dev = platform_get_drvdata(pdev);
	pegmatite_wdt_stop(wdt_dev);
}

static const struct of_device_id pegmatite_wdt_of_match_table[] = {
	{ .compatible = "marvell,pegmatite-wdt", },
	{},
};
MODULE_DEVICE_TABLE(of, pegmatite_wdt_of_match_table);

static struct platform_driver pegmatite_wdt_driver = {
	.probe		= pegmatite_wdt_probe,
	.remove		= pegmatite_wdt_remove,
	.shutdown	= pegmatite_wdt_shutdown,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "pegmatite_wdt",
		.of_match_table = pegmatite_wdt_of_match_table,
	},
};

module_platform_driver(pegmatite_wdt_driver);

module_param(heartbeat, int, 0);
MODULE_PARM_DESC(heartbeat, "Initial watchdog heartbeat in seconds");

module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started (default="
				__MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pegmatite_wdt");
