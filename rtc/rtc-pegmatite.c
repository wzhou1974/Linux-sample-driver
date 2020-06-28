/*
 * Driver for the RTC in Marvell 62x0 SoCs.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/gfp.h>
#include <linux/module.h>

#define RTC_STATUS 0x0
#define RTC_INT1   0x4
#define RTC_INT2   0x8
#define RTC_TIME   0xc
#define RTC_ALRM1  0x10
#define RTC_ALRM2  0x14
#define RTC_CC     0x18
#define RTC_TEST   0x1c

#define RTC_INT1_DISABLED 0
#define RTC_INT1_ENABLED  1

#define RTC_INT2_DISABLED 0
#define RTC_INT2_ENABLED  1

#define RTC_ALRM1_MASK  0x1
#define RTC_ALRM2_MASK  0x2

/*
 * Because of the wicked slow bus speed the spec suggests waiting 5us between register operations,
 * but it seems like we really need 10us
 */
#define writel_delay(x,y)	({ writel(x,y); udelay(10); })
#define readl_delay(x)		({ u32 __v = readl(x); udelay(10); __v; })

struct pegmatite_rtc_data {
	struct rtc_device *rtc;
	void __iomem *ioaddr;
	int		irq;
};

static int pegmatite_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct pegmatite_rtc_data *pdata = dev_get_drvdata(dev);
	void __iomem *ioaddr = pdata->ioaddr;
	unsigned long seconds = 0;

	/* convert to seconds */
	rtc_tm_to_time(tm, &seconds);

	/* spec says you need to write twice */
	writel_delay(seconds, ioaddr + RTC_TIME);
	writel_delay(seconds, ioaddr + RTC_TIME);
	return 0;
}

static int pegmatite_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct pegmatite_rtc_data *pdata = dev_get_drvdata(dev);
	void __iomem *ioaddr = pdata->ioaddr;
	unsigned long seconds;

	seconds = readl_delay(ioaddr + RTC_TIME);

	/* convert to rtc_time */
	rtc_time_to_tm(seconds, tm);

	return rtc_valid_tm(tm);
}

static int pegmatite_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	struct pegmatite_rtc_data *pdata = dev_get_drvdata(dev);
	void __iomem *ioaddr = pdata->ioaddr;
	unsigned long seconds;

	seconds = readl_delay(ioaddr + RTC_ALRM1);

	/* convert to rtc_time */
	rtc_time_to_tm(seconds, &alm->time);

	if (rtc_valid_tm(&alm->time) < 0) {
		dev_err(dev, "retrieved alarm date/time is not valid.\n");
		rtc_time_to_tm(0, &alm->time);
	}

	alm->enabled = !!(RTC_INT1_ENABLED & readl_delay(ioaddr + RTC_INT1));

	return 0;
}

static int pegmatite_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	struct pegmatite_rtc_data *pdata = dev_get_drvdata(dev);
	void __iomem *ioaddr = pdata->ioaddr;
	unsigned long seconds = 0;

	/* convert to seconds */
	rtc_tm_to_time(&alm->time, &seconds);

	writel_delay(seconds, ioaddr + RTC_ALRM1);
	writel_delay(alm->enabled ? RTC_INT1_ENABLED : RTC_INT1_DISABLED, ioaddr + RTC_INT1);

	return 0;
}

static int pegmatite_rtc_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pegmatite_rtc_data *pdata = platform_get_drvdata(pdev);
	void __iomem *ioaddr = pdata->ioaddr;

	if (pdata->irq < 0)
		return -EINVAL; /* fall back into rtc-dev's emulation */

	if (enabled)
		writel_delay(RTC_INT1_ENABLED, ioaddr + RTC_INT1);
	else
		writel_delay(RTC_INT1_DISABLED, ioaddr + RTC_INT1);

	return 0;
}

static irqreturn_t pegmatite_rtc_interrupt(int irq, void *data)
{
	struct pegmatite_rtc_data *pdata = data;
	void __iomem *ioaddr = pdata->ioaddr;

	/* clear interrupt */
	writel_delay(RTC_ALRM1_MASK, ioaddr + RTC_STATUS);

	rtc_update_irq(pdata->rtc, 1, RTC_IRQF | RTC_AF);
	return IRQ_HANDLED;
}

static const struct rtc_class_ops pegmatite_rtc_ops = {
	.read_time	= pegmatite_rtc_read_time,
	.set_time	= pegmatite_rtc_set_time,
};

static const struct rtc_class_ops pegmatite_rtc_alarm_ops = {
	.read_time	= pegmatite_rtc_read_time,
	.set_time	= pegmatite_rtc_set_time,
	.read_alarm	= pegmatite_rtc_read_alarm,
	.set_alarm	= pegmatite_rtc_set_alarm,
	.alarm_irq_enable = pegmatite_rtc_alarm_irq_enable,
};

static int __init pegmatite_rtc_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct pegmatite_rtc_data *pdata;
	u32 test_config;

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pdata->ioaddr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(pdata->ioaddr))
		return PTR_ERR(pdata->ioaddr);

	test_config = readl_delay(pdata->ioaddr + RTC_TEST);
        if(test_config != 0) {
		dev_err(&pdev->dev, "Initial power-up, running reset procedure\n");
		writel_delay(0, pdata->ioaddr + RTC_TEST);
		mdelay(500);
		writel_delay(0, pdata->ioaddr + RTC_TIME);
		udelay(62);
		writel_delay(3, pdata->ioaddr + RTC_STATUS);
		udelay(62);
		writel_delay(0, pdata->ioaddr + RTC_INT1);
		writel_delay(0, pdata->ioaddr + RTC_INT2);
		writel_delay(0, pdata->ioaddr + RTC_ALRM1);
		writel_delay(0, pdata->ioaddr + RTC_ALRM2);
		writel_delay(0, pdata->ioaddr + RTC_CC);
		writel_delay(0, pdata->ioaddr + RTC_TIME);
		writel_delay(3, pdata->ioaddr + RTC_STATUS);
		udelay(62);
        }

	pdata->irq = platform_get_irq(pdev, 0);

	platform_set_drvdata(pdev, pdata);

	if (pdata->irq >= 0) {
		device_init_wakeup(&pdev->dev, 1);
		pdata->rtc = devm_rtc_device_register(&pdev->dev, pdev->name,
						 &pegmatite_rtc_alarm_ops,
						 THIS_MODULE);
	} else {
		pdata->rtc = devm_rtc_device_register(&pdev->dev, pdev->name,
						 &pegmatite_rtc_ops, THIS_MODULE);
	}

	if (IS_ERR(pdata->rtc)) {
		return PTR_ERR(pdata->rtc);
	}

	if (pdata->irq >= 0) {
		writel_delay(RTC_INT1_DISABLED, pdata->ioaddr + RTC_INT1);
		writel_delay(RTC_INT2_DISABLED, pdata->ioaddr + RTC_INT2);
		if (devm_request_irq(&pdev->dev, pdata->irq, pegmatite_rtc_interrupt,
				     IRQF_SHARED,
				     pdev->name, pdata) < 0) {
			dev_warn(&pdev->dev, "interrupt not available.\n");
			pdata->irq = -1;
		}
	}

	return 0;
}

static int __exit pegmatite_rtc_remove(struct platform_device *pdev)
{
	struct pegmatite_rtc_data *pdata = platform_get_drvdata(pdev);

	if (pdata->irq >= 0)
		device_init_wakeup(&pdev->dev, 0);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id rtc_pegmatite_of_match_table[] = {
	{ .compatible = "marvell,pegmatite-rtc", },
	{}
};
MODULE_DEVICE_TABLE(of, rtc_pegmatite_of_match_table);
#endif

static struct platform_driver pegmatite_rtc_driver = {
	.remove		= __exit_p(pegmatite_rtc_remove),
	.driver		= {
		.name	= "rtc-pegmatite",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(rtc_pegmatite_of_match_table),
	},
};

module_platform_driver_probe(pegmatite_rtc_driver, pegmatite_rtc_probe);

MODULE_DESCRIPTION("Pegmatite RTC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rtc-pegmatite");
