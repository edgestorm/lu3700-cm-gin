
#include <linux/platform_device.h>
#include <linux/pm.h>

int lge_pwrsink_suspend_noirq(struct device *dev)
{
	struct dev_pm_ops *pm_ops = (struct dev_pm_ops*)dev_get_drvdata(dev);

	if(pm_ops && pm_ops->suspend_noirq)
		return pm_ops->suspend_noirq(dev);

	printk(KERN_ERR"%s: pm_ops or suspend_noirq func ptr is null\n", __func__); 
	return 0;
}

int lge_pwrsink_resume_noirq(struct device *dev)
{
	struct dev_pm_ops *pm_ops = (struct dev_pm_ops*)dev_get_drvdata(dev);

	if(pm_ops && pm_ops->resume_noirq)
		return pm_ops->resume_noirq(dev);

	printk(KERN_ERR"%s: pm_ops or resume_noirq func ptr is null\n", __func__); 
	return 0;
}

static int lge_pwrsink_probe(struct platform_device *pdev)
{
	printk(KERN_INFO"%s: setting driver data\n", __func__);
	dev_set_drvdata(&pdev->dev, pdev->dev.platform_data);
	return 0;
}

static int lge_pwrsink_remove(struct platform_device *dev)
{
	printk(KERN_INFO"%s\n", __func__);
	return 0;
}

static struct dev_pm_ops lge_pwrsink_pm_ops = {
	.suspend_noirq = lge_pwrsink_suspend_noirq,
	.resume_noirq = lge_pwrsink_resume_noirq,
};

static struct platform_driver lge_pwrsink_driver = {
	.probe = lge_pwrsink_probe,
	.remove = lge_pwrsink_remove,
	.driver = {
		.name = "lge-pwrsink",
		.pm = &lge_pwrsink_pm_ops,
	},
};

static int __init lge_pwrsink_init(void)
{
	printk(KERN_INFO "LGE Power Sink Driver Init\n");
	return platform_driver_register(&lge_pwrsink_driver);
}

static void __exit lge_pwrsink_exit(void)
{
	printk(KERN_INFO "LGE Power Sink Driver Exit\n");
	platform_driver_unregister(&lge_pwrsink_driver);
}

module_init(lge_pwrsink_init);
module_exit(lge_pwrsink_exit);

MODULE_AUTHOR("LG Electronics Inc.");
MODULE_DESCRIPTION("LGE Power Sink Driver");
MODULE_LICENSE("GPL");

