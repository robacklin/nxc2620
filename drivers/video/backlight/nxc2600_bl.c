
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/delay.h>
#include <asm/cacheflush.h>
#include <asm/mach-nxc2600/nxc2600.h>

#if 0 //marvin-
#define NXC2600BL_MAX_INTENSITY		(CONFIG_NXC2600_EXTAL_CLOCK/28800/2)

struct nxc2600_backlight 
{
	int powermode;
	int current_intensity;

	struct device *dev;
};
#endif

static void inline nxc2600bl_send_intensity(int intensity)
{
	uint32_t clock = 28800*intensity;
#ifdef CONFIG_NXC2600_PANEL_TFP507MWVGAHBE_03
	clock = 28800*intensity;
#endif
	nxc2600_pwm_disable(CONFIG_BACKLIGHT_NXC2600_PWM);
	nxc2600_pwm_init(CONFIG_BACKLIGHT_NXC2600_PWM, clock);
	if(intensity)
		nxc2600_pwm_enable(CONFIG_BACKLIGHT_NXC2600_PWM);
}

static void inline nxc2600bl_send_enable(int enable)
{
	if(enable)
		nxc2600_pwm_enable(CONFIG_BACKLIGHT_NXC2600_PWM);
	else
		nxc2600_pwm_disable(CONFIG_BACKLIGHT_NXC2600_PWM);
}

static void nxc2600bl_blank(struct nxc2600_backlight *bl, int mode)
{
	switch (mode) {
	case FB_BLANK_NORMAL:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_POWERDOWN:
		nxc2600bl_send_intensity(0);
		nxc2600bl_send_enable(0);
		break;

	case FB_BLANK_UNBLANK:
		nxc2600bl_send_intensity(bl->current_intensity);
		nxc2600bl_send_enable(1);
		break;
	}
}

#ifdef CONFIG_PM
static int nxc2600bl_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct backlight_device *dev = platform_get_drvdata(pdev);
	struct nxc2600_backlight *bl = dev_get_drvdata(&dev->dev);

	nxc2600bl_blank(bl, FB_BLANK_POWERDOWN);
	return 0;
}

static int nxc2600bl_resume(struct platform_device *pdev)
{
	struct backlight_device *dev = platform_get_drvdata(pdev);
	struct nxc2600_backlight *bl = dev_get_drvdata(&dev->dev);

	nxc2600bl_blank(bl, bl->powermode);
	return 0;
}
#else
#define nxc2600bl_suspend	NULL
#define nxc2600bl_resume	NULL
#endif

static int nxc2600bl_set_power(struct backlight_device *dev, int state)
{
	struct nxc2600_backlight *bl = dev_get_drvdata(&dev->dev);

	nxc2600bl_blank(bl, state);
	bl->powermode = state;

	return 0;
}

static int nxc2600bl_update_status(struct backlight_device *dev)
{
	struct nxc2600_backlight *bl = dev_get_drvdata(&dev->dev);

	if (bl->current_intensity != dev->props.brightness) {
		if (bl->powermode == FB_BLANK_UNBLANK)
			nxc2600bl_send_intensity(dev->props.brightness);
		if (dev->props.brightness > 0 && dev->props.brightness <= NXC2600BL_MAX_INTENSITY ) //marvin+
 		    bl->current_intensity = dev->props.brightness;
	}

	if (dev->props.fb_blank != bl->powermode)
		nxc2600bl_set_power(dev, dev->props.fb_blank);
          
	return 0;
}

static int nxc2600bl_get_intensity(struct backlight_device *dev)
{
	struct nxc2600_backlight *bl = dev_get_drvdata(&dev->dev);
	return bl->current_intensity;
}

static struct backlight_ops nxc2600bl_ops = {
	.get_brightness = nxc2600bl_get_intensity,
	.update_status  = nxc2600bl_update_status,
};

static int nxc2600bl_probe(struct platform_device *pdev)
{
	struct backlight_device *dev;
	struct nxc2600_backlight *bl;

 	bl = kzalloc(sizeof(struct nxc2600_backlight), GFP_KERNEL);
	if (unlikely(!bl))
		return -ENOMEM;

	dev = backlight_device_register("nxc2600-bl", &pdev->dev, bl, &nxc2600bl_ops);
	if (IS_ERR(dev)) {
		kfree(bl);
		return PTR_ERR(dev);
	}

	bl->powermode = FB_BLANK_POWERDOWN;
	bl->current_intensity = 0;

	bl->dev = &pdev->dev;

	platform_set_drvdata(pdev, dev);

	dev->props.fb_blank = FB_BLANK_UNBLANK;
	dev->props.max_brightness = NXC2600BL_MAX_INTENSITY;
	dev->props.brightness = NXC2600BL_MAX_INTENSITY/2;
	if(CONFIG_BACKLIGHT_NXC2600_PWM==0)
		nxc2600_gpio_vs_pwm0(0);
	else
		nxc2600_gpio_vs_pwm1(0);
	nxc2600bl_update_status(dev);

	printk(KERN_INFO "NXC2600 backlight initialised\n");

	return 0;
}

static int nxc2600bl_remove(struct platform_device *pdev)
{
	struct backlight_device *dev = platform_get_drvdata(pdev);
	struct nxc2600_backlight *bl = dev_get_drvdata(&dev->dev);

	backlight_device_unregister(dev);
	kfree(bl);

	return 0;
}

static struct platform_driver nxc2600bl_driver = 
{
	.probe		= nxc2600bl_probe,
	.remove		= nxc2600bl_remove,
	.suspend	= nxc2600bl_suspend,
	.resume		= nxc2600bl_resume,
	.driver		= {
		.name	= "nxc2600-bl",
	},
};

static int __init nxc2600bl_init(void)
{
	return platform_driver_probe(&nxc2600bl_driver,nxc2600bl_probe);
}

static void __exit nxc2600bl_exit(void)
{
	platform_driver_unregister(&nxc2600bl_driver);
}

module_init(nxc2600bl_init);
module_exit(nxc2600bl_exit);

MODULE_AUTHOR("Hardy Weng <hardy.weng@icnexus.com.tw>");
MODULE_DESCRIPTION("NXC2600 Backlight driver");
MODULE_LICENSE("GPL");
