/*
 *
 * Copyright (C) 2007 IC Nexus Co., LTD. 
 *
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 *
 */

/*
 *   
 *
 * 2008/04/01: initial
 *
 * Author: Hardy Weng <hardy.weng@icnexus.com.tw>
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/wm97xx.h>
#include <linux/io.h>
#include <asm/cacheflush.h>
#include <asm/mach-nxc2600/nxc2600.h>

struct continuous {
	u16 id;    /* codec id */
	u8 code;   /* continuous code */
	u8 reads;  /* number of coord reads per read cycle */
	u32 speed; /* number of coords per second */
};

#define WM_READS(sp) ((sp / HZ) + 1)

static const struct continuous cinfo[] = {
	{WM9705_ID2, 0, WM_READS(94), 94},
	{WM9705_ID2, 1, WM_READS(188), 188},
	{WM9705_ID2, 2, WM_READS(375), 375},
	{WM9705_ID2, 3, WM_READS(750), 750},
	{WM9712_ID2, 0, WM_READS(94), 94},
	{WM9712_ID2, 1, WM_READS(188), 188},
	{WM9712_ID2, 2, WM_READS(375), 375},
	{WM9712_ID2, 3, WM_READS(750), 750},
	{WM9713_ID2, 0, WM_READS(94), 94},
	{WM9713_ID2, 1, WM_READS(120), 120},
	{WM9713_ID2, 2, WM_READS(154), 154},
	{WM9713_ID2, 3, WM_READS(188), 188},
};

/* continuous speed index */
static int sp_idx;

/*
 * Pen sampling frequency (Hz) in continuous mode.
 */
static int cont_rate = 200;
module_param(cont_rate, int, 0);
MODULE_PARM_DESC(cont_rate, "Sampling rate in continuous mode (Hz)");

/*
 * Pressure readback.
 *
 * Set to 1 to read back pen down pressure
 */
static int pressure;
module_param(pressure, int, 1);
MODULE_PARM_DESC(pressure, "Pressure readback (1 = pressure, 0 = no pressure)");

/*
 * AC97 touch data slot.
 *
 * Touch screen readback data ac97 slot
 */
static int ac97_touch_slot = 5;
module_param(ac97_touch_slot, int, 0);
MODULE_PARM_DESC(ac97_touch_slot, "Touch screen data slot AC97 number");

static void
wm97xx_acc_pen_up(struct wm97xx *wm)
{
}

static int wm97xx_acc_pen_down(struct wm97xx *wm)
{
	return 0;
}

static int wm97xx_acc_startup(struct wm97xx *wm)
{
	int idx = 0;

	/* check we have a codec */
	if (wm->ac97 == NULL)
		return -ENODEV;

	/* Go you big red fire engine */
	for (idx = 0; idx < ARRAY_SIZE(cinfo); idx++) {
		if (wm->id != cinfo[idx].id)
			continue;
		sp_idx = idx;
		if (cont_rate <= cinfo[idx].speed)
			break;
	}
	wm->acc_rate = cinfo[sp_idx].code;
	wm->acc_slot = ac97_touch_slot;
        wm->pen_irq = NXC2600_GPIO_NIRQ(CONFIG_TOUCHSCREEN_WM97XX_IRQ_GPIO_PIN);


	dev_info(wm->dev,
		 "nxc2600 accelerated touchscreen driver, irq %d, %d samples/sec\n",
		wm->pen_irq,
		 cinfo[sp_idx].speed);

	return 0;
}

static void wm97xx_acc_shutdown(struct wm97xx *wm)
{
	/* codec specific deconfig */
	switch (wm->id & 0xffff) {
	case WM9705_ID2:
		wm->pen_irq = 0;
		break;
	case WM9712_ID2:
	case WM9713_ID2:
		/* disable interrupt */
		wm->pen_irq = 0;
		break;
	}
}

static void wm97xx_irq_enable(struct wm97xx *wm, int enable)
{
	if (enable)
	{
		wm97xx_config_gpio(wm, WM97XX_GPIO_13, WM97XX_GPIO_IN,
				   WM97XX_GPIO_POL_HIGH,
				   WM97XX_GPIO_STICKY,
				   WM97XX_GPIO_WAKE);
		wm97xx_config_gpio(wm, WM97XX_GPIO_2, WM97XX_GPIO_OUT,
				   WM97XX_GPIO_POL_HIGH,
				   WM97XX_GPIO_NOTSTICKY,
				   WM97XX_GPIO_NOWAKE);
		enable_irq(wm->pen_irq);
	}
	else
	{
		disable_irq(wm->pen_irq);
	}
}

static struct wm97xx_mach_ops nxc2600_mach_ops = {
	.acc_enabled = 1,
	.acc_pen_up = wm97xx_acc_pen_up,
	.acc_pen_down = wm97xx_acc_pen_down,
	.acc_startup = wm97xx_acc_startup,
	.acc_shutdown = wm97xx_acc_shutdown,
	.irq_enable = wm97xx_irq_enable,
	.irq_gpio = WM97XX_GPIO_3,
};

static int nxc2600_wm97xx_probe(struct platform_device *pdev)
{
	struct wm97xx *wm = platform_get_drvdata(pdev);
	return wm97xx_register_mach_ops(wm, &nxc2600_mach_ops);
}

static int nxc2600_wm97xx_remove(struct platform_device *pdev)
{
	struct wm97xx *wm = platform_get_drvdata(pdev);
	wm97xx_unregister_mach_ops(wm);
	return 0;
}

static struct platform_driver nxc2600_wm97xx_driver = {
	.probe = nxc2600_wm97xx_probe,
	.remove = nxc2600_wm97xx_remove,
	.driver = {
		.name = "wm97xx-touch",
	},
};

static int __init nxc2600_wm97xx_init(void)
{
	printk("wm97xx ts for nxc2600 initialed\n");
	return platform_driver_register(&nxc2600_wm97xx_driver);
}

static void __exit nxc2600_wm97xx_exit(void)
{
	platform_driver_unregister(&nxc2600_wm97xx_driver);
}

module_init(nxc2600_wm97xx_init);
module_exit(nxc2600_wm97xx_exit);

MODULE_DESCRIPTION("NXC2600 WM97XX ts driver");
MODULE_AUTHOR("Hardy Weng <hardy.weng@icnexus.com.tw>");
MODULE_LICENSE("GPL");
