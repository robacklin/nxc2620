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
 * 2007/10/16: initial
 *
 *
 * Author: Hardy Weng <hardy.weng@icnexus.com.tw>
 *
 */


#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/dmaengine.h>
#include <linux/init.h>
#include <linux/ide.h>
#include <linux/sysdev.h>
#include <linux/dma-mapping.h>

#include <asm/io.h>
#include <asm/mach-nxc2600/nxc2600.h>

#ifndef CONFIG_NXC2600_IDE_IRQ_PIN
#define CONFIG_NXC2600_IDE_IRQ_PIN	27
#endif
#define NXC2600_IDE_IRQ	(NXC2600_GPIO_NIRQ(CONFIG_NXC2600_IDE_IRQ_PIN))
#ifndef CONFIG_NXC2600_IDE_RESET_PIN
#define CONFIG_NXC2600_IDE_RESET_PIN	88
#endif
#ifndef CONFIG_NXC2600_IDE_CS
#define CONFIG_NXC2600_IDE_CS	5
#endif
#ifndef CONFIG_NXC2600_IDE_CS_ADDR
#define	CONFIG_NXC2600_IDE_CS_ADDR	0x09000000
#endif

static char drv_name[] = "nxc2600-ide";


static struct _nxc2600_ide_hwif 
{

        ide_hwif_t              *hwif;
        struct device           *dev;
        int                     irq;
        uint32_t                regbase;
} nxc2600_ide_hwif;

static void 
nxc2600_ide_setup_ports(hw_regs_t *hw, 
			struct _nxc2600_ide_hwif *hwif)
{
	int i;
	unsigned long *ata_regs = hw->io_ports;

	for (i = IDE_DATA_OFFSET; i < IDE_CONTROL_OFFSET; i++) 
	{
		*ata_regs++ = hwif->regbase + (i<<1);
	}
	hw->io_ports[IDE_CONTROL_OFFSET] = hwif->regbase + (6<<1);

}

static int 
nxc2600_ide_probe(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	ide_hwif_t *hwif;
	struct resource *res;
	hw_regs_t *hw;
	int i;
	int ret = 0;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

        if (res == NULL)
        {
                pr_debug("%s %d: no base address\n", drv_name, pdev->id);
                ret = -ENODEV;
                goto out;
        }

	nxc2600_set_gpio_output_mode(CONFIG_NXC2600_IDE_RESET_PIN,0);
#if CONFIG_NXC2600_IDE_CS == 1
	nxc2600_gpio_vs_cs1(0);
#elif CONFIG_NXC2600_IDE_CS == 2
	nxc2600_gpio_vs_cs2(0);
#elif CONFIG_NXC2600_IDE_CS == 3
	nxc2600_gpio_vs_cs3(0);
#elif CONFIG_NXC2600_IDE_CS == 4
	nxc2600_gpio_vs_cs4(0);
#elif CONFIG_NXC2600_IDE_CS == 5
	nxc2600_gpio_vs_cs5(0);
#else
#error wrong ide cs
#endif
	REG32(NXC2600_SM_ACR0+CONFIG_NXC2600_IDE_CS*4) = 0x000000ff|((res->start&0xff000000)>>16);
        REG32(NXC2600_SM_CR0+CONFIG_NXC2600_IDE_CS*4) = 
				NXC2600_SM_CR_TYPE_NORMAL|
                                NXC2600_SM_CR_BL_CA_8|
                                NXC2600_SM_CR_BCM_NORMAL|
                                NXC2600_SM_CR_BW_BITS_16|
                                NXC2600_SM_CR_TAS(7)|
                                NXC2600_SM_CR_TAH(7)|
                                NXC2600_SM_CR_TBP_31|
                                NXC2600_SM_CR_TAW_31|
                                NXC2600_SM_CR_STRV(0xf);

	nxc2600_set_gpio_output(CONFIG_NXC2600_IDE_RESET_PIN,1);
        udelay(30);
	nxc2600_set_gpio_output(CONFIG_NXC2600_IDE_RESET_PIN,0);
	udelay(30);

	nxc2600_set_gpio_input_mode(CONFIG_NXC2600_IDE_IRQ_PIN,0,NXC2600_GPIO_IRQ_TRIGGER_RISING_EDGE);

	memset(&nxc2600_ide_hwif, 0, sizeof(nxc2600_ide_hwif));
	nxc2600_ide_hwif.dev                  = 0;

	nxc2600_ide_hwif.dev = dev;
	nxc2600_ide_hwif.irq = platform_get_irq(pdev, 0);

	if (nxc2600_ide_hwif.irq < 0) 
	{
		pr_debug("%s %d: no IRQ\n", drv_name, pdev->id);
		ret = -ENODEV;
		goto out;
	}

	if (!request_mem_region (res->start, res->end-res->start, pdev->name)) 
	{
		pr_debug("%s: request_mem_region failed\n", drv_name);
		ret =  -EBUSY;
		goto out;
	}

	nxc2600_ide_hwif.regbase = (uint32_t)ioremap(res->start, res->end-res->start);
	if (nxc2600_ide_hwif.regbase == 0) 
	{
		ret = -ENOMEM;
		goto out;
	}

        for (i = 0; i < MAX_HWIFS; i++)
                if (!ide_hwifs[i].io_ports[IDE_DATA_OFFSET])
                        break;

        if (i >= MAX_HWIFS)
        {
                printk(KERN_ERR "nxc2600 ide: no free slot for interface\n");
		return -ENOMEM;
        }

        hwif 				= ide_hwifs + i;
	default_hwif_mmiops(hwif);
	hw 				= &hwif->hw;
	hwif->irq = hw->irq             = nxc2600_ide_hwif.irq;

	nxc2600_ide_setup_ports(hw, &nxc2600_ide_hwif);
	memcpy(hwif->io_ports, hw->io_ports, sizeof(hwif->io_ports));

	hwif->noprobe = 0;
	hwif->hold                      = 1;
	hwif->mmio  			= 1;

	hwif->autodma                   = 0;
	hwif->channel                   = 0;
	hwif->select_data               = 0;
	hwif->config_data               = 0;

	hwif->drives[0].autodma         = 0;
	hwif->drives[0].autotune        = 0;

	hwif->drives[0].no_io_32bit     = 1;   

	nxc2600_ide_hwif.hwif                 = hwif;
	hwif->hwif_data                 = &nxc2600_ide_hwif;

	probe_hwif_init(hwif);

	ide_proc_register_port(hwif);

	dev_set_drvdata(dev, hwif);

 out:
	return ret;
}

static int 
nxc2600_ide_remove(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct resource *res;
	ide_hwif_t *hwif = dev_get_drvdata(dev);

	ide_unregister(hwif - ide_hwifs);

	iounmap((void *)nxc2600_ide_hwif.regbase);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, res->end - res->start);

	return 0;
}

static struct device_driver nxc2600_ide_driver = 
{
	.name		= drv_name,
	.bus		= &platform_bus_type,
	.probe 		= nxc2600_ide_probe,
	.remove		= nxc2600_ide_remove,
};

static struct resource nxc2600_ide_resource[] =
{
	{
        .name = drv_name,
        .start = CONFIG_NXC2600_IDE_CS_ADDR,
        .end   = CONFIG_NXC2600_IDE_CS_ADDR + 0x00100000 - 1,
        .flags = IORESOURCE_MEM
	},
	{
	.name	= drv_name,
	.start = NXC2600_IDE_IRQ,
	.end   = NXC2600_IDE_IRQ,
	.flags = IORESOURCE_IRQ
	}
};

static struct platform_device nxc2600_ide_device =
{
        .name  = drv_name,
        .id    = 0,
        .num_resources = 2,
        .resource = nxc2600_ide_resource
};

static int __init 
nxc2600_ide_init(void)
{
	int ret = driver_register(&nxc2600_ide_driver);
	if(ret)
		return ret;
	return platform_device_register(&nxc2600_ide_device);
}

static void __exit 
nxc2600_ide_exit(void)
{
	platform_device_unregister(&nxc2600_ide_device);
	driver_unregister(&nxc2600_ide_driver);
}

module_init(nxc2600_ide_init);
module_exit(nxc2600_ide_exit);

MODULE_AUTHOR("Hardy Weng <hardy.weng@icnexus.com.tw>");
MODULE_DESCRIPTION("NXC2600 Gen Bus IDE Driver");
MODULE_LICENSE("GPL");

