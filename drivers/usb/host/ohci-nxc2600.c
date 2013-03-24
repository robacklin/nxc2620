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
 * 2007/07/16: initial
 *
 *
 * Author: Hardy Weng <hardy.weng@icnexus.com.tw>
 *
 */

#include <linux/device.h>
#include <linux/platform_device.h>
#include <asm/cacheflush.h>
#include <asm/mach-nxc2600/nxc2600.h>

static char ohci_hcd_nxc2600_name[] = "nxc2600_ohci_hcd";

static void start_hc(struct platform_device *dev)
{
	nxc2600_enable_clock(NXC2600_CDR_UHC);
}

static void stop_hc(struct platform_device *dev)
{
	nxc2600_disable_clock(NXC2600_CDR_UHC);
}


static int 
usb_hcd_probe (const struct hc_driver *driver,
	       struct platform_device *dev)
{
	int retval;
	struct usb_hcd *hcd;

	if (dev->resource[0].flags != IORESOURCE_MEM ||
			dev->resource[1].flags != IORESOURCE_IRQ) {
		dev_err (&dev->dev,"invalid resource type\n");
		return -ENOMEM;
	}

	hcd = usb_create_hcd (driver, &dev->dev, "nxc2600");
	if (!hcd)
		return -ENOMEM;
	hcd->rsrc_start = dev->resource[0].start;
	hcd->rsrc_len = dev->resource[0].end - dev->resource[0].start + 1;

	if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len, hcd_name)) {
		dev_err(&dev->dev, "request_mem_region [0x%08llx, 0x%08llx] "
				"failed\n", hcd->rsrc_start, hcd->rsrc_len);
		retval = -EBUSY;
		goto err1;
	}

	hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);
	if (!hcd->regs) {
		dev_err(&dev->dev, "ioremap [[0x%08llx, 0x%08llx] failed\n",
				hcd->rsrc_start, hcd->rsrc_len);
		retval = -ENOMEM;
		goto err2;
	}

	start_hc(dev);

	ohci_hcd_init(hcd_to_ohci(hcd));

	retval = usb_add_hcd(hcd, NXC2600_IRQ_UHC, IRQF_DISABLED);
	if (retval == 0)
		return retval;

	stop_hc(dev);
	iounmap(hcd->regs);
 err2:
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
 err1:
	usb_put_hcd(hcd);
	return retval;
}


static void 
usb_hcd_remove (struct usb_hcd *hcd, struct platform_device *dev)
{
	usb_remove_hcd(hcd);
	stop_hc(dev);
	iounmap(hcd->regs);
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	usb_put_hcd(hcd);
}

/*-------------------------------------------------------------------------*/

static int __devinit
ohci_nxc2600_start (struct usb_hcd *hcd)
{
	struct ohci_hcd	*ohci = hcd_to_ohci (hcd);
	int		ret;

	if ((ret = ohci_init(ohci)) < 0)
		return ret;

	if ((ret = ohci_run (ohci)) < 0) {
		err ("can't start %s", hcd->self.bus_name);
		ohci_stop (hcd);
		return ret;
	}

	return 0;
}

/*-------------------------------------------------------------------------*/

static const struct hc_driver ohci_hc_nxc2600_drv = 
{
	.description =		hcd_name,
	.product_desc =		"NXC2600 OHCI",
	.hcd_priv_size =	sizeof(struct ohci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq =			ohci_irq,
	.flags =		HCD_USB11 | HCD_MEMORY,

	/*
	 * basic lifecycle operations
	 */
	.start =		ohci_nxc2600_start,
	.stop =			ohci_stop,
	.shutdown =		ohci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue =		ohci_urb_enqueue,
	.urb_dequeue =		ohci_urb_dequeue,
	.endpoint_disable =	ohci_endpoint_disable,

	/*
	 * scheduling support
	 */
	.get_frame_number =	ohci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data =	ohci_hub_status_data,
	.hub_control =		ohci_hub_control,
	.hub_irq_enable =	ohci_rhsc_enable,
#ifdef	CONFIG_PM
	.bus_suspend =		ohci_bus_suspend,
	.bus_resume =		ohci_bus_resume,
#endif
	.start_port_reset =	ohci_start_port_reset,
};

/*-------------------------------------------------------------------------*/

static int 
ohci_hcd_nxc2600_drv_probe(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	int ret;
	device_init_wakeup(dev,1);
	ret = usb_hcd_probe(&ohci_hc_nxc2600_drv, pdev);
	return ret;
}

static int 
ohci_hcd_nxc2600_drv_remove(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct usb_hcd *hcd = dev_get_drvdata(dev);
	device_init_wakeup(dev,0);
	usb_hcd_remove(hcd, pdev);
	return 0;
}

static struct resource ohci_hcd_nxc2600_resource[] = 
{
{
	.name		= ohci_hcd_nxc2600_name,
	.start		= CPHYSADDR(NXC2600_OHCI_HCD),
	.end		= CPHYSADDR(NXC2600_OHCI_HCD) + 0x10000 -1,
	.flags		= IORESOURCE_MEM,
},
{
	.name           = ohci_hcd_nxc2600_name,
	.start          = NXC2600_IRQ_UHC,
	.end            = NXC2600_IRQ_UHC,
	.flags          = IORESOURCE_IRQ,
}
};
static u64 ohci_hcd_dmamask = ~0UL;
static struct platform_device ohci_hcd_nxc2600_dev =
{
	.name		= ohci_hcd_nxc2600_name,
	.id		= 0,
	.num_resources	= ARRAY_SIZE(ohci_hcd_nxc2600_resource), 
	.resource	= ohci_hcd_nxc2600_resource,
	.dev	=	{
		.dma_mask		=	&ohci_hcd_dmamask,
	},
};

static struct device_driver ohci_hcd_nxc2600_drv =
{
	.name		= ohci_hcd_nxc2600_name,
	.bus		= &platform_bus_type,
	.probe		= ohci_hcd_nxc2600_drv_probe,
	.remove		= ohci_hcd_nxc2600_drv_remove,
};

static int __init ohci_hcd_nxc2600_init (void)
{
	int err;
	err = driver_register(&ohci_hcd_nxc2600_drv);
	if(err) return err;
       	return platform_device_register(&ohci_hcd_nxc2600_dev);
}

static void __exit ohci_hcd_nxc2600_cleanup (void)
{
	platform_device_unregister(&ohci_hcd_nxc2600_dev);
	driver_unregister(&ohci_hcd_nxc2600_drv);
}

//module_init (ohci_hcd_nxc2600_init);
//module_exit (ohci_hcd_nxc2600_cleanup);

MODULE_DESCRIPTION("NXC2600 OHCI-USB driver");
MODULE_AUTHOR("Hardy Weng <hardy.weng@icnexus.com.tw>");
MODULE_LICENSE("GPL");



