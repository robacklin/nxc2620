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


#include <linux/types.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sysdev.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/serial_8250.h>
#include <linux/fb.h>
#include <asm/delay.h>
#include <asm/bootinfo.h>
#include <linux/tty.h>
#include <asm/elf.h>
#include <linux/root_dev.h>
#include <linux/initrd.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>

#include <asm/cacheflush.h>
#include <asm/addrspace.h>
#include <asm/mach-nxc2600/nxc2600.h>


extern void nxc2600_tick_timer_init(void);
extern void (*board_time_init)(void);

#ifdef CONFIG_SPI_NXC2600
#ifdef CONFIG_MTD_M25P80
static struct mtd_partition spi_flash_partitions[] = {
        {
                .name = "params",
                .size = 0x00010000,
                .offset = 0,
                .mask_flags = MTD_CAP_ROM
        }, 
};

static struct flash_platform_data spi_flash_data = {
        .name = "m25p80",
        .parts = spi_flash_partitions,
        .nr_parts = ARRAY_SIZE(spi_flash_partitions),
        .type = "m25p05",
};


static struct spi_board_info spi_board_infos[] __initdata = 
{
        {
                /* the modalias must be the same as spi device driver name */
                .modalias = "m25p80", /* Name of spi_driver for this device */
                .max_speed_hz = 25000000,     /* max spi clock (SCK) speed in HZ */
                .bus_num = 0, /* Framework bus number */
                .chip_select = 0, /* Framework chip select. On STAMP537 it is SPISSEL1*/
                .platform_data = &spi_flash_data,
                .controller_data = NULL,
                .mode = SPI_MODE_3,
        },
};
#endif
#endif
#ifdef CONFIG_MFD_SM501
#include <linux/sm501.h>
#include <linux/sm501-regs.h>
#define CONFIG_NXC2600_SM501_ADDR	(0x18000000)
#define	CONFIG_NXC2600_SM501_MEMSIZE	(0x4000000)
#define CONFIG_NXC2600_SM501_IRQ_PIN	(89)
#define CONFIG_NXC2600_SM501_IRQ	(NXC2600_GPIO_NIRQ(CONFIG_NXC2600_SM501_IRQ_PIN))
#define NXC2600_SM501_MMIO_ADDR_OFFSET (0x4000000-0x200000)

static struct resource nxc2600_sm501_resource[] =
{
        [0] = {
                .start  = CPHYSADDR(CONFIG_NXC2600_SM501_ADDR),
                .end    = CPHYSADDR(CONFIG_NXC2600_SM501_ADDR + CONFIG_NXC2600_SM501_MEMSIZE) -1,
                .flags  = IORESOURCE_MEM,
        },
        [1] = {
                .start  = CPHYSADDR(CONFIG_NXC2600_SM501_ADDR + NXC2600_SM501_MMIO_ADDR_OFFSET),
                .end    = CPHYSADDR(CONFIG_NXC2600_SM501_ADDR + NXC2600_SM501_MMIO_ADDR_OFFSET + 0x200000  )-1,
                .flags  = IORESOURCE_MEM,
        },
        [2] = {
                .start  = CONFIG_NXC2600_SM501_IRQ,
                .end    = CONFIG_NXC2600_SM501_IRQ,
                .flags  = IORESOURCE_IRQ,
        },
};

static struct fb_videomode nxc2600_sm501_def_mode_crt =
{
 	NULL, 60, 1024, 768, 15384, 168, 8, 29, 3, 144, 6,
        0, FB_VMODE_NONINTERLACED
};

static struct fb_videomode nxc2600_sm501_def_mode_pnl =
{
#if defined(CONFIG_SM50X_PANEL_TFP507MWVGAHBE_03)
	NULL, 60, 640, 480, 39719, 48, 10, 33, 10, 96, 2,
#elif defined(CONFIG_SM50X_PANEL_DATAIMAGE_CLAA070LC0ACW)
	NULL, 60, 800, 480, 39719, 48, 10, 33, 10, 96, 2,
#endif
        0, FB_VMODE_NONINTERLACED
};

static struct sm501_platdata_fbsub nxc2600_sm501fb_pdata_crt = 
{
	.def_bpp	= 32,
	.max_mem	= 0x2000000,
	.def_mode	= &nxc2600_sm501_def_mode_crt,
        .flags          = (
			   SM501FB_FLAG_USE_INIT_MODE|
                           SM501FB_FLAG_USE_HWCURSOR|
                           SM501FB_FLAG_USE_HWACCEL|
                           SM501FB_FLAG_DISABLE_AT_EXIT),

};

static struct sm501_platdata_fbsub nxc2600_sm501fb_pdata_pnl = 
{
	.def_bpp	= 16,
	.max_mem	= 0x2000000,
	.def_mode	= &nxc2600_sm501_def_mode_pnl,
        .flags          = (
			   SM501FB_FLAG_USE_INIT_MODE|
			   SM501FB_FLAG_USE_HWCURSOR|
			   SM501FB_FLAG_USE_HWACCEL|
                           SM501FB_FLAG_DISABLE_AT_EXIT),

};

static struct sm501_platdata_fb nxc2600_sm501fb_def_pdata = 
{
        .fb_route               = SM501_FB_CRT_PANEL, //SM501_FB_OWN, marvin+
        .fb_crt                 = &nxc2600_sm501fb_pdata_crt,
        .fb_pnl                 = &nxc2600_sm501fb_pdata_pnl,
	.flags			= SM501_FBPD_SWAP_FB_ENDIAN,
};


static struct sm501_initdata nxc2600_sm501_initdata = 
{
	.devices	= SM501_USE_FBACCEL,
	.mclk		= 72000000,
	.m1xclk		= 144000000,
};

static struct sm501_platdata nxc2600_sm501_platdata =
{
	.init		= &nxc2600_sm501_initdata,
	.fb		= &nxc2600_sm501fb_def_pdata,
};

static struct platform_device nxc2600_device_sm501 = 
{
        .name           = "sm501",
        .id             = 0,
        .num_resources  = ARRAY_SIZE(nxc2600_sm501_resource),
        .resource       = nxc2600_sm501_resource,
        .dev            = {
                .platform_data = &nxc2600_sm501_platdata,
        },
};
#endif
#ifdef CONFIG_USB_GADGET_NXC2600
static struct platform_device nxc2600_device_udc =
{
	.name		= "nxc2600_udc",
	.id		= 0,
	.num_resources	= 0,
	.resource	= NULL,
	
};
#endif
#ifdef CONFIG_BACKLIGHT_NXC2600
static struct platform_device nxc2600_device_backlight =
{
        .name           = "nxc2600-bl",
        .id             = 0,
        .num_resources  = 0,
        .resource       = NULL,

};
#endif
#ifdef CONFIG_NXC2600_SIR0
static struct resource nxc2600_irda0_resource =
{
        .start  = NXC2600_IRQ_UART0,
        .end    = NXC2600_IRQ_UART0,
        .flags  = IORESOURCE_IRQ,
};

static struct platform_device nxc2600_device_irda0 =
{
        .name           = "irda0",
        .id             = 0,
        .num_resources  = ARRAY_SIZE(nxc2600_irda0_resource),
        .resource       = nxc2600_irda0_resource,
};
#endif

#ifdef CONFIG_NXC2600_SIR1
static struct resource nxc2600_irda1_resource =
{
        .start  = NXC2600_IRQ_UART1,
        .end    = NXC2600_IRQ_UART1,
        .flags  = IORESOURCE_IRQ,
};

static struct platform_device nxc2600_device_irda1 =
{
        .name           = "irda1",
        .id             = 0,
        .num_resources  = ARRAY_SIZE(nxc2600_irda1_resource),
        .resource       = nxc2600_irda1_resource,
};
#endif
#ifdef CONFIG_NXC2600_SIR2
static struct resource nxc2600_irda2_resource =
{
        .start  = NXC2600_IRQ_UART2,
        .end    = NXC2600_IRQ_UART2,
        .flags  = IORESOURCE_IRQ,
};

static struct platform_device nxc2600_device_irda2 =
{
        .name           = "irda2",
        .id             = 0,
        .num_resources  = ARRAY_SIZE(nxc2600_irda2_resource),
        .resource       = nxc2600_irda2_resource,
};
#endif
#ifdef CONFIG_NXC2600_SIR3
static struct resource nxc2600_irda3_resource =
{
        .start  = NXC2600_IRQ_UART3,
        .end    = NXC2600_IRQ_UART3,
        .flags  = IORESOURCE_IRQ,
};

static struct platform_device nxc2600_device_irda3 =
{
        .name           = "irda3",
        .id             = 0,
        .num_resources  = ARRAY_SIZE(nxc2600_irda3_resource),
        .resource       = nxc2600_irda3_resource,
};
#endif

static struct platform_device *nxc2600_devices[] __initdata = 
{
#ifdef CONFIG_MFD_SM501
	&nxc2600_device_sm501,
#endif
#ifdef CONFIG_USB_GADGET_NXC2600
	&nxc2600_device_udc,
#endif
#ifdef CONFIG_BACKLIGHT_NXC2600
	&nxc2600_device_backlight,
#endif
#ifdef CONFIG_NXC2600_SIR0
	&nxc2600_device_irda0,
#endif
#ifdef CONFIG_NXC2600_SIR1
        &nxc2600_device_irda1,
#endif
#ifdef CONFIG_NXC2600_SIR2
        &nxc2600_device_irda2,
#endif
#ifdef CONFIG_NXC2600_SIR3
        &nxc2600_device_irda3,
#endif

};


static int __init nxc2600_platform_init(void)
{
#ifdef CONFIG_MFD_SM501
        nxc2600_gpio_vs_cs1(0);
	 REG32(NXC2600_SM_CR1) = NXC2600_SM_CR_TYPE_NORMAL|
                                NXC2600_SM_CR_BL_CA_4|
                                NXC2600_SM_CR_BCM_NORMAL|
                                NXC2600_SM_CR_BW_BITS_32|
                                NXC2600_SM_CR_TAS(3)|
                                NXC2600_SM_CR_TAH(3)|
                                NXC2600_SM_CR_TBP_7|
                                NXC2600_SM_CR_TAW_7|
                                NXC2600_SM_CR_STRV(3);


        REG32(NXC2600_SM_ACR1) = 0x000018fc;

#ifdef CONFIG_FB_SM501
        nxc2600_gpio_vs_lcd(1);
#endif
#endif
#ifdef CONFIG_SPI_NXC2600
        spi_register_board_info(spi_board_infos,
                                ARRAY_SIZE(spi_board_infos));
#endif

        return platform_add_devices(nxc2600_devices, ARRAY_SIZE(nxc2600_devices));

}

arch_initcall(nxc2600_platform_init);



void __init board_setup(void)
{
	nxc2600_setup_idle_mode();
	REG32(NXC2600_SBA_CNTL) = 
#ifndef CONFIG_USB_GADGET_NXC2600
			NXC2600_SBA_CNTL_USB_PORT0_HOST|
#endif
			NXC2600_SBA_CNTL_WT_DMAC_CIM_ETHC_LCDC_UHC_CPU;
}

const char *get_system_type(void)
{
        return "NXC2600";
}
void __init plat_mem_setup(void)
{

        board_time_init = nxc2600_tick_timer_init;
        set_io_port_base(KSEG1);
	board_setup();
}


