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
 * 2007/11/30: 
 *
 *	1. fix uart3 cts/rts.
 *
 * 2007/07/16: initial
 *
 *
 * Author: Hardy Weng <hardy.weng@icnexus.com.tw>
 *
 */

#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/serial_core.h>
#include <linux/signal.h>
#include <linux/slab.h>
#include <linux/types.h>

#include <linux/serial_8250.h>
#include <asm/cacheflush.h>
#include <asm/delay.h>
#include <asm/mach-nxc2600/nxc2600.h>

#include "8250.h"

#if defined(CONFIG_NXC2600_UART0) || defined(CONFIG_NXC2600_UART1) || defined(CONFIG_NXC2600_UART2) || defined(CONFIG_NXC2600_UART3)

static struct plat_serial8250_port nxc2600_data[] = {
#ifdef CONFIG_NXC2600_UART0
	{
		.iobase		= CPHYSADDR(NXC2600_UART(0)),
		.membase	= CPHYSADDR(NULL),
		.mapbase	= CPHYSADDR(NXC2600_UART(0)),
		.flags		= UPF_IOREMAP | UPF_BOOT_AUTOCONF | UPF_SKIP_TEST | UPF_BUGGY_UART,
		.irq		= NXC2600_IRQ_UART0,
		.uartclk	= CONFIG_NXC2600_EXTAL_CLOCK,
		.regshift	= 2,
		.iotype		= UPIO_MEM,
	},
#endif
#ifdef CONFIG_NXC2600_UART1
	{
                .iobase         = CPHYSADDR(NXC2600_UART(1)),
                .membase        = CPHYSADDR(NULL),
		.mapbase	= CPHYSADDR(NXC2600_UART(1)),
                .flags          = UPF_IOREMAP | UPF_BOOT_AUTOCONF | UPF_SKIP_TEST | UPF_BUGGY_UART,
                .irq            = NXC2600_IRQ_UART1,
                .uartclk        = CONFIG_NXC2600_EXTAL_CLOCK,
                .regshift       = 2,
                .iotype         = UPIO_MEM,
        },
#endif
#ifdef CONFIG_NXC2600_UART2
	{
                .iobase         = CPHYSADDR(NXC2600_UART(2)),
                .membase        = CPHYSADDR(NULL),
		.mapbase	= CPHYSADDR(NXC2600_UART(2)),
                .flags          = UPF_IOREMAP | UPF_BOOT_AUTOCONF | UPF_SKIP_TEST | UPF_BUGGY_UART,
                .irq            = NXC2600_IRQ_UART2,
                .uartclk        = CONFIG_NXC2600_EXTAL_CLOCK,
                .regshift       = 2,
                .iotype         = UPIO_MEM,
        },
#endif
#if defined(CONFIG_NXC2600_UART3)
	{
                .iobase         = CPHYSADDR(NXC2600_UART(3)),
                .membase        = CPHYSADDR(NULL),
		.mapbase	= CPHYSADDR(NXC2600_UART(3)),
                .flags          = UPF_IOREMAP | UPF_BOOT_AUTOCONF | UPF_SKIP_TEST | UPF_BUGGY_UART,
                .irq            = NXC2600_IRQ_UART3,
                .uartclk        = CONFIG_NXC2600_EXTAL_CLOCK,
                .regshift       = 2,
                .iotype         = UPIO_MEM,
        },
#endif
	{ },
};

static struct platform_device nxc2600_device = {
	.name			= "serial8250",
	.id			= PLAT8250_DEV_PLATFORM,
	.dev			= {
		.platform_data	= nxc2600_data,
	},
};

static int __init nxc2600_init(void)
{
#ifdef CONFIG_NXC2600_UART0
	nxc2600_gpio_vs_uart0(0);
#endif
#ifdef CONFIG_NXC2600_UART1
	nxc2600_gpio_vs_uart1(0);
#endif
#ifdef CONFIG_NXC2600_UART2
	nxc2600_gpio_vs_uart2(0);
#endif
#ifdef CONFIG_NXC2600_UART3
	nxc2600_gpio_vs_uart3(0);
#ifdef CONFIG_NXC2600_UART3_CTSRTS
	nxc2600_gpio_vs_uart3_ctsrts(0);
	REG8(NXC2600_UART_MCR(3)) |= NXC2600_UART_MCR_ENABLE_MODEM;
#endif
#endif
	return platform_device_register(&nxc2600_device);
}

static void __exit nxc2600_exit(void)
{
	platform_device_unregister(&nxc2600_device);

#ifdef CONFIG_NXC2600_UART0
        nxc2600_gpio_vs_uart0(1);
#endif
#ifdef CONFIG_NXC2600_UART1
        nxc2600_gpio_vs_uart1(1);
#endif
#ifdef CONFIG_NXC2600_UART2
        nxc2600_gpio_vs_uart2(1);
#endif
#ifdef CONFIG_NXC2600_UART3
        nxc2600_gpio_vs_uart3(1);
#ifdef CONFIG_NXC2600_UART3_CTSRTS
        nxc2600_gpio_vs_uart3_ctsrts(1);
#endif
#endif

}

module_init(nxc2600_init);
module_exit(nxc2600_exit);

MODULE_AUTHOR("Hardy Weng <hardy.weng@icnexus.com.tw>");
MODULE_DESCRIPTION("NXC2600 serial driver");
MODULE_LICENSE("GPL");

#endif
