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

#ifndef _NXC2600_GPIO_H_
#define _NXC2600_GPIO_H_

#define GPIO_PIN_MASK			(0xff000000)

#define GPIO_MODE_NORMAL		(0x00000000)
#define GPIO_MODE_IRQ_LOW_LEVEL		(0x00010000)
#define GPIO_MODE_IRQ_HIGH_LEVEL	(0x00020000)
#define GPIO_MODE_IRQ_FALLING		(0x00030000)
#define GPIO_MODE_IRQ_RISING		(0x00040000)
#define GPIO_MODE_INVERTED		(0x00800000)

#define GPIO_DIR_IN			(0x00400000)
#define GPIO_DIR_OUT			(0x00000000)

#define GPIO_FUNC_NORMAL		(0x00000000)
#define GPIO_FUNC_CIM			(0x00000001)
#define GPIO_FUNC_DMA			(0x00000002)
#define GPIO_FUNC_UART3			(0x00000003)
#define GPIO_FUNC_UART3_CTSRTS		(0x00000004)
#define GPIO_FUNC_UART1			(0x00000005)
#define GPIO_FUNC_OHCI_HCD		(0x00000006)
#define GPIO_FUNC_PS2			(0x00000007)
#define GPIO_FUNC_MSC			(0x00000008)
#define GPIO_FUNC_PRT			(0x00000009)
#define GPIO_FUNC_LCD			(0x0000000a)
#define GPIO_FUNC_SCC0			(0x0000000b)
#define GPIO_FUNC_SCC1			(0x0000000c)
#define GPIO_FUNC_AC97			(0x0000000d)
#define GPIO_FUNC_SSI			(0x0000000e)
#define GPIO_FUNC_NAND			(0x0000000f)
#define GPIO_FUNC_DCS1			(0x00000010)
#define GPIO_FUNC_CS1			(0x00000020)
#define GPIO_FUNC_CS2			(0x00000030)
#define GPIO_FUNC_CS3			(0x00000040)
#define GPIO_FUNC_CS4			(0x00000050)
#define GPIO_FUNC_CS5			(0x00000060)
#define GPIO_FUNC_PCMCIA		(0x00000070)
#define GPIO_FUNC_PWM0			(0x00000080)
#define GPIO_FUNC_PWM1			(0x00000090)
#define GPIO_FUNC_UART2			(0x000000a0)
#define GPIO_FUNC_MAC			(0x000000b0)
#define GPIO_FUNC_UART0			(0x000000c0)


#define MAX_GPIOS	(128)

struct device;
typedef struct _GPIO GPIO;

extern GPIO* 
gpio_request( uint32_t pin,
	      uint32_t operation,
	      struct device *dev );

extern void 
gpio_release( GPIO *gpio );

/* set output, auto change direction if not ouput */
extern int 
gpio_set_out( GPIO *gpio, 
	      int hilow );

extern int
gpio_set_out_multi( GPIO **gpio,
	            int *hilow,
	            uint32_t gpios);

/* set input, auto change direction if not input */
extern int 
gpio_get_in( GPIO *gpio );

extern int
gpio_get_in_multi( GPIO **gpio,
		   int *hilow,
		   uint32_t gpios );

#endif

