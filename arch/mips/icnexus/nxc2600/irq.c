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
 * 2008/06/30:
 *
 *      1. fix pending interrupt register typo.
 *
 * 2007/11/20:
 *    
 *	1. fix no interrupt.
 *
 * 2007/07/16: initial
 *
 *
 * Author: Hardy Weng <hardy.weng@icnexus.com.tw>
 *
 */


#include <linux/interrupt.h>
#include <linux/types.h>
#include <asm/delay.h>
#include <asm/cacheflush.h>
#include <asm/addrspace.h>
#include <asm/mach-nxc2600/nxc2600.h>


static void
mask_intc_irq(unsigned int irq)
{
	REG32(NXC2600_INTC_MSR) = (1<<irq);
};

static void
unmask_intc_irq( unsigned int irq )
{
	REG32(NXC2600_INTC_MCR) = (1<<irq);
}

static struct irq_chip nxc2600_intc_irq =
{
	.name		=	"NXC2600 INTC",
	.ack		=	unmask_intc_irq,
	.mask		=	mask_intc_irq,
	.mask_ack	=	unmask_intc_irq,
	.unmask		=	unmask_intc_irq,
};

static void
mask_gpio_irq( unsigned int irq )
{
	unsigned int gp;

	irq -= NXC2600_GPIO_BASE_IRQ(0);
	gp = irq/32;
	irq %= 32;
	REG32(NXC2600_GPIO_IM(gp)) |= (1<<irq);
	REG32(NXC2600_GPIO_IE(gp)) &= ~(1<<irq);
}

static void
unmask_gpio_irq( unsigned int irq )
{
	unsigned int gp;

	irq -= NXC2600_GPIO_BASE_IRQ(0);
	gp = irq/32;
	irq %= 32;
	REG32(NXC2600_GPIO_IM(gp)) &= ~(1<<irq);
	REG32(NXC2600_GPIO_IE(gp)) |= (1<<irq);
}

static int
set_type_gpio_irq(unsigned int irq, 
		  unsigned int flow_type)
{
	uint32_t trigger = 0;

	switch(flow_type)
	{
		case IRQ_TYPE_EDGE_RISING:
			trigger = NXC2600_GPIO_IRQ_TRIGGER_RISING_EDGE;
			break;
		case IRQ_TYPE_EDGE_FALLING:
			trigger = NXC2600_GPIO_IRQ_TRIGGER_FALLING_EDGE;
			break;
		case IRQ_TYPE_LEVEL_HIGH:
			trigger = NXC2600_GPIO_IRQ_TRIGGER_HIGH_LEVEL;
			break;
		case IRQ_TYPE_LEVEL_LOW:
			trigger = NXC2600_GPIO_IRQ_TRIGGER_LOW_LEVEL;
			break;
		default:
			return -1;
	}

	nxc2600_set_gpio_input_mode(irq-NXC2600_GPIO_BASE_IRQ(0), 0, trigger);
	return 0;
}

static struct irq_chip nxc2600_gpio_irq =
{
	.name		=	"NXC2600 GPIO",
	.ack		=	unmask_gpio_irq,
	.mask		=	mask_gpio_irq,
	.mask_ack	=	unmask_gpio_irq,
	.unmask		=	unmask_gpio_irq,
	.set_type	=	set_type_gpio_irq,
};



static void
mask_dma_irq( unsigned int irq )
{
	irq -= NXC2600_DMA_BASE_IRQ;
	REG32(NXC2600_DMA_DCS(irq)) &= ~NXC2600_DMA_DCS_TIE;
}

static void
unmask_dma_irq( unsigned int irq )
{
	irq -= NXC2600_DMA_BASE_IRQ;
	REG32(NXC2600_DMA_IP) &= ~NXC2600_DMA_IP_CIRQ(irq);
	REG32(NXC2600_DMA_DCS(irq)) |= NXC2600_DMA_DCS_TIE;
}

static struct irq_chip nxc2600_dma_irq =
{
	.name		=	"NXC2600 DMA",
	.ack		=	unmask_dma_irq,
	.mask		=	mask_dma_irq,
	.mask_ack	=	unmask_dma_irq,
	.unmask		=	unmask_dma_irq,
};


static int dma_get_pending_irq( void )
{
	u32 pending;
	int irq;

	pending = REG32(NXC2600_DMA_IP);

	for( irq = 0; irq < 8; irq++ )
	{
		if(pending & NXC2600_DMA_IP_CIRQ(irq)) 
		{
			mask_dma_irq(NXC2600_DMA_BASE_IRQ+irq);
			return (NXC2600_DMA_BASE_IRQ+irq);
		}
	}

	return -1;
}

static int gpio_get_pending_irq( int gp )
{
	u32 pending;
	int gpio; 

	pending = REG32(NXC2600_GPIO_IRQ(gp));

	for(gpio = 0; gpio < 32; gpio++ )
	{
		if( pending & (1<<gpio) ) 
		{
			mask_gpio_irq(NXC2600_GPIO_BASE_IRQ(gp)+gpio);
			return (NXC2600_GPIO_BASE_IRQ(gp)+gpio);
		}
	}

	return -1;
}

asmlinkage void plat_irq_dispatch(void)
{
	u32 pending;
	int irq = -1;

	pending = REG32(NXC2600_INTC_PR);

	if( pending & NXC2600_INTC_PR_OST0 )
		irq = NXC2600_IRQ_OST0;
	else if( pending & NXC2600_INTC_PR_DMAC )
		irq = dma_get_pending_irq();
	else if( pending & NXC2600_INTC_PR_GPIO0 )
		irq = gpio_get_pending_irq(0);
	else if( pending & NXC2600_INTC_PR_GPIO1 )
		irq = gpio_get_pending_irq(1);
	else if( pending & NXC2600_INTC_PR_GPIO2 )
		irq = gpio_get_pending_irq(2);
	else if( pending & NXC2600_INTC_PR_GPIO3 )
		irq = gpio_get_pending_irq(3);
	else
	{
		pending &= ~(NXC2600_INTC_PR_DMAC|NXC2600_INTC_PR_GPIO0|NXC2600_INTC_PR_GPIO1|NXC2600_INTC_PR_GPIO2|NXC2600_INTC_PR_GPIO3);
		if( pending )
		{
			for(irq = 31; irq > 0; irq--)
			{
				if( pending & (1<<irq) )
				{
					mask_intc_irq(irq);
					break;
				}
			}
		}
	}

	if( irq >= 0 )
		do_IRQ(irq);
}

void __init arch_init_irq(void)
{
	int irq;

        clear_c0_status(0xff04);
        set_c0_status(0x0400);

	/* Disable all interrupts */
	REG32(NXC2600_INTC_MSR) = 0xffffffff;

	/* Disable gpio interrupts */
	REG32(NXC2600_GPIO_IE(0)) = 0;
	REG32(NXC2600_GPIO_IM(0)) = 0;
	REG32(NXC2600_GPIO_IE(1)) = 0;
        REG32(NXC2600_GPIO_IM(1)) = 0;
	REG32(NXC2600_GPIO_IE(2)) = 0;
        REG32(NXC2600_GPIO_IM(2)) = 0;
	REG32(NXC2600_GPIO_IE(3)) = 0;
        REG32(NXC2600_GPIO_IM(3)) = 0;
	/* Enable gpio group interrupts in INTC */
	REG32(NXC2600_INTC_MCR) = NXC2600_INTC_PR_GPIO0|NXC2600_INTC_PR_GPIO1|NXC2600_INTC_PR_GPIO2|NXC2600_INTC_PR_GPIO3;

	/* Disable dma interrupts */
	REG32(NXC2600_DMA_DCS(0)) &= ~NXC2600_DMA_DCS_TIE;
	REG32(NXC2600_DMA_DCS(1)) &= ~NXC2600_DMA_DCS_TIE;
	REG32(NXC2600_DMA_DCS(2)) &= ~NXC2600_DMA_DCS_TIE;
	REG32(NXC2600_DMA_DCS(3)) &= ~NXC2600_DMA_DCS_TIE;
	REG32(NXC2600_DMA_DCS(4)) &= ~NXC2600_DMA_DCS_TIE;
	REG32(NXC2600_DMA_DCS(5)) &= ~NXC2600_DMA_DCS_TIE;
	REG32(NXC2600_DMA_DCS(6)) &= ~NXC2600_DMA_DCS_TIE;
	REG32(NXC2600_DMA_DCS(7)) &= ~NXC2600_DMA_DCS_TIE;
	/* Enable dma interrupt in INTC */
	REG32(NXC2600_INTC_MCR) = NXC2600_INTC_PR_DMAC;


	for(irq = 1; irq < 32; irq++)
	{
                set_irq_chip(irq, &nxc2600_intc_irq);
        }
	for(; irq < (32+32*4); irq++)
	{
		set_irq_chip(irq, &nxc2600_gpio_irq);
	}
	for(; irq < (32+32*4+8); irq++)
	{
		set_irq_chip(irq, &nxc2600_dma_irq);
	}

}

