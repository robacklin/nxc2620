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


#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/irq.h>
#include <linux/param.h>
#include <asm/bootinfo.h>
#include <asm/delay.h>
#include <asm/cacheflush.h>
#include <asm/mach-nxc2600/nxc2600.h>

extern void (*mips_timer_ack)(void);

static void
nxc2600_tick_timer(void)
{
	REG16(NXC2600_OST_CR(0)) &= ~NXC2600_OST_CR_UF;
}

void __init
plat_timer_setup(struct irqaction *irq)
{
	setup_irq(NXC2600_IRQ_OST0, irq);
	nxc2600_start_system_timer( HZ, 0 );
}

void 
nxc2600_tick_timer_init(void)
{
	mips_timer_ack = nxc2600_tick_timer;
}
