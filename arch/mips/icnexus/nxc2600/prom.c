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
 * Hardy Weng <hardy.weng@icnexus.com.tw>
 *
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/blkdev.h>
#include <linux/bootmem.h>
#include <linux/smp.h>
#include <linux/initrd.h>
#include <linux/pm.h>

#include <asm/bootinfo.h>
#include <asm/reboot.h>

#include <asm/mach-nxc2600/nxc2600.h>

static int prom_argc;
static char **prom_argv,**prom_envp;

void  __init prom_init_cmdline(void);
void __init prom_init(void)
{

	prom_argc = (int)fw_arg0;
	prom_argv = (char **)fw_arg1;
	prom_envp = (char **)fw_arg2;
	prom_init_cmdline();

}

void __init prom_free_prom_memory(void)
{
}

void prom_putchar(char c)
{
	uint32_t timeout = 10000;

        if (c == '\n') prom_putchar ('\r');

        /* Wait for fifo to shift out some bytes */
        while ( !(REG8(NXC2600_UART_LSR(CONFIG_NXC2600_DEFAULT_DEBUG_PORT))&NXC2600_UART_LSR_THRE) ) {if(timeout--) udelay(1); else break;}

	REG8(NXC2600_UART_THR(CONFIG_NXC2600_DEFAULT_DEBUG_PORT)) = (u8)c;

}

#define prom_argv(index) ((char *)(long)prom_argv[(index)])

char * __init prom_getcmdline(void)
{
	return &(arcs_cmdline[0]);
}


void  __init prom_init_cmdline(void)
{
	char *cp;
	int actr;

	actr = 1;

	cp = &(arcs_cmdline[0]);
	while(actr < prom_argc) 
	{
	        strcpy(cp, prom_argv(actr));
		cp += strlen(prom_argv(actr));
		*cp++ = ' ';
		actr++;
	}
	if (cp != &(arcs_cmdline[0])) 
	{
		/* get rid of trailing space */
		--cp;
		*cp = '\0';
	}
}

