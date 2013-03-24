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
#include <asm/delay.h>
#include <asm/bootinfo.h>
#include <linux/tty.h>
#include <asm/elf.h>
#include <linux/root_dev.h>
#include <linux/initrd.h>
#include <asm/cacheflush.h>
#include <asm/addrspace.h>
#include <asm/mach-nxc2600/nxc2600.h>


void
nxc2600_aic_set_dma_mode_tx_packet(struct nxc2600_dma_mode *dma_mode)
{
        switch(REG32(NXC2600_AIC_CR2)&NXC2600_AIC_CR2_OASS_MASK)
        {

                case NXC2600_AIC_CR2_OASS_8_BIT:
                        dma_mode->mode |= NXC2600_DMA_DCS_TSZ_8_BIT |
                                          NXC2600_DMA_DCS_DP_8_BIT |
                                          NXC2600_DMA_DCS_SP_8_BIT;

                        break;
                case NXC2600_AIC_CR2_OASS_16_BIT:
                        dma_mode->mode |= NXC2600_DMA_DCS_TSZ_16_BIT |
                                          NXC2600_DMA_DCS_DP_16_BIT |
                                          NXC2600_DMA_DCS_SP_16_BIT;
                        break;

                case NXC2600_AIC_CR2_OASS_18_BIT:
                case NXC2600_AIC_CR2_OASS_20_BIT:
                        dma_mode->mode |= NXC2600_DMA_DCS_TSZ_32_BIT |
                                          NXC2600_DMA_DCS_DP_32_BIT |
                                          NXC2600_DMA_DCS_SP_32_BIT;
                        break;
        }
}
void
nxc2600_aic_set_dma_mode_rx_packet(struct nxc2600_dma_mode *dma_mode)
{
        switch(REG32(NXC2600_AIC_CR2)&NXC2600_AIC_CR2_IASS_MASK)
        {
                case NXC2600_AIC_CR2_IASS_8_BIT:
                        dma_mode->mode |= NXC2600_DMA_DCS_TSZ_8_BIT |
                                          NXC2600_DMA_DCS_DP_8_BIT |
                                          NXC2600_DMA_DCS_SP_8_BIT;

                        break;
                case NXC2600_AIC_CR2_IASS_16_BIT:
                        dma_mode->mode |= NXC2600_DMA_DCS_TSZ_16_BIT |
                                          NXC2600_DMA_DCS_DP_16_BIT |
                                          NXC2600_DMA_DCS_SP_16_BIT;
                        break;
                case NXC2600_AIC_CR2_IASS_18_BIT:
                case NXC2600_AIC_CR2_IASS_20_BIT:
                        dma_mode->mode |= NXC2600_DMA_DCS_TSZ_32_BIT |
                                          NXC2600_DMA_DCS_DP_32_BIT |
                                          NXC2600_DMA_DCS_SP_32_BIT;
                        break;
        }
}
EXPORT_SYMBOL(nxc2600_aic_set_dma_mode_tx_packet);
EXPORT_SYMBOL(nxc2600_aic_set_dma_mode_rx_packet);

