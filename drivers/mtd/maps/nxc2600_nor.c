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
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>
#include <asm/cacheflush.h>
#include <asm/delay.h>
#include <asm/mach-nxc2600/nxc2600.h>

static struct mtd_partition nxc2600_partitions[] =
{
#if CONFIG_MTD_PHYSMAP_LEN > 0x400000
	{       .name   =       "ramdisk",
                .offset =       0,
                .size   =       4096*1024,
        },
#endif
        {       .name   =       "bootloader",
                .offset =       MTDPART_OFS_APPEND,
                .size   =       256*1024,
        },
        {       .name   =       "params",
                .offset =       MTDPART_OFS_APPEND,
                .size   =       128*1024,
        },
        {       .name   =       "linux kernel",
                .offset =       MTDPART_OFS_APPEND,
                .size   =       2*1024*1024-256*1024-128*1024,
        },
#if CONFIG_MTD_PHYSMAP_LEN > 0x400000
	{	.name	=	"resv",
		.offset	=	MTDPART_OFS_APPEND,
		.size	=	MTDPART_SIZ_FULL,
	},
#else
	{       .name   =       "ramdisk",
                .offset =       MTDPART_OFS_APPEND,
                .size   =       MTDPART_SIZ_FULL,
        },
#endif
};

static int __init 
nxc2600_nor_mtd(void)
{
	nxc2600_sm_nor_boot_flash_init(CONFIG_MTD_PHYSMAP_START);
	
	physmap_set_partitions(nxc2600_partitions, ARRAY_SIZE(nxc2600_partitions));
	return 0;
}

static void __exit 
cleanup_nxc2600_nor_mtd(void)
{
}

module_init(nxc2600_nor_mtd);
module_exit(cleanup_nxc2600_nor_mtd);

MODULE_DESCRIPTION("NXC2600 NOR flash driver");
MODULE_AUTHOR("Hardy Weng <hardy.weng@icnexus.com.tw>");
MODULE_LICENSE("GPL");

