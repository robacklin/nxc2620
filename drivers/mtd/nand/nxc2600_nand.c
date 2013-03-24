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
#include <linux/miscdevice.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>
#include <asm/io.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <asm/cacheflush.h>
#include <asm/mach-nxc2600/nxc2600.h>

static struct mtd_info *nxc2600_nand_mtd;


static struct mtd_partition nxc2600_partition[] =
{
	{	.name	=	"bootloader",
		.offset	=	0,
		.size	=	256*1024,
	},
	{	.name	=	"params",
		.offset	=	MTDPART_OFS_APPEND,
		.size	=	256*1024,
	},
	{	.name	=	"linux kernel",
		.offset	=	MTDPART_OFS_APPEND,
		.size	=	2*1024*1024,
	},
	{
		.name	=	"initrd",
		.offset	=	MTDPART_OFS_APPEND,
		.size	=	2*1024*1024+512*1024,
	},
	{	.name	=	"rootfs",
		.offset	=	MTDPART_OFS_APPEND,
		.size	=	120*1024*1024,
	},
	{
		.name   =	"private data",
		.offset =	MTDPART_OFS_APPEND,
		.size	=	MTDPART_SIZ_FULL,
	},
};


static void
nxc2600_cmd_ctrl( struct mtd_info *mtd,
		   int cmd,
		   unsigned int ctrl )
{
	static int nce = 1;

	if( cmd == NAND_CMD_NONE )
		return;

	if(ctrl&NAND_NCE)
	{
		if(ctrl&NAND_CTRL_CHANGE)
			nce = !nce;
		nxc2600_nand_set_nce(nce);
	}
	if(ctrl&NAND_CLE)
		writeb(cmd, nxc2600_nand_set_cle(1));
	else
		writeb(cmd, nxc2600_nand_set_ale(1));

}

static void
nxc2600_nand_ecc_hwctl(  struct mtd_info *mtd,
			  int mode )
{

	switch(mode)
	{
		case NAND_ECC_WRITE:
		case NAND_ECC_READ:
			nxc2600_nand_ecc_enable();
			break;
	}
}

static int
nxc2600_nand_ecc_calculate( struct mtd_info *mtd,
			    const uint8_t *dat,
                            uint8_t *ecc_code)
{
	uint32_t ecc;
	uint8_t *tmp = (uint8_t*)&ecc;
	nxc2600_nand_ecc_disable();
	ecc = nxc2600_nand_get_ecc();
	ecc_code[0] = tmp[0];
	ecc_code[1] = tmp[1];
	ecc_code[2] = tmp[2];
	return 0;
}

static int
nxc2600_nand_ecc_correct( struct mtd_info *mtd,
			 uint8_t *dat,
                         uint8_t *read_ecc,
                         uint8_t *calc_ecc)
{
#if 1
	if(*read_ecc == 0xff) return 0;
	return ((*read_ecc ^ *calc_ecc)? -1:0);
#else
       uint8_t a, b, c, d1, d2, d3, add, bit, i;

       /* Do error detection */
       d1 = calc_ecc[0] ^ read_ecc[0];
       d2 = calc_ecc[1] ^ read_ecc[1];
       d3 = calc_ecc[2] ^ read_ecc[2];

       if ((d1 | d2 | d3) == 0) {
               /* No errors */
               return 0;
       }
       else {
               a = (d1 ^ (d1 >> 1)) & 0x55;
               b = (d2 ^ (d2 >> 1)) & 0x55;
               c = (d3 ^ (d3 >> 1)) & 0x54;

               /* Found and will correct single bit error in the data */
               if ((a == 0x55) && (b == 0x55) && (c == 0x54)) {
                       c = 0x80;
                       add = 0;
                       a = 0x80;
                       for (i=0; i<4; i++) {
                               if (d1 & c)
                                       add |= a;
                               c >>= 2;
                               a >>= 1;
                       }
                       c = 0x80;
                       for (i=0; i<4; i++) {
                               if (d2 & c)
                                       add |= a;
                               c >>= 2;
                               a >>= 1;
                       }
                       bit = 0;
                       b = 0x04;
                       c = 0x80;
                       for (i=0; i<3; i++) {
                               if (d3 & c)
                                       bit |= b;
                               c >>= 2;
                               b >>= 1;
                       }
                       b = 0x01;
                       a = dat[add];
                       a ^= (b << bit);
                       dat[add] = a; 
                       return 0;
               }
               else {
                       i = 0;
                       while (d1) {
                               if (d1 & 0x01)
                                       ++i;
                               d1 >>= 1;
                       }
                       while (d2) {
                               if (d2 & 0x01)
                                       ++i;
                               d2 >>= 1;
                       }
                       while (d3) {
                               if (d3 & 0x01)
                                       ++i;
                               d3 >>= 1;
                       }
                       if (i == 1) {
                               /* ECC Code Error Correction */
                               read_ecc[0] = calc_ecc[0];
                               read_ecc[1] = calc_ecc[1];
                               read_ecc[2] = calc_ecc[2];
                               return 0;
                       }
                       else {
                               /* Uncorrectable Error */
                               printk("uncorrectable ECC error\n");
                               return -1;
                       }
               }
       }

       /* Should never happen */
       return -1;

#endif
}

static int
nxc2600_ready( struct mtd_info *mtd )
{
	int ready = (nxc2600_nand_is_ready()? 1:0);
	if(ready)
		udelay(10);
	return ready;
}

static int
nxc2600_init( void )
{
#if 0
	static struct nand_ecclayout nxc2600_nand_oobinfo =
	{
        	.eccbytes = 3,
	        .eccpos = {NXC2600_NAND_ECC_POS, 
			   NXC2600_NAND_ECC_POS+1,
			   NXC2600_NAND_ECC_POS+2,
			   NXC2600_NAND_ECC_POS+3,
			   NXC2600_NAND_ECC_POS+4,
			   NXC2600_NAND_ECC_POS+5,
			   NXC2600_NAND_ECC_POS+6,
			   NXC2600_NAND_ECC_POS+7,
                           NXC2600_NAND_ECC_POS+8,
                           NXC2600_NAND_ECC_POS+9,
                           NXC2600_NAND_ECC_POS+10,
                           NXC2600_NAND_ECC_POS+11,
                           NXC2600_NAND_ECC_POS+12,
                           NXC2600_NAND_ECC_POS+13,
                           NXC2600_NAND_ECC_POS+14,
                           NXC2600_NAND_ECC_POS+15,
                           NXC2600_NAND_ECC_POS+16,
                           NXC2600_NAND_ECC_POS+17,
                           NXC2600_NAND_ECC_POS+18,
                           NXC2600_NAND_ECC_POS+19,
                           NXC2600_NAND_ECC_POS+20,
                           NXC2600_NAND_ECC_POS+21,
                           NXC2600_NAND_ECC_POS+22,
                           NXC2600_NAND_ECC_POS+23,
                           NXC2600_NAND_ECC_POS+24,
                           NXC2600_NAND_ECC_POS+25,
                           NXC2600_NAND_ECC_POS+26,
                           NXC2600_NAND_ECC_POS+27,
                           NXC2600_NAND_ECC_POS+28,
                           NXC2600_NAND_ECC_POS+29,
                           NXC2600_NAND_ECC_POS+30,
                           NXC2600_NAND_ECC_POS+31,

                           },

	};
#endif
	static const char *part_probes[] = { "cmdlinepart", NULL };
	struct nand_chip *this;
	const char *part_type;
        struct mtd_partition *mtd_parts;
        int mtd_parts_nb = 0;
        int ret;
	int pagesize;
	int ecccount;
	
	nxc2600_nand_init();

	nxc2600_nand_mtd = kzalloc(sizeof(struct mtd_info) +
                                  sizeof(struct nand_chip),
                                  GFP_KERNEL);
        if (!nxc2600_nand_mtd) 
	{
                printk("Unable to allocate NXC2600 NAND MTD device structure.\n");
                return -ENOMEM;
        }

	this = (struct nand_chip *)&nxc2600_nand_mtd[1];

	pagesize = nxc2600_nand_get_page_size();
	ecccount = pagesize / NXC2600_NAND_ECC_BLOCK;

	nxc2600_nand_mtd->owner = THIS_MODULE;
        nxc2600_nand_mtd->priv = this;
	nxc2600_nand_mtd->name = "nxc2600-nand";

	this->IO_ADDR_R = (void*)NXC2600_NAND_DATA_PORT;
        this->IO_ADDR_W = (void*)NXC2600_NAND_DATA_PORT;
        this->cmd_ctrl = nxc2600_cmd_ctrl;
        this->dev_ready = nxc2600_ready;
	this->chip_delay = 100;
	this->ecc.size = NXC2600_NAND_ECC_BLOCK;
	this->ecc.mode = NAND_ECC_HW;
	this->ecc.hwctl = nxc2600_nand_ecc_hwctl;
	this->ecc.calculate = nxc2600_nand_ecc_calculate;
	this->ecc.correct = nxc2600_nand_ecc_correct;
	//this->ecc.layout = &nxc2600_nand_oobinfo;
	this->ecc.bytes = 3;
	
	this->options = ((nxc2600_nand_get_bus_width()==16)? NAND_BUSWIDTH_16:0);

	if (nand_scan (nxc2600_nand_mtd, 1)) 
	{
                printk(KERN_NOTICE "No NAND device\n");
                ret = -ENXIO;
                goto err;
        }

#ifdef CONFIG_MTD_CMDLINE_PARTS
        mtd_parts_nb = parse_mtd_partitions(nxc2600_nand_mtd, part_probes,
                                            &mtd_parts, 0);
        if (mtd_parts_nb > 0)
                part_type = "command line";
        else
                mtd_parts_nb = 0;
#endif
        if (!mtd_parts_nb) 
	{
                mtd_parts = nxc2600_partition;
                mtd_parts_nb = ARRAY_SIZE(nxc2600_partition);
                part_type = "default";
        }

        printk(KERN_NOTICE "Using %s partition definition\n", part_type);
        ret = add_mtd_partitions(nxc2600_nand_mtd, mtd_parts, mtd_parts_nb);
        if (ret)
                goto err;

        return 0;

err:
        kfree(nxc2600_nand_mtd);

        return ret;

}
static void 
nxc2600_cleanup(void)
{
        nand_release(nxc2600_nand_mtd);

        kfree(nxc2600_nand_mtd);
}

module_init(nxc2600_init);
module_exit(nxc2600_cleanup);

MODULE_DESCRIPTION("NXC2600 NAND flash driver");
MODULE_AUTHOR("Hardy Weng <hardy.weng@icnexus.com.tw>");
MODULE_LICENSE("GPL");

