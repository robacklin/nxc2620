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
 * 2008/03/10: 
 *    
 *      1. fixed wrong i2c code.
 *
 * 2007/07/16: initial
 *
 *
 * Author: Hardy Weng <hardy.weng@icnexus.com.tw>
 *
 */


#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/i2c.h>

#include <asm/cacheflush.h>
#include <asm/delay.h>
#include <asm/mach-nxc2600/nxc2600.h>

static int
nxc2600_xfer(struct i2c_adapter *adap,
	     struct i2c_msg *msgs,
             int num)
{
	int i;
	for(i=0;i<num;i++)
	{
		if(msgs[i].flags&I2C_M_RD)
		{
			if(nxc2600_i2c_send_start( msgs[i].addr, 1))
				break;
			if(nxc2600_i2c_read_data(msgs[i].addr,
						 msgs[i].buf, 
						 msgs[i].len)<=0) break;
		}
		else
		{
			if(nxc2600_i2c_send_start( msgs[i].addr, 0))
                                break;

			if(nxc2600_i2c_write_data(msgs[i].addr,
						 msgs[i].buf,
						 msgs[i].len)!=msgs[i].len) break;
		}
		nxc2600_i2c_send_stop();
	}
	return i;
}

static u32
nxc2600_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm nxc2600_algo = 
{
	.master_xfer	= nxc2600_xfer,
	.functionality	= nxc2600_func,
};

static struct i2c_adapter nxc2600_adapter = 
{
	name:              "nxc2600 adapter",
	id:                0,
	algo:              &nxc2600_algo,
};

static int __init
i2c_nxc2600_init(void)
{
        nxc2600_i2c_init(100000);

        if (i2c_add_adapter(&nxc2600_adapter) < 0) 
	{
		printk("NXC2600 I2C : failed to initialize.\n");
                return -ENODEV;
	}
	printk(KERN_INFO "NXC2600 I2C initialized.\n ");
	return 0;
}

static void __exit
i2c_nxc2600_exit(void)
{
	i2c_del_adapter(&nxc2600_adapter);
}

module_init (i2c_nxc2600_init);
module_exit (i2c_nxc2600_exit);

MODULE_AUTHOR("Hardy Weng <hardy.weng@icnexus.com.tw>");
MODULE_DESCRIPTION("NXC2600 I2C Driver");
MODULE_LICENSE("GPL");

