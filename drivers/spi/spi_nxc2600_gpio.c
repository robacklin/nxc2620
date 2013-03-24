/*
 *
 * Copyright (C) 2007,2008 IC Nexus Co., LTD.
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
 *
 *
 * 2008/03/01: initial
 *
 *
 * Author: Hardy Weng <hardy.weng@icnexus.com.tw>
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>

#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>

#include <asm/cacheflush.h>
#include <asm/mach-nxc2600/nxc2600.h>
#include <asm/mach-nxc2600/gpio.h>

#define NXC2600_SPI_CLK_GPIO_PIN	72
#define	NXC2600_SPI_CE_GPIO_PIN		73
#define	NXC2600_SPI_DT_GPIO_PIN		74
#define	NXC2600_SPI_DR_GPIO_PIN		75
#define	NXC2600_SPI_CE2_GPC_GPIO_PIN	76
struct spi_gpio
{
	struct spi_bitbang	bitbang;
	struct platform_device	*dev;
	GPIO	*clk,*ce,*dt,*dr,*ce2_gpc;
	
};

static inline struct spi_gpio *spidev_to_sg(struct spi_device *spi)
{
	return (struct spi_gpio *) spi_master_get_devdata(spi->master);
}

static inline void setsck(struct spi_device *dev, int on)
{
	struct spi_gpio *sg = spidev_to_sg(dev);
	gpio_set_out(sg->clk, on ? 1 : 0);
}

static inline void setmosi(struct spi_device *dev, int on)
{
	struct spi_gpio *sg = spidev_to_sg(dev);

	gpio_set_out(sg->dt, on ? 1 : 0);
}

static inline u32 getmiso(struct spi_device *dev)
{
	struct spi_gpio *sg = spidev_to_sg(dev);
	u32 val =  gpio_get_in(sg->dr) ? 1 : 0;
	return val;
}

#define spidelay(x) ndelay(x)

#define	EXPAND_BITBANG_TXRX
#include <linux/spi/spi_bitbang.h>


static u32 spi_gpio_txrx_mode0(struct spi_device *spi,
				      unsigned nsecs, u32 word, u8 bits)
{
	return bitbang_txrx_be_cpha0(spi, nsecs, 0, word, bits);
}

static u32 spi_gpio_txrx_mode1(struct spi_device *spi,
				      unsigned nsecs, u32 word, u8 bits)
{
	return bitbang_txrx_be_cpha1(spi, nsecs, 0, word, bits);
}

static u32 spi_gpio_txrx_mode2(struct spi_device *spi,
				      unsigned nsecs, u32 word, u8 bits)
{
	return bitbang_txrx_be_cpha0(spi, nsecs, 1, word, bits);
}

static u32 spi_gpio_txrx_mode3(struct spi_device *spi,
				      unsigned nsecs, u32 word, u8 bits)
{
	return bitbang_txrx_be_cpha1(spi, nsecs, 1, word, bits);
}


static void spi_gpio_chipselect(struct spi_device *dev, int value)
{
	struct spi_gpio *sg = spidev_to_sg(dev);
        switch (value) {
        case BITBANG_CS_ACTIVE:
		gpio_set_out(sg->ce, 0);
                break;
        case BITBANG_CS_INACTIVE:
		gpio_set_out(sg->ce, 1);
                break;
        }

}

static int spi_gpio_probe(struct platform_device *dev)
{
	struct spi_master	*master;
	struct spi_gpio  *sp;
	int ret;


	master = spi_alloc_master(&dev->dev, sizeof(struct spi_gpio));
	if (master == NULL) {
		dev_err(&dev->dev, "failed to allocate spi master\n");
		ret = -ENOMEM;
		goto err;
	}

	sp = spi_master_get_devdata(master);

	platform_set_drvdata(dev, sp);

	sp->clk = gpio_request( NXC2600_SPI_CLK_GPIO_PIN,
				GPIO_DIR_OUT,
				&(dev->dev));
	sp->ce  = gpio_request( NXC2600_SPI_CE_GPIO_PIN,
				GPIO_DIR_OUT,
				&(dev->dev));
	sp->dt  = gpio_request( NXC2600_SPI_DT_GPIO_PIN,
				GPIO_DIR_OUT,
				&(dev->dev));
	sp->dr	= gpio_request( NXC2600_SPI_DR_GPIO_PIN,
				GPIO_DIR_IN,
				&(dev->dev));
	/* setup spi bitbang adaptor */
	sp->bitbang.master = spi_master_get(master);
	sp->bitbang.chipselect = spi_gpio_chipselect;

	sp->bitbang.txrx_word[SPI_MODE_0] = spi_gpio_txrx_mode0;
	sp->bitbang.txrx_word[SPI_MODE_1] = spi_gpio_txrx_mode1;
	sp->bitbang.txrx_word[SPI_MODE_2] = spi_gpio_txrx_mode2;
	sp->bitbang.txrx_word[SPI_MODE_3] = spi_gpio_txrx_mode3;

	/* set state of spi pins */
	gpio_set_out(sp->clk, 0);
	gpio_set_out(sp->dt, 0);
	gpio_set_out(sp->ce, 1);


	master->bus_num = 0;
        master->num_chipselect = 1;


	ret = spi_bitbang_start(&sp->bitbang);
	if (ret)
		goto err_no_bitbang;

	printk("NXC2600 GPIO SPI initialed.\n");
	return 0;

 err_no_bitbang:
	spi_master_put(sp->bitbang.master);
 err:
	return ret;

}

static int spi_gpio_remove(struct platform_device *dev)
{
	struct spi_gpio *sp = platform_get_drvdata(dev);

	spi_bitbang_stop(&sp->bitbang);
	spi_master_put(sp->bitbang.master);

	return 0;
}

/* all gpio should be held over suspend/resume, so we should
 * not need to deal with this
*/

#define spi_gpio_suspend NULL
#define spi_gpio_resume NULL


static struct platform_driver spi_gpio_drv = {
	.probe		= spi_gpio_probe,
        .remove		= spi_gpio_remove,
        .suspend	= spi_gpio_suspend,
        .resume		= spi_gpio_resume,
        .driver		= {
		.name	= "nxc2600_spi_gpio",
		.owner	= THIS_MODULE,
        },
};

static struct platform_device spi_gpio_dev =
{
        .name  = "nxc2600_spi_gpio",
        .id    = 0,
        .num_resources = 0,
        .resource =NULL,
};

static int __init spi_gpio_init(void)
{
        int ret = platform_driver_register(&spi_gpio_drv);
	if(ret)
		return ret;
	return platform_device_register(&spi_gpio_dev);
}

static void __exit spi_gpio_exit(void)
{
	platform_device_unregister(&spi_gpio_dev);
        platform_driver_unregister(&spi_gpio_drv);
}

module_init(spi_gpio_init);
module_exit(spi_gpio_exit);

MODULE_DESCRIPTION("NXC2600 GPIO SPI Driver");
MODULE_AUTHOR("Hardy Weng <hardy.weng@icnexus.com.tw>");
MODULE_LICENSE("GPL");
