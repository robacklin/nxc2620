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

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <asm/delay.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/dma-mapping.h>
#include <linux/completion.h>
#include <asm/cacheflush.h>
#include <asm/mach-nxc2600/nxc2600.h>
#include <asm/mach-nxc2600/gpio.h>



struct nxc2600_spi {
	struct spi_bitbang bitbang;

	volatile void __iomem *regs;
	int irq;
	unsigned freq_max;
	unsigned freq_min;

	unsigned len;
	unsigned tx_count;
	unsigned rx_count;
	const u8 *tx;
	u8 *rx;

	void (*rx_word)(struct nxc2600_spi *hw);
	void (*tx_word)(struct nxc2600_spi *hw);
	int (*txrx_bufs)(struct spi_device *spi, struct spi_transfer *t);
	irqreturn_t (*irq_callback)(struct nxc2600_spi *hw);

	struct completion master_done;

	struct spi_master *master;
	struct device *dev;
};


static void nxc2600_spi_bits_handlers_set(struct nxc2600_spi *hw, int bpw);


static void nxc2600_spi_chipsel(struct spi_device *spi, int value)
{

	switch (value) {
	case BITBANG_CS_INACTIVE:
		nxc2600_ssi_disable();
		break;

	case BITBANG_CS_ACTIVE:
		nxc2600_ssi_enable();
		break;
	}
}

static int nxc2600_spi_setupxfer(struct spi_device *spi, struct spi_transfer *t)
{
	struct nxc2600_spi *hw = spi_master_get_devdata(spi->master);
	unsigned bpw, hz;
	bpw = t ? t->bits_per_word : spi->bits_per_word;
	hz = t ? t->speed_hz : spi->max_speed_hz;

	if (bpw != 8) {
		dev_err(&spi->dev, "setupxfer: invalid bits_per_word=%d\n",
			bpw);
		return -EINVAL;
	}
	if (hz > spi->max_speed_hz || hz > hw->freq_max || hz < hw->freq_min) {
		dev_err(&spi->dev, "setupxfer: clock rate=%d out of range\n",
			hz);
		return -EINVAL;
	}

	nxc2600_spi_bits_handlers_set(hw, spi->bits_per_word);
	nxc2600_ssi_init(NXC2600_SSI_MODE_CE,
                                 NXC2600_SSI_FORMAT_MOTO_SPI,
                                 1,
                                 8,
                                 hz);

	return 0;
}

/* the spi->mode bits understood by this driver: */
#define MODEBITS (SPI_CPOL | SPI_CPHA )

static int nxc2600_spi_setup(struct spi_device *spi)
{
	struct nxc2600_spi *hw = spi_master_get_devdata(spi->master);
	if (spi->bits_per_word == 0)
		spi->bits_per_word = 8;
	if (spi->bits_per_word !=8) {
		dev_err(&spi->dev, "setup: invalid bits_per_word=%d\n",
			spi->bits_per_word);
		return -EINVAL;
	}

	if (spi->mode & ~MODEBITS) {
		dev_dbg(&spi->dev, "setup: unsupported mode bits %x\n",
			spi->mode & ~MODEBITS);
		return -EINVAL;
	}

	if (spi->max_speed_hz == 0 || spi->max_speed_hz> hw->freq_max)
		spi->max_speed_hz = hw->freq_max;
	if (spi->max_speed_hz > hw->freq_max
			|| spi->max_speed_hz < hw->freq_min)
		return -EINVAL;

	nxc2600_spi_bits_handlers_set(hw, spi->bits_per_word);
        nxc2600_ssi_init(NXC2600_SSI_MODE_CE,
                                 NXC2600_SSI_FORMAT_MOTO_SPI,
                                 1,
                                 8,
                                 spi->max_speed_hz);
	return 0;
}


/* routines to handle different word sizes in pio mode */
#define NXC2600_SPI_RX_WORD(size, mask)					\
static void nxc2600_spi_rx_word_##size(struct nxc2600_spi *hw)		\
{									\
	u32 fifoword =  NXC2600_SSI_DR_DATA(REG16(NXC2600_SSI_DR)) & (u32)(mask);		\
	if (hw->rx) {							\
		*(u##size *)hw->rx = (u##size)fifoword;			\
		hw->rx += (size) / 8;					\
	}								\
	hw->rx_count += (size) / 8;					\
}

#define NXC2600_SPI_TX_WORD(size, mask)					\
static void nxc2600_spi_tx_word_##size(struct nxc2600_spi *hw)		\
{									\
	u32 fifoword = 0;						\
	if (hw->tx) {							\
		fifoword = *(u##size *)hw->tx & (u32)(mask);		\
		hw->tx += (size) / 8;					\
	}								\
	hw->tx_count += (size) / 8;					\
	REG16(NXC2600_SSI_DR) = fifoword;				\
	REG16(NXC2600_SSI_CR0) |= NXC2600_SSI_CR0_TFLUSH;		\
}

NXC2600_SPI_RX_WORD(8,0xff)
NXC2600_SPI_TX_WORD(8,0xff)

static int nxc2600_spi_pio_txrxb(struct spi_device *spi, struct spi_transfer *t)
{
	struct nxc2600_spi *hw = spi_master_get_devdata(spi->master);

	hw->tx = t->tx_buf;
	hw->rx = t->rx_buf;
	hw->len = t->len;
	hw->tx_count = 0;
	hw->rx_count = 0;

	/* start the transfer */
	if(hw->tx)
	{
		if((hw->tx_count=nxc2600_ssi_write((uint8_t*)hw->tx,hw->len))!=hw->len)
			return hw->tx_count;
	}
	if(hw->rx)
	{
		hw->rx_count = nxc2600_ssi_read(hw->rx, hw->len);

	}
	//wait_for_completion(&hw->master_done);
	return (hw->rx_count >= hw->len) ? hw->rx_count : hw->tx_count;
}

static irqreturn_t nxc2600_spi_pio_irq_callback(struct nxc2600_spi *hw)
{
	return IRQ_HANDLED;
}

static int nxc2600_spi_txrx_bufs(struct spi_device *spi, struct spi_transfer *t)
{
	struct nxc2600_spi *hw = spi_master_get_devdata(spi->master);
	return hw->txrx_bufs(spi, t);
}
#if 0
static irqreturn_t nxc2600_spi_irq(int irq, void *dev, struct pt_regs *regs)
{
	struct nxc2600_spi *hw = dev;
	return hw->irq_callback(hw);
}
#endif
static void nxc2600_spi_bits_handlers_set(struct nxc2600_spi *hw, int bpw)
{

	hw->rx_word = &nxc2600_spi_rx_word_8;
	hw->tx_word = &nxc2600_spi_tx_word_8;
	hw->txrx_bufs = &nxc2600_spi_pio_txrxb;
	hw->irq_callback = &nxc2600_spi_pio_irq_callback;
}

static int nxc2600_spi_probe(struct platform_device *pdev)
{
	struct nxc2600_spi *hw;
	struct spi_master *master;
	int err = 0;

	master = spi_alloc_master(&pdev->dev, sizeof(struct nxc2600_spi));
	if (master == NULL) {
		dev_err(&pdev->dev, "No memory for spi_master\n");
		err = -ENOMEM;
		goto err_nomem;
	}

	hw = spi_master_get_devdata(master);

	hw->master = spi_master_get(master);
	hw->dev = &pdev->dev;


	platform_set_drvdata(pdev, hw);

	init_completion(&hw->master_done);

	hw->bitbang.master = hw->master;
	hw->bitbang.setup_transfer = nxc2600_spi_setupxfer;
	hw->bitbang.chipselect = nxc2600_spi_chipsel;
	hw->bitbang.master->setup = nxc2600_spi_setup;
	hw->bitbang.txrx_bufs = nxc2600_spi_txrx_bufs;

	hw->irq = NXC2600_IRQ_SSI;

	if (request_mem_region(NXC2600_SSI, 0x30,
			pdev->name) == NULL) {
		dev_err(&pdev->dev, "Cannot reserve iomem region\n");
		err = -ENXIO;
		goto err_no_iores;
	}

	nxc2600_spi_bits_handlers_set(hw, 8);
#if 0
	err = request_irq(hw->irq, nxc2600_spi_irq, 0, pdev->name, hw);
	if (err) {
		dev_err(&pdev->dev, "Cannot claim IRQ\n");
		goto err_no_irq;
	}
#endif
	master->bus_num = 0;
	master->num_chipselect = 1;

	hw->freq_max = 12000000;
	hw->freq_min = 1000000;

	nxc2600_gpio_vs_ssi(0);

	err = spi_bitbang_start(&hw->bitbang);
	if (err) {
		dev_err(&pdev->dev, "Failed to register SPI master %d\n",err);
		goto err_register;
	}

	dev_info(&pdev->dev,
		"spi master registered: bus_num=%d num_chipselect=%d\n",
		master->bus_num, master->num_chipselect);

	return 0;

err_register:
	free_irq(hw->irq, hw);

	release_mem_region(NXC2600_SSI, 0x30);

err_no_iores:
	spi_master_put(hw->master);
err_nomem:
	return err;
}

static int nxc2600_spi_remove(struct platform_device *pdev)
{
	struct nxc2600_spi *hw = platform_get_drvdata(pdev);

	dev_info(&pdev->dev, "spi master remove: bus_num=%d\n",
		hw->master->bus_num);

	spi_bitbang_stop(&hw->bitbang);
	//free_irq(hw->irq, hw);
	release_mem_region(NXC2600_SSI, 0x30);
	platform_set_drvdata(pdev, NULL);

	spi_master_put(hw->master);
	return 0;
}

static struct platform_driver nxc2600_spi_drv = {
	.probe = nxc2600_spi_probe,
	.remove = nxc2600_spi_remove,
	.driver = {
		.name = "nxc2600-spi",
		.owner = THIS_MODULE,
	},
};

static struct platform_device nxc2600_spi_dev =
{
        .name  = "nxc2600-spi",
        .id    = 0,
        .num_resources = 0,
        .resource =NULL,
};


static int __init nxc2600_spi_init(void)
{
	int ret = platform_driver_register(&nxc2600_spi_drv);
	if(ret)
		return ret;
	return platform_device_register(&nxc2600_spi_dev);
}
module_init(nxc2600_spi_init);

static void __exit nxc2600_spi_exit(void)
{
	platform_device_unregister(&nxc2600_spi_dev);
	platform_driver_unregister(&nxc2600_spi_drv);
}
module_exit(nxc2600_spi_exit);

MODULE_DESCRIPTION("NXC2600 SPI Driver");
MODULE_AUTHOR("Hardy Weng <hardy.weng@icnexus.com.tw>");
MODULE_LICENSE("GPL");
