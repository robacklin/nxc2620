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
 * 2008/06/20: 
 *
 *	1. fixed cannot detect hotplug twice.
 *
 * 2007/07/16: initial
 *
 *
 * Author: Hardy Weng <hardy.weng@icnexus.com.tw>
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/signal.h>
#include <linux/mm.h>
#include <asm/cacheflush.h>
#include <asm/io.h>
#include <asm/scatterlist.h>
#include <asm/irq.h>
#include <asm/unaligned.h>
#include <asm/mach-nxc2600/nxc2600.h>

#define DRIVER_NAME	"nxc2600-mmc"

struct nxc2600_mmc_host
{
        struct {
                struct dma_client client;
                struct dma_chan *channel;
                int chno;
                int len;
                int dir;
                dma_cookie_t cookie;
                spinlock_t lock;
                uint32_t dest,src;
        } dma;

        spinlock_t lock;
        uint32_t cmdat;
        uint32_t regs;

        struct mmc_host *mmc;
        struct mmc_request *mrq;
        struct mmc_data *data;
	int present;

        uint32_t clock;
        uint32_t power_mode;

        struct tasklet_struct data_task;
        struct tasklet_struct finish_task;
};



extern int nxc2600_dma_get_chno(struct dma_chan *chan);
static enum dma_state_client
nxc2600_dma_event( struct dma_client *client,
                    struct dma_chan *chan,
                    enum dma_state state)
{
        struct nxc2600_mmc_host *host = (struct nxc2600_mmc_host *)client;
        enum dma_state_client ack = DMA_NAK;

        spin_lock(&(host->dma.lock));
        switch (state)
        {
                case DMA_RESOURCE_AVAILABLE:
                        if( host->dma.channel == NULL )
                        {
                                ack = DMA_ACK;
                                host->dma.channel = chan;
                                host->dma.chno = nxc2600_dma_get_chno(chan);
                        }
                        break;
                case DMA_RESOURCE_REMOVED:
                        if( host->dma.channel == chan )
                        {
                                ack = DMA_ACK;
                                host->dma.channel = NULL;
                                host->dma.chno = -1;
                        }
                        break;
                default:
                        break;
        }
        spin_unlock(&(host->dma.lock));

        return ack;
}

static int
nxc2600_mmc_start_dma(struct nxc2600_mmc_host *host,
			struct scatterlist *sg)
{
        struct dma_async_tx_descriptor *tx;

        tx = host->dma.channel->device->device_prep_dma_memcpy(host->dma.channel, sg_dma_len(sg), 0);
        if (!tx)
                return -ENOMEM;

        tx->ack = 1;
        tx->callback = NULL; //nxc2600_mmc_dma_callback;
        tx->callback_param = host;
        tx->tx_set_src(host->dma.src, tx, 0);
        tx->tx_set_dest(host->dma.dest, tx, 0);
        host->dma.cookie = tx->tx_submit(tx);
	host->dma.channel->device->device_issue_pending(host->dma.channel);
        return 0;

}


static void
nxc2600_mmc_tasklet_finish(unsigned long param)
{
        struct nxc2600_mmc_host *host = (struct nxc2600_mmc_host *) param;
	struct mmc_request *mrq = host->mrq;
	host->mrq  = NULL;
        host->data = NULL;
        mmc_request_done(host->mmc, mrq);
}

static int
nxc2600_mmc_prepare_data(struct nxc2600_mmc_host *host,
                         struct mmc_data *data,
                         unsigned int flags)
{
        int i;

        if(host->dma.channel==NULL)
                return MMC_ERR_FIFO;

        host->dma.dir = (flags & MMC_DATA_READ)? DMA_FROM_DEVICE:DMA_TO_DEVICE;

        host->dma.len = dma_map_sg(mmc_dev(host->mmc), data->sg,
                                   data->sg_len, host->dma.dir);

        if (host->dma.len == 0)
                return MMC_ERR_INVALID;

        if(nxc2600_mmc_set_block_size(data->blksz))
                goto _dataerr;

        if(nxc2600_mmc_set_block_count(data->blocks))
                goto _dataerr;

        for ( i = 0; i < host->dma.len; i++)
        {
	        if (flags & MMC_DATA_READ)
        	{
	                host->dma.src = CPHYSADDR(NXC2600_MMC_RXFIFO);
        	        host->dma.dest = sg_dma_address(&data->sg[i]);
	        }
        	else
	        {
        	        host->dma.src = sg_dma_address(&data->sg[i]);
	                host->dma.dest = CPHYSADDR(NXC2600_MMC_TXFIFO);
        	}
		if(nxc2600_mmc_start_dma(host,&data->sg[i]))
			goto _dataerr;

        }

        data->bytes_xfered = 0;
        return MMC_ERR_NONE;

_dataerr:
        dma_unmap_sg(mmc_dev(host->mmc),data->sg,data->sg_len,host->dma.dir);
        return MMC_ERR_INVALID;
}

static int
nxc2600_mmc_send_command(
                         uint32_t cmdat,
                         struct mmc_command *cmd)
{
        switch (mmc_resp_type(cmd))
        {
                case MMC_RSP_NONE:
                        break;
                case MMC_RSP_R1:
                case MMC_RSP_R1B:
                        cmdat |= NXC2600_MMC_CMDAT_RESPONSE_FORMAT_R1_R1B;
                        break;
                case MMC_RSP_R2:
                        cmdat |= NXC2600_MMC_CMDAT_RESPONSE_FORMAT_R2;
                        break;
                case MMC_RSP_R3:
                        cmdat |= NXC2600_MMC_CMDAT_RESPONSE_FORMAT_R3;
                        break;
                default:
                        printk(KERN_INFO "nxc2600 mmc: unhandled response type %02x\n",
                        mmc_resp_type(cmd));
                        return MMC_ERR_INVALID;
        }

        if(cmd->flags&MMC_RSP_BUSY)
                cmdat|=NXC2600_MMC_CMDAT_BUSY;

        return ((nxc2600_mmc_send_cmd( cmd->opcode, cmd->arg, cmdat))? MMC_ERR_TIMEOUT:MMC_ERR_NONE);
}

static void
nxc2600_mmc_cmd_complete(struct nxc2600_mmc_host *host, 
			 unsigned int status,
			 struct mmc_command *cmd)
{

	if(cmd->flags & MMC_RSP_PRESENT)
		nxc2600_mmc_get_response(&(cmd->resp[0]),((cmd->flags & MMC_RSP_136)? 4:1));
	if(!host->present)
		cmd->error = MMC_ERR_FAILED;
        else if(status&(NXC2600_MMC_STAT_TIMEOUT_RES))
                cmd->error = MMC_ERR_TIMEOUT;
        else if((status&(NXC2600_MMC_STAT_CRC_RES_ERR)) && (cmd->flags & MMC_RSP_CRC))
        {
                if (cmd->opcode == MMC_ALL_SEND_CID ||
                    cmd->opcode == MMC_SEND_CSD ||
                    cmd->opcode == MMC_SEND_CID) {
                        /* a bogus CRC error can appear if the msb of
                           the 15 byte response is a one */
                        if ((cmd->resp[0] & 0x80000000) == 0)
                                cmd->error = MMC_ERR_BADCRC;
                }
        }

	
	if (host->data==NULL || cmd->error != MMC_ERR_NONE) 
		tasklet_schedule(&(host->finish_task));
	else if(host->data->flags & MMC_DATA_WRITE)
                nxc2600_mmc_prepare_data(host,host->data, host->data->flags);

}

static void
nxc2600_mmc_start_cmd(struct nxc2600_mmc_host *host,
			struct mmc_command *cmd,
                        unsigned int cmdat)
{

        uint32_t status;
        int timeout = 1000;
        nxc2600_mmc_send_command(cmdat,cmd);

        while (timeout--)
        {
               status = REG32(NXC2600_MMC_STAT);
               if(status&NXC2600_MMC_STAT_END_CMD_RES)
                       break;
               udelay(3);
        }

        REG16(NXC2600_MMC_IREG) = NXC2600_MMC_IREG_END_CMD_RES;
        status = REG32(NXC2600_MMC_STAT);
        nxc2600_mmc_cmd_complete(host,status,cmd);

}


static int 
nxc2600_mmc_data_complete(struct nxc2600_mmc_host *host, 
			  unsigned int status)
{
	struct mmc_data *data = host->data;
	nxc2600_mmc_disable_interrupt(NXC2600_MMC_IMASK_DATA_TRAN_DONE);
	if (!data)
		return 0;
	nxc2600_mmc_stop_clock();
	
	dma_unmap_sg(mmc_dev(host->mmc), data->sg, host->dma.len,
		     host->dma.dir);

        if(status&(NXC2600_MMC_STAT_CRC_READ_ERR|
                   NXC2600_MMC_STAT_CRC_RES_ERR|
                   NXC2600_MMC_STAT_CRC_WRITE_ERR|
                   NXC2600_MMC_STAT_CRC_WRITE_NO_CRC))
                data->error = MMC_ERR_BADCRC;
        else if(status&(NXC2600_MMC_STAT_TIMEOUT_READ|
                     NXC2600_MMC_STAT_TIMEOUT_RES))
                data->error = MMC_ERR_TIMEOUT;
        else if(status&(NXC2600_MMC_STAT_DATA_FIFO_FULL))
                data->error = MMC_ERR_FIFO;

	if (data->error == MMC_ERR_NONE)
		data->bytes_xfered = data->blocks * data->blksz;
	else
		data->bytes_xfered = 0;

	host->data = NULL;
	if (host->mrq->stop) 
	{  
		nxc2600_mmc_stop_clock();

		nxc2600_mmc_start_cmd(host, host->mrq->stop,0);
	} 
	else 
	{
		tasklet_schedule(&(host->finish_task));
	}
	return 1;
}

static irqreturn_t
nxc2600_mmc_irq(int irq,
                void *dev_id)
{

        uint16_t intr;
        uint32_t status;
        struct nxc2600_mmc_host * host = (struct nxc2600_mmc_host *)dev_id;
        intr = REG16(NXC2600_MMC_IREG);
        status = REG32(NXC2600_MMC_STAT);
        if(intr & NXC2600_MMC_IREG_DATA_TRAN_DONE)
                REG16(NXC2600_MMC_IREG) =NXC2600_MMC_IREG_DATA_TRAN_DONE;
        if(host->mrq)
        {
                if(intr & NXC2600_MMC_IREG_DATA_TRAN_DONE)
                {
                        nxc2600_mmc_data_complete(host,status);
                }
        }

        return IRQ_HANDLED;
}


static void 
nxc2600_mmc_request(struct mmc_host *mmc, 
		    struct mmc_request *mrq)
{
	struct nxc2600_mmc_host *host = mmc_priv(mmc);
	unsigned int cmdat;
	spin_lock_irq(&(host->lock));
	nxc2600_mmc_stop_clock();

	host->mrq = mrq;
 	host->data = mrq->data;
	cmdat = host->cmdat;
	host->cmdat &= ~NXC2600_MMC_CMDAT_INIT;

	if (mrq->data)
	{
		if(mrq->data->flags & MMC_DATA_READ)
			nxc2600_mmc_prepare_data(host,mrq->data,mrq->data->flags);
		cmdat |= NXC2600_MMC_CMDAT_DATA_EN|NXC2600_MMC_CMDAT_BUS_WIDTH_4BIT|NXC2600_MMC_CMDAT_DMA_EN;

                if( mrq->data->flags & MMC_DATA_WRITE)
                        cmdat |= NXC2600_MMC_CMDAT_WRITE;
                if( mrq->data->flags & MMC_DATA_STREAM)
                        cmdat |= NXC2600_MMC_CMDAT_STREAM_BLOCK;

	}
	nxc2600_mmc_start_cmd(host, host->mrq->cmd, cmdat);
	spin_unlock_irq(&(host->lock));
	if(mrq->data)
		nxc2600_mmc_enable_interrupt(NXC2600_MMC_IMASK_DATA_TRAN_DONE);
}



static irqreturn_t 
nxc2600_mmc_detect_card(int irq, 
			void *devid)
{
	struct nxc2600_mmc_host *host = (struct nxc2600_mmc_host *)devid;
	mmc_detect_change(host->mmc, msecs_to_jiffies(100));
	nxc2600_set_gpio_input_mode(CONFIG_NXC2600_MMC_HOTPLUG_PIN, 0, (!host->present)? NXC2600_GPIO_IRQ_TRIGGER_HIGH_LEVEL:NXC2600_GPIO_IRQ_TRIGGER_LOW_LEVEL);
	host->present = !host->present;
	return IRQ_HANDLED;

}


static int 
nxc2600_mmc_get_ro(struct mmc_host *mmc)
{
	return 0;
}
static void
nxc2600_mmc_set_ios(struct mmc_host* mmc,
                    struct mmc_ios* ios)
{
        struct nxc2600_mmc_host *host = mmc_priv(mmc);

        spin_lock_irq(&(host->lock));
        if(ios->power_mode != host->power_mode)
        {
                host->power_mode = ios->power_mode;
                if (ios->power_mode == MMC_POWER_OFF)
                        nxc2600_mmc_disable_power();
                else if (ios->power_mode == MMC_POWER_UP)
                        nxc2600_mmc_enable_power();
                else if( ios->power_mode == MMC_POWER_ON)
                        host->cmdat |= NXC2600_MMC_CMDAT_INIT;
        }
        if (ios->clock && ios->clock != host->clock)
        {
                nxc2600_mmc_set_clock(1,ios->clock);
                host->clock = ios->clock;
        }
        spin_unlock_irq(&(host->lock));
}

static const struct mmc_host_ops nxc2600_mmc_ops = {
	.request	= nxc2600_mmc_request,
	.get_ro		= nxc2600_mmc_get_ro,
	.set_ios	= nxc2600_mmc_set_ios,
};

#define res_size(_r) (((_r)->end - (_r)->start) + 1)
static int nxc2600_mmc_probe(struct platform_device *pdev)
{
	int  ret;
	struct mmc_host *mmc;
	struct nxc2600_mmc_host *host = NULL;
	struct resource *res;

	nxc2600_mmc_init();
        nxc2600_mmc_enable_power();
        nxc2600_mmc_reset();
        nxc2600_mmc_disable_interrupt(0xff);
        nxc2600_mmc_stop_clock();



	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
        if (res == NULL) 
	{
                printk(KERN_ERR "mmc: no IO resource.\n");
                return -ENOENT;
        }

        if(request_mem_region(res->start,
                              res_size(res),
                              "mmc") == NULL )
        {
                printk(KERN_ERR "mmc: request IO failed.\n");
        	return -ENXIO;
        }


	mmc = mmc_alloc_host(sizeof(struct nxc2600_mmc_host), &pdev->dev);
	if (!mmc) {
		ret = -ENOMEM;
		goto out;

	}
	mmc->ops = &nxc2600_mmc_ops;
        mmc->bus_ops = NULL;

        mmc->f_min = 24000000;
        mmc->f_max = 24000000;

        mmc->max_blk_size = NXC2600_MMC_MAX_BLOCK_SIZE;
        mmc->max_blk_count = 1; //NXC2600_MMC_MAX_BLOCK_COUNT;

        mmc->max_seg_size = PAGE_SIZE; //mmc->max_blk_size*mmc->max_blk_count;
        mmc->max_phys_segs = 1;
	mmc->ocr_avail = MMC_VDD_32_33|MMC_VDD_33_34;

	host = mmc_priv(mmc); 
	spin_lock_init(&(host->dma.lock));
        host->dma.channel = NULL;
        host->dma.client.event_callback = nxc2600_dma_event;
        dma_cap_set(DMA_MEMCPY, host->dma.client.cap_mask);
        dma_async_client_register(&(host->dma.client));
        dma_async_client_chan_request(&(host->dma.client));

	host->present = 0;
	host->mmc = mmc;
        host->clock = 0;
        host->power_mode = MMC_POWER_OFF;
        host->cmdat = 0;
        host->dma.len = 0;
        host->dma.cookie = 0;
        tasklet_init(&(host->finish_task), nxc2600_mmc_tasklet_finish,
                       (unsigned long) host);


	spin_lock_init(&host->lock);
        ret = request_irq(NXC2600_IRQ_MSC, nxc2600_mmc_irq, IRQF_DISABLED, "MMC/SD", host);

        if( ret )
                goto out;

        ret = request_irq(NXC2600_MMC_HOTPLUG_IRQ, nxc2600_mmc_detect_card,
                          IRQF_TRIGGER_LOW|IRQF_DISABLED, "MMC/SD detect", host);
        if( ret )
                goto _request_detect_card_irq_failed;

	platform_set_drvdata(pdev, mmc);


	mmc_add_host(mmc);
	return 0;

_request_detect_card_irq_failed:
	free_irq(NXC2600_IRQ_MSC, &host);

 out:
	if (mmc)
		mmc_free_host(mmc);

	return ret;
}

static int 
nxc2600_mmc_remove(struct platform_device *pdev)
{
	struct mmc_host *mmc = platform_get_drvdata(pdev);
	struct nxc2600_mmc_host *host = mmc_priv(mmc);

	platform_set_drvdata(pdev, NULL);

	disable_irq(NXC2600_IRQ_MSC);
	mmc_remove_host(mmc);
        nxc2600_mmc_stop_clock();
        nxc2600_mmc_disable_power();

        dma_async_client_unregister(&(host->dma.client));


        free_irq(NXC2600_IRQ_MSC, host);
        free_irq(NXC2600_MMC_HOTPLUG_IRQ, host);
        mmc_free_host(mmc);
	return 0;
}

#ifdef CONFIG_PM
static int 
nxc2600_mmc_suspend(struct platform_device *dev, 
		    pm_message_t state)
{
	struct mmc_host *mmc = platform_get_drvdata(dev);
	int ret = 0;

	if (mmc)
		ret = mmc_suspend_host(mmc, state);

	return ret;
}

static int 
nxc2600_mmc_resume(struct platform_device *dev)
{
	struct mmc_host *mmc = platform_get_drvdata(dev);
	int ret = 0;

	if (mmc)
		ret = mmc_resume_host(mmc);

	return ret;
}
#else
#define nxc2600_mmc_suspend	NULL
#define nxc2600_mmc_resume	NULL
#endif

static struct resource nxc2600_mmc_resources[] =
{
        {
                .start          = CPHYSADDR(NXC2600_MMC),
                .end            = CPHYSADDR(NXC2600_MMC) + 0x40 - 1,
                .flags          = IORESOURCE_MEM,
        },
        {
                .start          = NXC2600_IRQ_MSC,
                .end            = NXC2600_IRQ_MSC,
                .flags          = IORESOURCE_IRQ,
        },
};
static u64 mmc_dmamask = ~0UL;

static struct platform_device nxc2600_mmc_dev =
{
        .name           = DRIVER_NAME,
        .id             = 0,
        .num_resources  = ARRAY_SIZE(nxc2600_mmc_resources),
        .resource       = nxc2600_mmc_resources,
        .dev    =       {
                .dma_mask               =       &mmc_dmamask,
        },
};

static struct platform_driver nxc2600_mmc_driver = 
{
	.probe		= nxc2600_mmc_probe,
	.remove		= nxc2600_mmc_remove,
	.suspend	= nxc2600_mmc_suspend,
	.resume		= nxc2600_mmc_resume,
	.driver		= {
		.name	= DRIVER_NAME,
	},
};


static int __init 
nxc2600_mmc_module_init(void)
{
	platform_driver_register(&nxc2600_mmc_driver);
	return platform_device_register(&nxc2600_mmc_dev);
}

static void __exit 
nxc2600_mmc_cleanup(void)
{
	platform_device_unregister(&nxc2600_mmc_dev);
	platform_driver_unregister(&nxc2600_mmc_driver);

}

module_init(nxc2600_mmc_module_init);
module_exit(nxc2600_mmc_cleanup);

MODULE_AUTHOR("Hardy Weng <hardy.weng@icnexus.com.tw>");
MODULE_DESCRIPTION("NXC2600 SD/Multimedia Card Interface Driver");
MODULE_LICENSE("GPL");

