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
#include <linux/module.h>
#include <linux/async_tx.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/memory.h>
#include <linux/ioport.h>
#include <asm/addrspace.h>
#include <asm/cacheflush.h>
#include <asm/cacheops.h>
#include <asm/mach-nxc2600/nxc2600.h>

static char nxc2600_dma_name[] = "nxc2600_dma";

#define MAX_DMA_DESCS	64
struct nxc2600_dma_desc
{
	struct dma_async_tx_descriptor  desc;
	size_t				len;
	struct nxc2600_dma_mode		dma_mode;
	int				memset_value;
	uint32_t			src,dest;
	dma_cookie_t			cookie;
};

struct nxc2600_dma_chan
{
	struct dma_chan			chan;

	int				ch;
	spinlock_t			lock;
	spinlock_t			finish_lock;
	void __iomem			*io_base;
        struct tasklet_struct 		finish_task;

	struct nxc2600_dma_desc		desc[MAX_DMA_DESCS];
	int				current_desc;
	int				finish_desc;
	int				last_used_desc;
};

#ifndef CONFIG_ICNEXUS_NXC2620
#define	MAX_NXC2600_DMA_CHANNELS	(8)
#else
#define MAX_NXC2600_DMA_CHANNELS	(6)
#endif

int
nxc2600_dma_get_chno( struct dma_chan *chan )
{
	struct nxc2600_dma_chan *ch = (struct nxc2600_dma_chan*)chan;
	return ch->ch;
}
EXPORT_SYMBOL(nxc2600_dma_get_chno);

static dma_cookie_t
nxc2600_desc_assign_cookie(struct nxc2600_dma_chan *chan )
{
	dma_cookie_t cookie = chan->chan.cookie;
	cookie++;
	if (cookie < 0)
		cookie = 1;
	chan->chan.cookie = cookie;
	return cookie;
}

static void
nxc2600_dma_start(struct nxc2600_dma_chan *chan,
		   struct nxc2600_dma_desc *desc)
{
	if(desc->cookie>0)
        {
                nxc2600_dma_disable(chan->ch);
                nxc2600_dma_set_src( chan->ch, (void*)desc->src );
                nxc2600_dma_set_dest( chan->ch, (void*)desc->dest );
                nxc2600_dma_set_mode( chan->ch, desc->len, &desc->dma_mode );
                flush_cache_all();
                nxc2600_dma_start_transfer(chan->ch);
        }
}

static dma_cookie_t
nxc2600_dma_tx_submit(struct dma_async_tx_descriptor *tx)
{
	struct nxc2600_dma_chan *chan = (struct nxc2600_dma_chan *)tx->chan;
	struct nxc2600_dma_desc *desc = (struct nxc2600_dma_desc*)tx;

	spin_lock_irq(&chan->lock);
	desc->cookie = nxc2600_desc_assign_cookie(chan);
	spin_unlock_irq(&chan->lock);

	return desc->cookie;
}


static int 
nxc2600_dma_alloc_chan_resources(struct dma_chan *dchan)
{
	static int ch = 0;
	struct nxc2600_dma_chan *chan = (struct nxc2600_dma_chan *)dchan;
	if(ch!=chan->ch)
		return -EBUSY;

	ch++;
	if(ch>=MAX_NXC2600_DMA_CHANNELS)
		ch=0;

	return 0;

}

static void
nxc2600_dma_mem_set( dma_addr_t addr,
                     struct dma_async_tx_descriptor *tx,
                     int index,
		     int fill,
		     int is_src )
{
	struct nxc2600_dma_desc *desc = (struct nxc2600_dma_desc*)tx;

        static const struct nxc2600_dma_mode dma_dest_modes[] =
        {
                NXC2600_DMA_MODE_AIC_TX,
		NXC2600_DMA_MODE_MMC_TX,
		NXC2600_DMA_MODE_UART0_TX,
		NXC2600_DMA_MODE_UART1_TX,
		NXC2600_DMA_MODE_UART2_TX,
		NXC2600_DMA_MODE_UART3_TX,
	};
	static const struct nxc2600_dma_mode dma_src_modes[] =
        {
                NXC2600_DMA_MODE_AIC_RX,
		NXC2600_DMA_MODE_MMC_RX,
		NXC2600_DMA_MODE_UART0_RX,
                NXC2600_DMA_MODE_UART1_RX,
                NXC2600_DMA_MODE_UART2_RX,
                NXC2600_DMA_MODE_UART3_RX,
        };
	static const struct nxc2600_dma_mode dma_mem[] =
	{
                NXC2600_DMA_MODE_MEMCPY,
		NXC2600_DMA_MODE_MEMSET,
        };
	int i, size;
        const struct nxc2600_dma_mode *dma_modes;

        if( !desc->dma_mode.addr )
        {

		desc->dma_mode = dma_mem[(fill)? 1:0];

		if(is_src)
		{
			dma_modes = dma_src_modes;
			size = ARRAY_SIZE(dma_src_modes);
		}
		else
		{
			dma_modes = dma_dest_modes;
                        size = ARRAY_SIZE(dma_dest_modes);
		}
                for( i = 0; i<size; i++)
                {
                        if( CPHYSADDR(dma_modes[i].addr) == CPHYSADDR(addr) )
			{
				desc->dma_mode = dma_modes[i];
                                break;
			}
                }
                if(desc->dma_mode.set_packet!=NULL)
                        desc->dma_mode.set_packet(&(desc->dma_mode));

	}

	if(is_src)
		desc->src = addr;
	else
 		desc->dest = addr;

}


static void
nxc2600_dma_memcpy_set_dest( dma_addr_t addr,
                            struct dma_async_tx_descriptor *tx,
                            int index)
{
	nxc2600_dma_mem_set(addr, tx, index, 0, 0);
}

static void
nxc2600_dma_memset_set_dest( dma_addr_t addr,
                            struct dma_async_tx_descriptor *tx,
                            int index)
{
	
        nxc2600_dma_mem_set(addr, tx, index, 1, 0);
}


static void
nxc2600_dma_memcpy_set_src( dma_addr_t addr, 
			 struct dma_async_tx_descriptor *tx,
			 int index )
{
	nxc2600_dma_mem_set(addr, tx, index, 0, 1);
}

static void
nxc2600_dma_memset_set_src( dma_addr_t addr,
                         struct dma_async_tx_descriptor *tx,
                         int index )
{
        nxc2600_dma_mem_set(addr, tx, index, 1, 1);
}


static struct dma_async_tx_descriptor *
nxc2600_dma_prep_dma_memcpy( struct dma_chan *dchan,
			     size_t len, 
			     int int_en)
{
	struct nxc2600_dma_chan *chan = (struct nxc2600_dma_chan *)dchan;
        struct nxc2600_dma_desc *desc = NULL;
	int ndesc;

        BUG_ON((len<1) || (len > NXC2600_DMA_MAX_BYTE_COUNT));

	spin_lock_irq(&chan->lock);
	ndesc = chan->last_used_desc+1;
	if(ndesc>=MAX_DMA_DESCS)
		ndesc = 0;
	if(chan->desc[ndesc].len == 0 )
	{
		chan->last_used_desc = ndesc;
		desc = &chan->desc[ndesc];
		dma_async_tx_descriptor_init(&desc->desc,dchan);
		desc->desc.tx_set_src = nxc2600_dma_memcpy_set_src;
		desc->desc.tx_set_dest = nxc2600_dma_memcpy_set_dest;
		desc->desc.tx_submit = nxc2600_dma_tx_submit;
		desc->len = len;
		desc->cookie = 0;
		desc->dma_mode.addr = 0;
	}
	spin_unlock_irq(&chan->lock);

        return ((desc==NULL)? NULL:&desc->desc);
}

static struct dma_async_tx_descriptor *
nxc2600_dma_prep_dma_memset(struct dma_chan *dchan, 
			    int value, 
			    size_t len,
			    int int_en)
{
	struct nxc2600_dma_chan *chan = (struct nxc2600_dma_chan *)dchan;
        struct nxc2600_dma_desc *desc = NULL;
        int ndesc;

        BUG_ON((len<1) || (len > NXC2600_DMA_MAX_BYTE_COUNT));


        spin_lock_irq(&chan->lock);
        ndesc = chan->last_used_desc+1;
        if(ndesc>=MAX_DMA_DESCS)
                ndesc = 0;
        if(chan->desc[ndesc].len == 0 )
        {
                chan->last_used_desc = ndesc;
                desc = &chan->desc[ndesc];
                dma_async_tx_descriptor_init(&desc->desc,dchan);
                desc->desc.tx_set_src = nxc2600_dma_memset_set_src;
                desc->desc.tx_set_dest = nxc2600_dma_memset_set_dest;
                desc->desc.tx_submit = nxc2600_dma_tx_submit;
                desc->len = len;
		desc->cookie = 0;
                desc->dma_mode.addr = 0;
		desc->memset_value = value;
		desc->src = (uint32_t)&desc->memset_value;
        }

	spin_unlock_irq(&chan->lock);

	return ((desc==NULL)? NULL:&desc->desc);
}

static void 
nxc2600_dma_dependency_added(struct dma_chan *dchan)
{
}

static void 
nxc2600_dma_free_chan_resources(struct dma_chan *dchan)
{
	//struct nxc2600_dma_chan *chan = (struct nxc2600_dma_chan *)dchan;
}

static void
nxc2600_dma_tasklet_finish(unsigned long param)
{
        struct nxc2600_dma_chan *chan = (struct nxc2600_dma_chan*) param;
	spin_lock_irq(&(chan->finish_lock));
        if(chan->desc[chan->finish_desc].desc.callback)
		chan->desc[chan->finish_desc].desc.callback(chan->desc[chan->finish_desc].desc.callback_param);

	chan->desc[chan->finish_desc].len = 0;

	chan->finish_desc++;
	if(chan->finish_desc>=MAX_DMA_DESCS)
		chan->finish_desc=0;

	spin_unlock_irq(&(chan->finish_lock));
}

static enum dma_status 
nxc2600_dma_is_complete(struct dma_chan *dchan,
			dma_cookie_t cookie,
			dma_cookie_t *done,
			dma_cookie_t *used)
{
	struct nxc2600_dma_chan *chan = (struct nxc2600_dma_chan *)dchan;
	enum dma_status ret = DMA_IN_PROGRESS;
	spin_lock_irq(&(chan->finish_lock));
	
	if((chan->desc[chan->finish_desc].cookie>cookie) ||
	   (chan->desc[chan->finish_desc].len==0) ||
	   (chan->desc[chan->finish_desc].cookie>(cookie+MAX_DMA_DESCS)))
	{
		ret = DMA_SUCCESS;
	}
	spin_unlock_irq(&(chan->finish_lock));

	return ret;
}

static void
nxc2600_dma_issue_pending(struct dma_chan *dchan)
{
	struct nxc2600_dma_chan *chan = (struct nxc2600_dma_chan *)dchan;

        spin_lock_irq(&chan->lock);
	if(chan->last_used_desc == chan->current_desc)
                nxc2600_dma_start(chan,&chan->desc[chan->current_desc]);
        spin_unlock_irq(&chan->lock);

}

static irqreturn_t
nxc2600_dma_interrupt( int irq,
		       void *dev_id)
{
	int ndesc;
	struct nxc2600_dma_chan *chan = (struct nxc2600_dma_chan *)dev_id;

	nxc2600_dma_cleanup_interrupt(chan->ch);
	ndesc = chan->current_desc;
       	chan->desc[ndesc].cookie = 0;

        ndesc++;
       	if(ndesc>=MAX_DMA_DESCS)
               	ndesc=0;

        nxc2600_dma_start(chan,&chan->desc[ndesc]);
	chan->current_desc = ndesc;

	tasklet_schedule(&(chan->finish_task));

	return IRQ_HANDLED;
}


static int __devexit 
nxc2600_dma_remove(struct platform_device *dev)
{
	struct dma_device *device = platform_get_drvdata(dev);

	dma_async_device_unregister(device);

	free_irq(platform_get_irq(dev, 0), device);

	do {
		struct resource *res;
		res = platform_get_resource(dev, IORESOURCE_MEM, 0);
		release_mem_region(res->start, res->end - res->start);
	} while (0);

	kfree(device);

	return 0;
}

#define res_size(_r) (((_r)->end - (_r)->start) + 1)

static int __devinit 
nxc2600_dma_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret = 0;
	struct nxc2600_dma_chan *chan;
	struct dma_device *dma_dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	dma_dev = kzalloc(sizeof(*dma_dev), GFP_KERNEL);
	if (!dma_dev)
		return -ENOMEM;

	platform_set_drvdata(pdev, dma_dev);

	INIT_LIST_HEAD(&dma_dev->channels);

	/* set base routines */
	dma_cap_set(DMA_MEMCPY, dma_dev->cap_mask);
	dma_cap_set(DMA_MEMSET, dma_dev->cap_mask);
	//dma_cap_set(DMA_INTERRUPT, dma_dev->cap_mask);
	dma_dev->device_alloc_chan_resources = nxc2600_dma_alloc_chan_resources;
	dma_dev->device_free_chan_resources = nxc2600_dma_free_chan_resources;
	dma_dev->device_is_tx_complete = nxc2600_dma_is_complete;
	dma_dev->device_issue_pending = nxc2600_dma_issue_pending;
	dma_dev->device_dependency_added = nxc2600_dma_dependency_added;
	dma_dev->dev = &pdev->dev;
	dma_dev->device_prep_dma_memcpy = nxc2600_dma_prep_dma_memcpy;
  //      dma_dev->device_prep_dma_interrupt = nxc2600_dma_prep_dma_interrupt;
	dma_dev->device_prep_dma_memset = nxc2600_dma_prep_dma_memset;

	chan = kzalloc(sizeof(*chan), GFP_KERNEL);
	if (!chan) 
	{
		ret = -ENOMEM;
		goto err_free_dma;
	}

	chan->ch = platform_get_irq(pdev,0)-NXC2600_DMA_BASE_IRQ;
	chan->current_desc = 0;
	chan->finish_desc = 0;
	chan->last_used_desc = -1;

	chan->io_base = request_mem_region(res->start,
                                          res_size(res),
                                          "dma");

	if (!chan->io_base) 
	{
		ret = -ENOMEM;
		goto err_free_nxc2600_chan;
	}


	spin_lock_init(&chan->lock);
	spin_lock_init(&chan->finish_lock);
	INIT_RCU_HEAD(&chan->chan.rcu);
	chan->chan.device = dma_dev;

	ret = request_irq(platform_get_irq(pdev,0),
                                nxc2600_dma_interrupt, IRQF_DISABLED, pdev->name, chan);
        if (ret)
                goto err_free_nxc2600_chan;

	list_add_tail(&chan->chan.device_node, &dma_dev->channels);

	printk(KERN_INFO "NXC2600 DMA(%d) irq %d: "
	  "( %s%s%s%s%s%s%s%s%s%s)\n",
	chan->ch, platform_get_irq(pdev,0),
	  dma_has_cap(DMA_PQ_XOR, dma_dev->cap_mask) ? "pq_xor " : "",
	  dma_has_cap(DMA_PQ_UPDATE, dma_dev->cap_mask) ? "pq_update " : "",
	  dma_has_cap(DMA_PQ_ZERO_SUM, dma_dev->cap_mask) ? "pq_zero_sum " : "",
	  dma_has_cap(DMA_XOR, dma_dev->cap_mask) ? "xor " : "",
	  dma_has_cap(DMA_DUAL_XOR, dma_dev->cap_mask) ? "dual_xor " : "",
	  dma_has_cap(DMA_ZERO_SUM, dma_dev->cap_mask) ? "xor_zero_sum " : "",
	  dma_has_cap(DMA_MEMSET, dma_dev->cap_mask)  ? "fill " : "",
	  dma_has_cap(DMA_MEMCPY_CRC32C, dma_dev->cap_mask) ? "cpy+crc " : "",
	  dma_has_cap(DMA_MEMCPY, dma_dev->cap_mask) ? "cpy " : "",
	  dma_has_cap(DMA_INTERRUPT, dma_dev->cap_mask) ? "intr " : "");

        tasklet_init(&(chan->finish_task), nxc2600_dma_tasklet_finish,
                                (unsigned long) chan);

	dma_async_device_register(dma_dev);
	goto out;

 err_free_nxc2600_chan:
	kfree(chan);
 err_free_dma:
	kfree(dma_dev);
 out:
	return ret;
}

static struct resource nxc2600_dma_resources[][2] =
{
	{
		{
	        .start          = CPHYSADDR(NXC2600_DMA(0)),
	        .end            = CPHYSADDR(NXC2600_DMA(0)) + 0x20 - 1,
        	.flags          = IORESOURCE_MEM,
		},
		{
	        .start          = NXC2600_DMA_BASE_IRQ,
        	.end            = NXC2600_DMA_BASE_IRQ,
	        .flags          = IORESOURCE_IRQ,
		},
	},
	{
		{
                .start          = CPHYSADDR(NXC2600_DMA(1)),
                .end            = CPHYSADDR(NXC2600_DMA(1)) + 0x20 - 1,
                .flags          = IORESOURCE_MEM,
        	},
        	{
                .start          = NXC2600_DMA_BASE_IRQ+1,
                .end            = NXC2600_DMA_BASE_IRQ+1,
                .flags          = IORESOURCE_IRQ,
        	},
	},
	{
		{
                .start          = CPHYSADDR(NXC2600_DMA(2)),
                .end            = CPHYSADDR(NXC2600_DMA(2)) + 0x20 - 1,
                .flags          = IORESOURCE_MEM,
        	},
        	{
                .name           = nxc2600_dma_name,
                .start          = NXC2600_DMA_BASE_IRQ+2,
                .end            = NXC2600_DMA_BASE_IRQ+2,
                .flags          = IORESOURCE_IRQ,
        	},
	},
	{
		{
                .start          = CPHYSADDR(NXC2600_DMA(3)),
                .end            = CPHYSADDR(NXC2600_DMA(3)) + 0x20 - 1,
                .flags          = IORESOURCE_MEM,
        	},
        	{
                .start          = NXC2600_DMA_BASE_IRQ+3,
                .end            = NXC2600_DMA_BASE_IRQ+3,
                .flags          = IORESOURCE_IRQ,
        	},
	},
	{
		{
                .start          = CPHYSADDR(NXC2600_DMA(4)),
                .end            = CPHYSADDR(NXC2600_DMA(4)) + 0x20 - 1,
                .flags          = IORESOURCE_MEM,
        	},
        	{
                .start          = NXC2600_DMA_BASE_IRQ+4,
                .end            = NXC2600_DMA_BASE_IRQ+4,
                .flags          = IORESOURCE_IRQ,
        	},
	},
	{
		{
                .start          = CPHYSADDR(NXC2600_DMA(5)),
                .end            = CPHYSADDR(NXC2600_DMA(5)) + 0x20 - 1,
                .flags          = IORESOURCE_MEM,
        	},
        	{
                .start          = NXC2600_DMA_BASE_IRQ+5,
                .end            = NXC2600_DMA_BASE_IRQ+5,
                .flags          = IORESOURCE_IRQ,
		},
        },
#ifndef CONFIG_ICNEXUS_NXC2620
	{
		{
                .start          = CPHYSADDR(NXC2600_DMA(6)),
                .end            = CPHYSADDR(NXC2600_DMA(6)) + 0x20 - 1,
                .flags          = IORESOURCE_MEM,
 	        },
        	{
                .start          = NXC2600_DMA_BASE_IRQ+6,
                .end            = NXC2600_DMA_BASE_IRQ+6,
                .flags          = IORESOURCE_IRQ,
        	},
	},
	{
		{
                .start          = CPHYSADDR(NXC2600_DMA(7)),
                .end            = CPHYSADDR(NXC2600_DMA(7)) + 0x20 - 1,
                .flags          = IORESOURCE_MEM,
        	},
        	{
                .start          = NXC2600_DMA_BASE_IRQ+7,
                .end            = NXC2600_DMA_BASE_IRQ+7,
                .flags          = IORESOURCE_IRQ,
        	},
	},
#endif
};

static struct platform_device nxc2600_dma_dev[] =
{
	{
	        .name           = nxc2600_dma_name,
        	.id             = 0,
	        .num_resources  = ARRAY_SIZE(nxc2600_dma_resources[0]),
        	.resource       = nxc2600_dma_resources[0],
	},
	{
                .name           = nxc2600_dma_name,
                .id             = 1,
                .num_resources  = ARRAY_SIZE(nxc2600_dma_resources[1]),
                .resource       = nxc2600_dma_resources[1],
        },
	{
                .name           = nxc2600_dma_name,
                .id             = 2,
                .num_resources  = ARRAY_SIZE(nxc2600_dma_resources[2]),
                .resource       = nxc2600_dma_resources[2],
        },
	{
                .name           = nxc2600_dma_name,
                .id             = 3,
                .num_resources  = ARRAY_SIZE(nxc2600_dma_resources[3]),
                .resource       = nxc2600_dma_resources[3],
        },
	{
                .name           = nxc2600_dma_name,
                .id             = 4,
                .num_resources  = ARRAY_SIZE(nxc2600_dma_resources[4]),
                .resource       = nxc2600_dma_resources[4],
        },
	{
                .name           = nxc2600_dma_name,
                .id             = 5,
                .num_resources  = ARRAY_SIZE(nxc2600_dma_resources[5]),
                .resource       = nxc2600_dma_resources[5],
        },
#ifndef CONFIG_ICNEXUS_NXC2620
	{
                .name           = nxc2600_dma_name,
                .id             = 6,
                .num_resources  = ARRAY_SIZE(nxc2600_dma_resources[6]),
                .resource       = nxc2600_dma_resources[6],
        },
	{
                .name           = nxc2600_dma_name,
                .id             = 7,
                .num_resources  = ARRAY_SIZE(nxc2600_dma_resources[7]),
                .resource       = nxc2600_dma_resources[7],
	},
#endif
};

static struct platform_driver nxc2600_dma_driver =
{
	.probe		= nxc2600_dma_probe,
	.remove		= nxc2600_dma_remove,
	.driver		= {
		.name	= nxc2600_dma_name,
	},
};

static int __init 
nxc2600_dma_module_init (void)
{
	int i;
	int ret = platform_driver_register(&nxc2600_dma_driver);
	if(ret) return ret;
	for( i = 0; i< ARRAY_SIZE(nxc2600_dma_dev); i++)
	{
		ret = platform_device_register(&nxc2600_dma_dev[i]);
		if(ret) goto _dev_reg_err;
	}
	nxc2600_dma_init();

	return 0;

_dev_reg_err:
	for(i--;i>=0;i--)
		platform_device_unregister(&nxc2600_dma_dev[i]);
        platform_driver_unregister(&nxc2600_dma_driver);
	return ret;
}

static void __exit 
nxc2600_dma_module_exit (void)
{
	int i;
	for( i = 0; i< ARRAY_SIZE(nxc2600_dma_dev); i++)
		platform_device_unregister(&nxc2600_dma_dev[i]);
	platform_driver_unregister(&nxc2600_dma_driver);
}

module_init(nxc2600_dma_module_init);
module_exit(nxc2600_dma_module_exit);


MODULE_AUTHOR("Hardy Weng <hardy.weng@icnexus.com.tw>");
MODULE_DESCRIPTION("NXC2600 DMA Engine Driver");
MODULE_LICENSE("GPL");

