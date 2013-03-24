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
#include <linux/ioport.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/rtnetlink.h>
#include <linux/delay.h>
#include <linux/ethtool.h>
#include <linux/mii.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>

#include <asm/cacheflush.h>
#include <asm/mach-nxc2600/nxc2600.h>

static char mac_name[] = {"NXC2620_MAC"};

struct MAC_DRV
{
	struct NXC2620_MAC_DMA_DESC	*desc;
	dma_addr_t	vaddr;
	uint8_t		*regs;
	spinlock_t	lock;
	struct net_device       *dev;
        struct resource         *irq;
        struct resource         *ioarea;
	struct mii_if_info 	mii;
	struct net_device_stats stats;
	struct timer_list media_checker;

};

void*
nxc2620_mac_alloc_buffer( void **data,
			  uint32_t size )
{
	struct sk_buff *skb = dev_alloc_skb( size+2 );
	if( skb == NULL )
	{
		printk("nxc2620 mac: out of memory\n");
		return NULL;
	}
	skb_reserve(skb, 2);

	if(data!=NULL) *data = skb->data;
	return skb;
}

void
nxc2620_mac_free_buffer( void *buf )
{
	if(buf != NULL)
		 dev_kfree_skb_irq((struct sk_buff *)buf);
}


int
nxc2620_mac_rx( void *priv )
{
	struct MAC_DRV *mac = (struct MAC_DRV*)priv;
	struct sk_buff *skb;
	uint32_t len;

	skb = (struct sk_buff *)(mac->desc->rx_buffer[mac->desc->rx_head]);
	len = mac->desc->rx[mac->desc->rx_head].fl-4;
	skb->dev = mac->dev;

	skb_put( skb, len );
	skb->protocol = eth_type_trans(skb, mac->dev);

	netif_rx(skb);
	mac->dev->last_rx = jiffies;
	mac->stats.rx_packets++;
	mac->stats.rx_bytes += len;

	return 0;
}

int
nxc2620_mac_tx( void *priv,
		uint32_t slot,
		uint32_t len )
{
	struct MAC_DRV *mac = (struct MAC_DRV*)priv;

	mac->stats.tx_packets++;
        mac->stats.tx_bytes += len;
	netif_start_queue(mac->dev);
	return 0;
}

static int 
mdio_read(struct net_device *ndev, 
	  int phy_id, 
	  int location)
{
	return nxc2620_mac_mdio_read( phy_id, location );

}

static void
mdio_write(struct net_device *ndev, 
	   int phy_id, 
	   int location, 
	   int val)
{
	nxc2620_mac_mdio_write( phy_id, location, val );
}

static int
phy_reset( struct net_device *ndev )
{
	struct MAC_DRV *mac = netdev_priv(ndev);
	struct ethtool_cmd ecmd;
	int timeout;

	mdio_write(ndev, mac->mii.phy_id, MII_BMCR, BMCR_RESET);
        while(mdio_read(ndev, mac->mii.phy_id, MII_BMCR) & BMCR_RESET) udelay(100);

        ecmd.speed=SPEED_100;
        ecmd.duplex=DUPLEX_FULL;
        ecmd.port=PORT_MII;
        ecmd.transceiver=XCVR_INTERNAL;
        ecmd.phy_address=mac->mii.phy_id;
        ecmd.autoneg=AUTONEG_ENABLE;

        mii_ethtool_sset(&(mac->mii), &ecmd);
	printk("phy: waitting for autonegotiation complete...\n");
	for( timeout = 0; timeout < 5000; timeout++)
	{
		if(mdio_read(ndev,mac->mii.phy_id, MII_BMSR) & BMSR_ANEGCOMPLETE)
		{
			printk("phy: autonegotiation complete.\n");
			mii_check_media(&(mac->mii),1,1);
			nxc2620_mac_change_mode( mac->mii.full_duplex, mac->mii.advertising & (ADVERTISE_100FULL|ADVERTISE_100HALF));
			return 0;
		}
		udelay(1000);
        }
	printk("phy: timeout, autonegotiation failed.\n");
	return -1;
}

static void
mac_media_checker(unsigned long data)
{
        struct net_device *ndev = (struct net_device *) data;
	struct MAC_DRV *mac = netdev_priv(ndev);

	if(mii_check_media(&(mac->mii),1,0))
		nxc2620_mac_change_mode( mac->mii.full_duplex, mac->mii.advertising & (ADVERTISE_100FULL|ADVERTISE_100HALF) );
	else if((!mii_link_ok(&(mac->mii))) && (mac->mii.advertising == ADVERTISE_RFAULT))
		nxc2620_mac_change_mode( 0, 0 );
        mac->media_checker.expires = jiffies+HZ;
        add_timer(&(mac->media_checker));
}

static irqreturn_t 
mac_interrupt ( int irq, 
		void *dev_instance)
{
	struct net_device *ndev = (struct net_device *) dev_instance;
        struct MAC_DRV *mac = netdev_priv(ndev);

	nxc2620_mac_dma_poll( mac, mac->desc );

	return IRQ_HANDLED;
}

static int
mac_init(struct net_device *dev)
{
	return 0;
}

static int
mac_open( struct net_device *ndev )
{
	struct MAC_DRV *mac = netdev_priv(ndev);
	int ret;

	ret = request_irq( ndev->irq, mac_interrupt, IRQF_DISABLED, ndev->name, ndev);
	if( ret )
	{
		printk("nxc2620 mac: request irq failed.\n");
		return ret;
	}

	phy_reset(ndev);

	nxc2620_mac_init();

	nxc2620_mac_dma_desc_init( mac->desc );

	netif_start_queue(ndev);

	init_timer(&(mac->media_checker));
	mac->media_checker.expires  = jiffies+HZ;
	mac->media_checker.data     = (unsigned long) ndev;
	mac->media_checker.function = mac_media_checker;
        add_timer(&(mac->media_checker));

	return 0;
}

static int
mac_start_xmit( struct sk_buff *skb, 
		struct net_device *ndev)
{
	struct MAC_DRV *mac = netdev_priv(ndev);
	int slot;
	int length;

	length = (skb->len<ETH_ZLEN)? ETH_ZLEN:skb->len;
	spin_lock_irq(&(mac->lock));
	slot = nxc2620_mac_send( mac->desc, skb->data, length);
	if(slot < 0) 
		netif_stop_queue (ndev);
	else 
		mac->desc->tx_buffer[slot] = skb;
	ndev->trans_start = jiffies;
	spin_unlock_irq(&(mac->lock));

	return 0;
}

static int
mac_stop( struct net_device *ndev)
{
	struct MAC_DRV *mac = netdev_priv(ndev);

	spin_lock_irq(&(mac->lock));
	netif_stop_queue(ndev);
	free_irq( ndev->irq, ndev);
	del_timer(&(mac->media_checker));
	nxc2620_mac_stop();
	nxc2620_mac_dma_desc_free( mac->desc);
	spin_unlock_irq(&(mac->lock));
	return 0;
}

static void
mac_tx_timeout(struct net_device *ndev)
{
        struct MAC_DRV *mac = netdev_priv(ndev);

        spin_lock_irq(&(mac->lock));
	netif_stop_queue(ndev);
	nxc2620_mac_stop();
	nxc2620_mac_dma_desc_free( mac->desc);

        phy_reset(ndev);
        nxc2620_mac_init();
        nxc2620_mac_dma_desc_init( mac->desc );
	ndev->trans_start = jiffies;
	netif_wake_queue(ndev);
        spin_unlock_irq(&(mac->lock));
}

static struct net_device_stats *
mac_get_stats( struct net_device *ndev)
{
	struct MAC_DRV *mac = netdev_priv(ndev);
	return &(mac->stats);
}

#define res_size(_r) (((_r)->end - (_r)->start) + 1)
static int
mac_probe(struct device *dev)
{
        struct platform_device *pdev = to_platform_device(dev);
        struct resource *res;
	struct net_device *ndev;
        int ret = 0;
        int i;
	int phy_id;
	struct MAC_DRV *mac;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
        if(res == NULL)
        {
                printk(KERN_ERR "NXC2620 MAC: no IO resource.\n");
                return -ENOENT;
        }

	if( res->start != NXC2620_MAC )
	{
		return -ENOENT;
	}

	nxc2620_mac_init();

	phy_id = nxc2620_mac_search_phy_id();
	if(phy_id<0)
	{
		printk("nxc2620 mac: no phy connected.\n");
		return -ENXIO;
	}

	ndev = alloc_etherdev(sizeof(struct MAC_DRV));
	if(!ndev)
	{
                printk("nxc2620 mac: could not allocate device.\n");
                return -ENOMEM;
        }

	mac = netdev_priv(ndev);

	mac->desc = kmalloc(sizeof(struct NXC2620_MAC_DMA_DESC),
			  GFP_KERNEL);
	if( !mac->desc )
	{
		printk("nxc2600 mac: out of memory.\n");
		ret = -ENOMEM;
		goto out;
	}
	
	mac->dev = ndev;

	ndev->base_addr = res->start;
	ndev->irq = NXC2600_IRQ_ETH;

        mac->ioarea = request_mem_region(res->start,
                                          res_size(res),
                                          "mac");

        if (mac->ioarea == NULL) 
	{
                printk(KERN_ERR "NXC2620 MAC: request IO memory failed.\n");
                ret = -ENXIO;
                goto out;
        }

        mac->regs = (uint8_t*)ioremap(res->start,  res_size(res));

        if (mac->regs == NULL) {
                printk(KERN_ERR "NXC2620 MAC: map IO failed.\n");
                ret = -ENXIO;
                goto out;
        }

        spin_lock_init(&(mac->lock));

	SET_MODULE_OWNER(ndev);
        SET_NETDEV_DEV(ndev, &pdev->dev);

	nxc2620_mac_get_addr(ndev->dev_addr);

	ether_setup(ndev);

	ndev->irq		 = NXC2600_IRQ_ETH;
	ndev->init		 = mac_init;
	ndev->open               = mac_open;
        ndev->hard_start_xmit    = mac_start_xmit;
        ndev->tx_timeout         = mac_tx_timeout;
        ndev->watchdog_timeo	 = 6*HZ;
        ndev->stop               = mac_stop;
        ndev->get_stats          = mac_get_stats;
	mac->mii.phy_id = phy_id;
        mac->mii.phy_id_mask  = 0x1f;
        mac->mii.reg_num_mask = 0x1f;
        mac->mii.force_media  = 0;
	mac->mii.supports_gmii = 0;
        mac->mii.full_duplex  = 0;
        mac->mii.dev          = ndev;
        mac->mii.mdio_read    = mdio_read;
        mac->mii.mdio_write   = mdio_write;



	platform_set_drvdata(pdev, ndev);
        ret = register_netdev(ndev);

        if (ret == 0)
	{
                printk("%s: NXC2620 MAC at %p IRQ %d PHY %d MAC: ",
                       ndev->name,  mac->ioarea, ndev->irq, mac->mii.phy_id );
                for (i = 0; i < 5; i++)
                        printk("%02x:", ndev->dev_addr[i]);
                printk("%02x\n", ndev->dev_addr[5]);
		return 0;
        }

	platform_set_drvdata(pdev, NULL);
out:

	free_netdev(ndev);
	return -ENXIO;

}

static int
mac_remove(struct device *dev)
{
        dev_set_drvdata(dev, NULL);
        return 0;
}



static struct resource nxc2620_mac_resource =
{
        .name = mac_name,
        .start = NXC2620_MAC,
        .end   = NXC2620_MAC + 0x30 - 1,
        .flags = IORESOURCE_MEM
};

static struct platform_device nxc2620_mac_device =
{
        .name  = mac_name,
        .id    = 0,
        .num_resources = 1,
        .resource = &nxc2620_mac_resource
};


static struct device_driver nxc2620_mac_driver =
{
        .name           = mac_name,
        .bus            = &platform_bus_type,
        .probe          = mac_probe,
        .remove         = mac_remove,
};

static int __init
nxc2620_mac_module_init(void)
{
	int err;
        err = driver_register(&nxc2620_mac_driver);
        if(err) return err;

        platform_device_register(&nxc2620_mac_device);

        return 0;

}

static void __exit
nxc2620_mac_module_cleanup(void)
{
	platform_device_unregister(&nxc2620_mac_device);
        driver_unregister(&nxc2620_mac_driver);
}

module_init(nxc2620_mac_module_init);
module_exit(nxc2620_mac_module_cleanup);



MODULE_DESCRIPTION("NXC2620 On-Chip Ethernet Driver");
MODULE_AUTHOR("Hardy Weng <hardy.weng@icnexus.com.tw>");
MODULE_LICENSE("GPL");

