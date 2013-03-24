/*
 *
 * Copyright (C) 2007, 2008 IC Nexus Co., LTD.
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
 *   NXC2600 UDC
 *
 * 2008/06/20: initial
 *
 *
 * Author: Hardy Weng <hardy.weng@icnexus.com.tw>
 *
 */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/clk.h>
#include <linux/usb/ch9.h>
#include <linux/usb_gadget.h>

#include <asm/cacheflush.h>
#include <asm/mach-nxc2600/nxc2600.h>
#define	NUM_ENDPOINTS	8
struct nxc2600_udc;
struct nxc2600_ep 
{
        struct usb_ep                   ep;
        struct list_head                queue;
	int				nep;

        unsigned                        maxpacket:16;

        unsigned                        stopped:1;
        unsigned                        is_in:1;
        unsigned                        is_iso:1; 
	struct nxc2600_udc		*udc;
	const struct usb_endpoint_descriptor *desc;
};

struct nxc2600_udc
{
        struct usb_gadget               gadget;
        struct nxc2600_ep               ep[NUM_ENDPOINTS];
        struct usb_gadget_driver        *driver;
        unsigned                        enabled;
        unsigned                        suspended;
        unsigned                        req_pending;
        struct platform_device          *pdev;
};

static const char driver_name[] = "nxc2600_udc";

static __inline__ struct nxc2600_udc *
to_udc(struct usb_gadget *g)
{
        return container_of(g, struct nxc2600_udc, gadget);
}

struct nxc2600_request 
{
        struct usb_request              req;
        struct list_head                queue;
};


static const char ep0name[] = "ep0";

static void 
done(struct nxc2600_ep *ep, 
     struct nxc2600_request *req, 
     int status)
{
	unsigned	stopped = ep->stopped;

	list_del_init(&req->queue);
	if (req->req.status == -EINPROGRESS)
		req->req.status = status;
	else
		status = req->req.status;

	ep->stopped = 1;
	req->req.complete(&ep->ep, &req->req);
	ep->stopped = stopped;

}


static int 
read_fifo(struct nxc2600_ep *ep, 
	  struct nxc2600_request *req, 
	  u32 sr)
{
	u8		*buf;
	unsigned int	count, bufferspace, is_done;
	unsigned long flags;

	if ((sr&NXC2600_UDC_SR_OPS_MASK)!=NXC2600_UDC_SR_OPS_RECEIVED)
                return 0;

	count = NXC2600_UDC_SR_RPS(sr);
        if(!count)
                return 0;

	local_irq_save(flags);
	buf = req->req.buf + req->req.actual;
	bufferspace = req->req.length - req->req.actual;

	if (count > ep->ep.maxpacket)
		count = ep->ep.maxpacket;
	if (count > bufferspace) {
		pr_debug("%s buffer overflow\n", ep->ep.name);
		req->req.status = -EOVERFLOW;
		count = bufferspace;
	}
	nxc2600_udc_ep_read_data(ep->nep,buf,count);

	req->req.actual += count;
	is_done = (count < ep->ep.maxpacket);
	if (count == bufferspace)
		is_done = 1;

	if (is_done)
		done(ep, req, 0);

	local_irq_restore(flags);
	return is_done;
}

static int 
write_fifo(struct nxc2600_ep *ep, 
	   struct nxc2600_request *req)
{
	unsigned	total, count, is_last;
	unsigned long flags;

	local_irq_save(flags);
	total = req->req.length - req->req.actual;
	if (ep->ep.maxpacket < total) {
		count = ep->ep.maxpacket;
		is_last = 0;
	} else {
		count = total;
		is_last = (count < ep->ep.maxpacket) || !req->req.zero;
	}

	nxc2600_udc_ep_write_data(ep->nep,req->req.buf + req->req.actual,count);
	req->req.actual += count;

	if (is_last)
		done(ep, req, 0);

	local_irq_restore(flags);
	return is_last;
}

static void 
nuke(struct nxc2600_ep *ep, 
     int status)
{
	struct nxc2600_request *req;

	ep->stopped = 1;
	if (list_empty(&ep->queue))
		return;

	while (!list_empty(&ep->queue)) {
		req = list_entry(ep->queue.next, struct nxc2600_request, queue);
		done(ep, req, status);
	}
}


static int 
nxc2600_ep_enable(struct usb_ep *_ep,
		  const struct usb_endpoint_descriptor *desc)
{
	struct nxc2600_ep	*ep = container_of(_ep, struct nxc2600_ep, ep);
	struct nxc2600_udc	*dev = ep->udc;
	u16		maxpacket;
	u32		tmp;
	unsigned long	flags;

	if (!_ep || !ep
			|| !desc || ep->desc
			|| _ep->name == ep0name
			|| desc->bDescriptorType != USB_DT_ENDPOINT
			|| (maxpacket = le16_to_cpu(desc->wMaxPacketSize)) == 0
			|| maxpacket > ep->maxpacket) {
		pr_debug("bad ep or descriptor\n");
		return -EINVAL;
	}

	if (!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN) {
		pr_debug("bogus device state\n");
		return -ESHUTDOWN;
	}

	tmp = desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK;
	switch (tmp) {
	case USB_ENDPOINT_XFER_CONTROL:
		pr_debug("only one control endpoint\n");
		return -EINVAL;
	case USB_ENDPOINT_XFER_INT:
		if (maxpacket > 64)
			goto bogus_max;
		break;
	case USB_ENDPOINT_XFER_BULK:
		if(maxpacket==64)
			goto ok;
bogus_max:
		pr_debug("bogus maxpacket %d\n", maxpacket);
		return -EINVAL;
	case USB_ENDPOINT_XFER_ISOC:
		return -EINVAL;
		break;
	}

ok:
	local_irq_save(flags);

	/* initialize endpoint to match this descriptor */
	ep->is_in = (desc->bEndpointAddress & USB_DIR_IN) != 0;
	ep->is_iso = (tmp == USB_ENDPOINT_XFER_ISOC);
	ep->stopped = 0;
	ep->desc = desc;
        ep->ep.maxpacket = maxpacket;
	local_irq_restore(flags);
	return 0;
}

static int 
nxc2600_ep_disable (struct usb_ep * _ep)
{
	struct nxc2600_ep	*ep = container_of(_ep, struct nxc2600_ep, ep);

	unsigned long	flags;
	if (ep == &ep->udc->ep[0])
		return -EINVAL;

	local_irq_save(flags);
	nuke(ep, -ESHUTDOWN);

	ep->desc = NULL;
	ep->ep.maxpacket = ep->maxpacket;
	ep->stopped = 1;
	
	local_irq_restore(flags);
	return 0;
}

static struct usb_request *
nxc2600_ep_alloc_request(struct usb_ep *_ep, 
			 unsigned int gfp_flags)
{
	struct nxc2600_request *req;

	req = kzalloc(sizeof (struct nxc2600_request), gfp_flags);
	if (!req)
		return NULL;
	INIT_LIST_HEAD(&req->queue);
	return &req->req;
}

static void 
nxc2600_ep_free_request(struct usb_ep *_ep, 
			struct usb_request *_req)
{
	struct nxc2600_request *req;

	req = container_of(_req, struct nxc2600_request, req);
	BUG_ON(!list_empty(&req->queue));
	kfree(req);
}

static int 
nxc2600_ep_queue(struct usb_ep *_ep,
		 struct usb_request *_req, 
		 gfp_t gfp_flags)
{
	struct nxc2600_request	*req;
	struct nxc2600_ep	*ep;
	struct nxc2600_udc	*udc;
	int			status =0;
	unsigned long		flags;

	req = container_of(_req, struct nxc2600_request, req);
	ep = container_of(_ep, struct nxc2600_ep, ep);
	if (!_req || !_req->complete
			|| !_req->buf || !list_empty(&req->queue)) {
		pr_debug("invalid request\n");
		return -EINVAL;
	}
	udc = ep->udc;

	if (!udc || !udc->driver || udc->gadget.speed == USB_SPEED_UNKNOWN) {
		pr_debug("invalid device\n");
		return -EINVAL;
	}

	_req->status = -EINPROGRESS;
	_req->actual = 0;

	local_irq_save(flags);

	if (ep->is_in)
	{
		if(REG32(NXC2600_UDC_IMR)&NXC2600_UDC_IMR_IIM(ep->nep))
		{
		u32 sr = REG32(NXC2600_UDC_ISR(ep->nep));
		if(sr&NXC2600_UDC_SR_IPS)
		{
			REG32(NXC2600_UDC_ISR(ep->nep)) &= ~NXC2600_UDC_SR_IPS;
			status = write_fifo(ep, req);
		}
		else status =0;	
		}
	}
	else
	{
		if(REG32(NXC2600_UDC_IMR)&NXC2600_UDC_IMR_OIM(ep->nep))
		{
		u32 sr = REG32(NXC2600_UDC_OSR(ep->nep));
		if((sr&NXC2600_UDC_SR_OPS_MASK)==NXC2600_UDC_SR_OPS_RECEIVED)
		{
			REG32(NXC2600_UDC_OSR(ep->nep)) &= ~NXC2600_UDC_SR_OPS_MASK;
			status = read_fifo(ep, req, sr);
		}
		else status = 0;
		}
	}

	if (req && !status) {

		list_add_tail (&req->queue, &ep->queue);

	}

	if (ep->is_in)
		REG32(NXC2600_UDC_IMR) &= ~NXC2600_UDC_IMR_IIM(ep->nep);
	else
		REG32(NXC2600_UDC_IMR) &= ~NXC2600_UDC_IMR_OIM(ep->nep);
	local_irq_restore(flags);
	return (status < 0) ? status : 0;
}

static int 
nxc2600_ep_dequeue(struct usb_ep *_ep,
		   struct usb_request *_req)
{
	struct nxc2600_ep	*ep;
	struct nxc2600_request	*req;
	unsigned long flags;

	ep = container_of(_ep, struct nxc2600_ep, ep);
	if (!_ep || ep->ep.name == ep0name)
		return -EINVAL;
	local_irq_save(flags);
	list_for_each_entry (req, &ep->queue, queue) {
		if (&req->req == _req)
			break;
	}
	if (&req->req != _req)
		return -EINVAL;

	done(ep, req, -ECONNRESET);
	local_irq_restore(flags);
	return 0;
}

static int 
nxc2600_ep_set_halt(struct usb_ep *_ep, 
		    int value)
{
	struct nxc2600_ep	*ep = container_of(_ep, struct nxc2600_ep, ep);
	unsigned long	flags;
	int		status = 0;

	if (!_ep || ep->is_iso)
		return -EINVAL;

	local_irq_save(flags);

	if (ep->is_in && (!list_empty(&ep->queue)))
		status = -EAGAIN;
	else {
		if (value) {
			ep->stopped = 1;
		} else {
			ep->stopped = 0;
		}
	}

	local_irq_restore(flags);
	return status;
}

static const struct usb_ep_ops nxc2600_ep_ops = 
{
	.enable		= nxc2600_ep_enable,
	.disable	= nxc2600_ep_disable,
	.alloc_request	= nxc2600_ep_alloc_request,
	.free_request	= nxc2600_ep_free_request,
	.queue		= nxc2600_ep_queue,
	.dequeue	= nxc2600_ep_dequeue,
	.set_halt	= nxc2600_ep_set_halt,
};

static int 
nxc2600_get_frame(struct usb_gadget *gadget)
{
	return -EOPNOTSUPP;
}

static void 
udc_reinit(struct nxc2600_udc *udc)
{
	u32 i;

	nxc2600_udc_init();

	INIT_LIST_HEAD(&udc->gadget.ep_list);
	INIT_LIST_HEAD(&udc->gadget.ep0->ep_list);

	for (i = 0; i < NUM_ENDPOINTS; i++) {
		struct nxc2600_ep *ep = &udc->ep[i];

		if (i != 0)
			list_add_tail(&ep->ep.ep_list, &udc->gadget.ep_list);
		ep->desc = NULL;
		ep->stopped = 1;
		ep->ep.maxpacket = ep->maxpacket;
		ep->nep = i;
		INIT_LIST_HEAD(&ep->queue);
	}
	udc->ep[0].stopped = 0;
}

static void 
stop_activity(struct nxc2600_udc *udc)
{
	struct usb_gadget_driver *driver = udc->driver;
	int i;

	REG32(NXC2600_UDC_DIPR) = 0xffffffff;
	REG32(NXC2600_UDC_IPR) = 0xffffffff;
	if (udc->gadget.speed == USB_SPEED_UNKNOWN)
		driver = NULL;
	udc->gadget.speed = USB_SPEED_UNKNOWN;
	udc->suspended = 0;


	for (i = 0; i < NUM_ENDPOINTS; i++) {
		struct nxc2600_ep *ep = &udc->ep[i];
		ep->stopped = 1;
		nuke(ep, -ESHUTDOWN);
	}
	if (driver)
		driver->disconnect(&udc->gadget);

	udc_reinit(udc);
}

static int 
nxc2600_pullup(struct usb_gadget *gadget, 
		int is_on)
{
	struct nxc2600_udc	*udc = to_udc(gadget);
	unsigned long	flags;

	local_irq_save(flags);
	udc->enabled = is_on = !!is_on;
	stop_activity(udc);
	local_irq_restore(flags);
	return 0;
}
static const struct usb_gadget_ops nxc2600_udc_ops = {
	.get_frame		= nxc2600_get_frame,
};


static int 
handle_ep(struct nxc2600_ep *ep,
	  u32 stats)
{
	struct nxc2600_request	*req;

	if (!list_empty(&ep->queue))
		req = list_entry(ep->queue.next,
			struct nxc2600_request, queue);
	else
		req = NULL;

	if (ep->is_in) {
		if (req)
			return (write_fifo(ep, req)<0)? -1:0;

	} else {
		if (req)
			return (read_fifo(ep, req, stats)<0)? -1:0;
	}
	return -1;
}

union setup {
	u32			raw[2];
	struct usb_ctrlrequest	r;
};

static int 
handle_setup(struct nxc2600_udc *udc, 
	     struct nxc2600_ep *ep, 
	     u32 sr)
{
	u32		tmp;
	union setup	pkt;
	int		status = 0;

	if ((sr&NXC2600_UDC_SR_OPS_MASK)==NXC2600_UDC_SR_OPS_SETUP) {
		pkt.raw[0] = REG32(NXC2600_UDC_RXFIFO);
		pkt.raw[1] = REG32(NXC2600_UDC_RXFIFO);
		tmp = REG32(NXC2600_UDC_TCFM);
		if (pkt.r.bRequestType & USB_DIR_IN) {
			ep->is_in = 1;
		} else {
			ep->is_in = 0;
		}
	} else {
		pr_debug("not SETUP command\n");
		status = -EINVAL;
	}
	ep->stopped = 0;
	if (unlikely(status != 0))
		goto stall;

	udc->req_pending = 1;

	if (udc->driver)
		status = udc->driver->setup(&udc->gadget, &pkt.r);
	else
		status = -ENODEV;
	if (status < 0) {
stall:
		udc->req_pending = 0;
	}
	return status;
}

static int 
handle_ep0_out(struct nxc2600_udc *udc, 
		u32 status)
{
	struct nxc2600_ep	*ep0 = &udc->ep[0];
	struct nxc2600_request	*req;
	
	if ((status &NXC2600_UDC_SR_OPS_MASK)== NXC2600_UDC_SR_OPS_SETUP) {
		nuke(ep0, 0);
		udc->req_pending = 0;
		return handle_setup(udc, ep0,status);
	}
	else if((status&NXC2600_UDC_SR_OPS_MASK)==NXC2600_UDC_SR_OPS_RECEIVED)
	{
		u32 count;
		count = NXC2600_UDC_SR_RPS(status);
                if (count == 0) {
                        REG32(NXC2600_UDC_TCFM); // ack zero packet
                }
                else {
                        /* EP0 OUT Data */
                        if (list_empty(&ep0->queue)) {
                                udc->req_pending = 1;
				return -1;
                        }
                        else
			{
				req = list_entry(ep0->queue.next, struct nxc2600_request, queue);
				if(write_fifo(ep0,req)<0)
					return -1;
			}
                }
		return 0;
	}
	return -1;
}

/*
 * Simulate a USB_REQ_SET_CONFIGURATION to the function driver,
 * this is required to enable the endpoints of the function driver.
 * UDC should let software have the chance to handle this standard
 * request, unfortunately UDC can't do that.
 */
static void 
psudo_set_config( struct nxc2600_udc *udc )
{
        struct usb_ctrlrequest ctrl;
	struct nxc2600_ep *ep0 = &udc->ep[0];
        int tmp;

        /* SETUP packet */
        ctrl.bRequestType = USB_RECIP_DEVICE;
        ctrl.bRequest = USB_REQ_SET_CONFIGURATION;
        ctrl.wValue = 1;
        ctrl.wIndex = 0;
        ctrl.wLength = 0;

        nuke(ep0, 0);
        ep0->stopped = 0;

        if (likely(ctrl.bRequestType & USB_DIR_IN)) {
                ep0->is_in = 1;
        } else {
                ep0->is_in = 0;
        }

        tmp = udc->driver->setup(&udc->gadget, &ctrl);
        if (unlikely(tmp < 0)) {
                ep0->stopped = 1;
        }
}

static irqreturn_t 
nxc2600_udc_irq (int irq, 
		 void *_udc)
{
	struct nxc2600_udc		*udc = _udc;
	u32 epint = REG32(NXC2600_UDC_IPR);
	u32 status = REG32(NXC2600_UDC_DIPR);

	if (status)
	{
		if(status&NXC2600_UDC_DIPR_SC)
		{
			psudo_set_config(udc);	
			udelay(100);
		}
		if(status&NXC2600_UDC_DIPR_URST)
		{
			stop_activity(udc);
                	udc->gadget.speed = USB_SPEED_FULL;
	                udc->suspended = 0;

		}
		REG32(NXC2600_UDC_DIPR) = status;
	}
	if(!epint)
		return IRQ_HANDLED;
	if ( epint & NXC2600_UDC_IPR_OIP(0))
	{
		u32 stat = REG32(NXC2600_UDC_OSR(0));	
		REG32(NXC2600_UDC_IPR) = NXC2600_UDC_IPR_OIP(0);
                if((stat&NXC2600_UDC_SR_OPS_MASK))
                {
			if(!handle_ep0_out(udc,stat))
				REG32(NXC2600_UDC_OSR(0)) = 0;
			else
				REG32(NXC2600_UDC_IMR) |= NXC2600_UDC_IMR_OIM(0);
		}
	}
	if (epint & NXC2600_UDC_IPR_IIP(0))
        {
		u32 stat = REG32(NXC2600_UDC_ISR(0));
		REG32(NXC2600_UDC_IPR) = NXC2600_UDC_IPR_IIP(0);
		if(stat&NXC2600_UDC_SR_IPS)
		{
			if(!handle_ep(&udc->ep[0],stat))
				REG32(NXC2600_UDC_ISR(0)) = 0;
			else
				REG32(NXC2600_UDC_IMR) |= NXC2600_UDC_IMR_IIM(0);
		}
        }
	if (epint & NXC2600_UDC_IPR_OIP(5))
        {
                u32 stat = REG32(NXC2600_UDC_OSR(5));
                REG32(NXC2600_UDC_IPR) = NXC2600_UDC_IPR_OIP(5);
                if((stat&NXC2600_UDC_SR_OPS_MASK)==NXC2600_UDC_SR_OPS_RECEIVED)
                {
                        if(!handle_ep(&udc->ep[5],stat))
				REG32(NXC2600_UDC_OSR(5)) = 0;
			else
				REG32(NXC2600_UDC_IMR) |= NXC2600_UDC_IMR_OIM(5);
                }
        }
        if (epint & NXC2600_UDC_IPR_OIP(6))
        {
                u32 stat = REG32(NXC2600_UDC_OSR(6));
                REG32(NXC2600_UDC_IPR) = NXC2600_UDC_IPR_OIP(6);
                if((stat&NXC2600_UDC_SR_OPS_MASK)==NXC2600_UDC_SR_OPS_RECEIVED)
                {
                        if(!handle_ep(&udc->ep[6],stat))
				REG32(NXC2600_UDC_OSR(6)) = 0;
			else
				REG32(NXC2600_UDC_IMR) |= NXC2600_UDC_IMR_OIM(6);
                }
        }
	if (epint & NXC2600_UDC_IPR_IIP(2))
        {
		u32 stat = REG32(NXC2600_UDC_ISR(2));
		REG32(NXC2600_UDC_IPR) = NXC2600_UDC_IPR_IIP(2);
                if(stat&NXC2600_UDC_SR_IPS)
                {
                	if(!handle_ep(&udc->ep[2],stat))
				REG32(NXC2600_UDC_ISR(2)) = 0;
			else
				REG32(NXC2600_UDC_IMR) |= NXC2600_UDC_IMR_IIM(2);
		}
        }
        if (epint & NXC2600_UDC_IPR_IIP(3))
        {
		u32 stat = REG32(NXC2600_UDC_ISR(3));
		REG32(NXC2600_UDC_IPR) = NXC2600_UDC_IPR_IIP(3);
                if(stat&NXC2600_UDC_SR_IPS)
                {
                	if(!handle_ep(&udc->ep[3],stat))
				REG32(NXC2600_UDC_ISR(3)) = 0;
			else
				REG32(NXC2600_UDC_IMR) |= NXC2600_UDC_IMR_IIM(3);
		}
        }
	return IRQ_HANDLED;
}


static void 
nop_release(struct device *dev)
{
	/* nothing to free */
}

static struct nxc2600_udc controller = {
	.gadget = {
		.ops	= &nxc2600_udc_ops,
		.ep0	= &controller.ep[0].ep,
		.name	= driver_name,
		.is_dualspeed	= 0,
		.dev	= {
			.bus_id = "gadget",
			.release = nop_release,
		}
	},
	.ep[0] = {
		.ep = {
			.name	= ep0name,
			.ops	= &nxc2600_ep_ops,
		},
		.udc		= &controller,
		.maxpacket	= 32,
	},
	.ep[1] = {
		.ep = {
			.name	= "ep1-int",
			.ops	= &nxc2600_ep_ops,
		},
		.udc		= &controller,
		.maxpacket	= 64,
	},
	.ep[2] = {
		.ep = {
			.name	= "ep2in-bulk",
			.ops	= &nxc2600_ep_ops,
		},
		.udc		= &controller,
		.maxpacket	= 64,
	},
	.ep[3] = {
		.ep = {
			.name	= "ep3in-bluk",
			.ops	= &nxc2600_ep_ops,
		},
		.udc		= &controller,
		.maxpacket	= 64,
	},
	.ep[4] = {
		.ep = {
			.name	= "ep4in-iso",
			.ops	= &nxc2600_ep_ops,
		},
		.udc		= &controller,
		.maxpacket	= 64,
	},
	.ep[5] = {
		.ep = {
			.name	= "ep5out-bluk",
			.ops	= &nxc2600_ep_ops,
		},
		.udc		= &controller,
		.maxpacket	= 64,
	},
        .ep[6] = {
                .ep = {
                        .name   = "ep6out-bluk",
                        .ops    = &nxc2600_ep_ops,
                },
                .udc            = &controller,
                .maxpacket      = 64,
        },
        .ep[7] = {
                .ep = {
                        .name   = "ep7out-iso",
                        .ops    = &nxc2600_ep_ops,
                },
                .udc            = &controller,
                .maxpacket      = 64,
        },

};

int 
usb_gadget_register_driver(struct usb_gadget_driver *driver)
{
	struct nxc2600_udc	*udc = &controller;
	int		retval;

	if (!driver
			|| driver->speed < USB_SPEED_FULL
			|| !driver->bind
			|| !driver->setup) {
		printk( "bad parameter.\n");
		return -EINVAL;
	}

	if (udc->driver) {
		printk("UDC already has a gadget driver\n");
		return -EBUSY;
	}
	udc->driver = driver;
	udc->gadget.dev.driver = &driver->driver;
	udc->gadget.dev.driver_data = &driver->driver;
	udc->enabled = 1;

	retval = driver->bind(&udc->gadget);
	if (retval) {
		printk("driver->bind() returned %d\n", retval);
		udc->driver = NULL;
		udc->gadget.dev.driver = NULL;
		udc->gadget.dev.driver_data = NULL;
		udc->enabled = 0;
		return retval;
	}

	local_irq_disable();
	nxc2600_pullup(&(udc->gadget), 1);
	local_irq_enable();

	printk("bound to %s\n", driver->driver.name);
	return 0;
}
EXPORT_SYMBOL (usb_gadget_register_driver);

int 
usb_gadget_unregister_driver (struct usb_gadget_driver *driver)
{
	struct nxc2600_udc *udc = &controller;
	if (!driver || driver != udc->driver || !driver->unbind)
		return -EINVAL;
	local_irq_disable();
	udc->enabled = 0;
	nxc2600_pullup(&udc->gadget, 0);
	local_irq_enable();

	driver->unbind(&udc->gadget);
	udc->driver = NULL;

	printk("unbound from %s\n", driver->driver.name);
	return 0;
}
EXPORT_SYMBOL (usb_gadget_unregister_driver);

static int __init 
nxc2600_udc_probe(struct platform_device *pdev)
{
	struct device	*dev = &pdev->dev;
	struct nxc2600_udc	*udc;
	int		retval;

	udc = &controller;
	udc->gadget.dev.parent = dev;
	udc->pdev = pdev;
	udc->enabled = 0;

	udc_reinit(udc);

	retval = device_register(&udc->gadget.dev);
	if (retval < 0)
		goto fail0;

	REG32(NXC2600_SBA_CNTL) &= ~NXC2600_SBA_CNTL_USB_PORT0_HOST;

	if (request_irq(NXC2600_IRQ_UDC, nxc2600_udc_irq,
			IRQF_DISABLED, driver_name, udc)) {
		printk("request irq %d failed\n", NXC2600_IRQ_UDC);
		retval = -EBUSY;
		goto fail1;
	}

	dev_set_drvdata(dev, udc);
	device_init_wakeup(dev, 1);

	printk("%s initialed\n", driver_name);
	return 0;

fail1:
	device_unregister(&udc->gadget.dev);
fail0:
	printk("%s probe failed, %d\n", driver_name, retval);
	return retval;
}

static int __exit 
nxc2600_udc_remove(struct platform_device *pdev)
{
	struct nxc2600_udc *udc = platform_get_drvdata(pdev);

	if (udc->driver)
		return -EBUSY;

	nxc2600_pullup(&(udc->gadget), 0);

	device_init_wakeup(&pdev->dev, 0);
	free_irq(NXC2600_IRQ_UDC, udc);
	device_unregister(&udc->gadget.dev);
	REG32(NXC2600_UDC_DIMR) = 0xffffffff;
	REG32(NXC2600_UDC_IMR) = 0xffffffff;
	REG32(NXC2600_UDC_IPR) = 0xffffffff;
	REG32(NXC2600_UDC_DIPR) = 0xffffffff;
	return 0;
}

static struct platform_driver nxc2600_udc_driver = 
{
	.remove		= __exit_p(nxc2600_udc_remove),
	.driver		= {
		.name	= driver_name,
		.owner	= THIS_MODULE,
	},
};

static int __init 
udc_init_module(void)
{
	return platform_driver_probe(&nxc2600_udc_driver, nxc2600_udc_probe);
}
module_init(udc_init_module);

static void __exit 
udc_exit_module(void)
{
	platform_driver_unregister(&nxc2600_udc_driver);
}
module_exit(udc_exit_module);

MODULE_DESCRIPTION("NXC2600 udc driver");
MODULE_AUTHOR("Hardy Weng <hardy.weng@icnexus.com.tw>");
MODULE_LICENSE("GPL");
