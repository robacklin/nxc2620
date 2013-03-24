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
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/pci.h>
#include <linux/pm.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/dmaengine.h>
#include <linux/device.h> //marvin+

#include <asm/irq.h>
#include <asm/pgtable.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/processor.h>
#include <asm/cacheflush.h>

#include <asm/mach-nxc2600/nxc2600.h>
#include <asm/mach-nxc2600/gpio.h>

#include "console/fbcon.h"

#define NR_PALETTE	256

struct nxc2600_fb_dma
{
        struct dma_client client;
        struct dma_chan *channel;
        int chno;
        int len;
        int dir;
        dma_cookie_t cookie;
        spinlock_t lock;
        uint32_t dest,src;
};

struct lcd_desc{
        unsigned int next_desc; /* LCDDAx */
        unsigned int databuf;   /* LCDSAx */
        unsigned int frame_id;  /* LCDFIDx */
        unsigned int cmd;       /* LCDCMDx */
};


struct lcd_cfb_info {
	struct fb_info		fb;
	struct display_switch	*dispsw;
	signed int		currcon;
	int			func_use_count;

	struct {
		u16 red, green, blue;
	} palette[NR_PALETTE];
#ifdef CONFIG_PM
	struct pm_dev		*pm;
#endif
	struct nxc2600_fb_dma	dma;
};

static struct lcd_cfb_info *nxc2600lcd_info;

struct nxc2600fb_info {
	unsigned int cfg;	/* panel mode and pin usage etc. */
	unsigned int w;
	unsigned int h;
	unsigned int bpp;	/* bit per pixel */
	unsigned int fclk;	/* frame clk */
	unsigned int hsw;	/* hsync width, in pclk */
	unsigned int vsw;	/* vsync width, in line count */
	unsigned int elw;	/* end of line, in pclk */
	unsigned int blw;	/* begin of line, in pclk */
	unsigned int efw;	/* end of frame, in line count */
	unsigned int bfw;	/* begin of frame, in line count */
};

static struct nxc2600fb_info nxc2600fb = 
{
#ifdef CONFIG_NXC2600_PANEL_FG0700A0DSSWAGT1
	NXC2600_LCD_CFG_MODE_TFT | NXC2600_LCD_CFG_HSP | NXC2600_LCD_CFG_VSP,
	800, 480, 16, 27, 100, 20, 0, 0, 0, 0,
#endif	
#ifdef CONFIG_NXC2600_PANEL_TD028THED1
        NXC2600_LCD_CFG_MODE_TFT | NXC2600_LCD_CFG_HSP | NXC2600_LCD_CFG_VSP,
	240, 320, 16, 60, 10, 2, 10, 10, 2, 2,
#endif
#ifdef CONFIG_NXC2600_PANEL_TFP507MWVGAHBE_03
	NXC2600_LCD_CFG_MODE_TFT | NXC2600_LCD_CFG_HSP | NXC2600_LCD_CFG_VSP,
	640, 480, 16, 60, 41, 10, 2, 2, 2, 2
#endif
#ifdef CONFIG_NXC2600_PANEL_SAMSUNG_LTE400WQF02
	NXC2600_LCD_CFG_MODE_TFT | NXC2600_LCD_CFG_HSP | NXC2600_LCD_CFG_VSP,
	480, 272, 16, 60, 41, 10, 2, 2, 2, 2
#endif
#ifdef CONFIG_NXC2600_PANEL_TOPSUN_TS188A_T
	NXC2600_LCD_CFG_MODE_TFT | NXC2600_LCD_CFG_HSP | NXC2600_LCD_CFG_VSP,
	480, 234, 16, 60, 82, 16, 2, 2, 2, 2,
#endif
#ifdef CONFIG_NXC2600_PANEL_DATAIMAGE_CLAA070LC0ACW
	NXC2600_LCD_CFG_MODE_TFT | NXC2600_LCD_CFG_HSP | NXC2600_LCD_CFG_VSP,
        800, 480, 16, 60, 41, 10, 2, 2, 2, 2,
#endif
#ifdef CONFIG_NXC2600_PANEL_TMT043DNAFWU3
	NXC2600_LCD_CFG_MODE_TFT | NXC2600_LCD_CFG_HSP | NXC2600_LCD_CFG_VSP,
	480, 272, 16, 60, 41, 10, 2, 2, 2, 2,
#endif
};

static struct lcd_desc lcd_palette_desc __attribute__ ((aligned (16)));
static struct lcd_desc lcd_frame_desc0 __attribute__ ((aligned (16)));
static struct lcd_desc lcd_frame_desc1 __attribute__ ((aligned (16)));

static volatile unsigned char *lcd_palette;
static volatile unsigned char *lcd_frame;

//extern struct display fb_display[MAX_NR_CONSOLES];

static int lcd_hw_init(void)
{

        struct nxc2600_panel panel =
        {
#ifdef CONFIG_NXC2600_PANEL_TFP507MWVGAHBE_03
 		
                //.name = "TFP507MWVGAHBE_03",
                .name = "VGA_640x480",
                .ctrl           =       NXC2600_LCD_CTRL_BPP_16,
                .cfg            =       NXC2600_LCD_CFG_MODE_TFT |
                                        NXC2600_LCD_CFG_HSP |
                                        NXC2600_LCD_CFG_VSP,
                .fclk           =       60,
                .hsw            =       41,
                .vsw            =       10,
                .elw            =       2,
                .blw            =       2,
                .efw            =       2,
                .bfw            =       2,
                .max_xres       =       640,
                .min_xres       =       640,
                .max_yres       =       480,
                .min_yres       =       480,
                .xres           =       0,
                .yres           =       0,
                .bpp            =       16,
                
#endif
#ifdef CONFIG_NXC2600_PANEL_SAMSUNG_LTE400WQF02
                .name = "LTE400WQF02",
                .ctrl           =       NXC2600_LCD_CTRL_BPP_16,
                .cfg            =       NXC2600_LCD_CFG_MODE_TFT |
                                        NXC2600_LCD_CFG_HSP |
                                        NXC2600_LCD_CFG_VSP,
                .fclk           =       60,
                .hsw            =       41,
                .vsw            =       10,
                .elw            =       2,
                .blw            =       2,
                .efw            =       2,
                .bfw            =       2,
                .max_xres       =       480,
                .min_xres       =       480,
                .max_yres       =       272,
                .min_yres       =       272,
                .xres           =       0,
                .yres           =       0,
                .bpp            =       16,
#endif
#ifdef CONFIG_NXC2600_PANEL_TOPSUN_TS188A_T
                .name = "TS188A_T",
                .ctrl           =       NXC2600_LCD_CTRL_BPP_16,
                .cfg            =       NXC2600_LCD_CFG_MODE_TFT |
                                        NXC2600_LCD_CFG_HSP |
                                        NXC2600_LCD_CFG_VSP,
                .fclk           =       60,
                .hsw            =       82,
                .vsw            =       16,
                .elw            =       2,
                .blw            =       2,
                .efw            =       2,
                .bfw            =       2,
                .max_xres       =       480,
                .min_xres       =       480,
                .max_yres       =       234,
                .min_yres       =       234,
                .xres           =       0,
                .yres           =       0,
                .bpp            =       16,
#endif
#ifdef CONFIG_NXC2600_PANEL_DATAIMAGE_CLAA070LC0ACW
                .name = "VGA_800x480",
                .ctrl           =       NXC2600_LCD_CTRL_BPP_16,
                .cfg            =       NXC2600_LCD_CFG_MODE_TFT |
                                        NXC2600_LCD_CFG_HSP |
                                        NXC2600_LCD_CFG_VSP,
                .fclk           =       60,
                .hsw            =       41,
                .vsw            =       10,
                .elw            =       2,
                .blw            =       2,
                .efw            =       2,
                .bfw            =       2,
                .max_xres       =       800,
                .min_xres       =       800,
                .max_yres       =       480,
                .min_yres       =       480,
                .xres           =       0,
                .yres           =       0,
                .bpp            =       16,
#endif
#ifdef CONFIG_NXC2600_PANEL_TMT043DNAFWU3
		.name = "480x272",
                .ctrl           =       NXC2600_LCD_CTRL_BPP_16,
                .cfg            =       NXC2600_LCD_CFG_MODE_TFT |
                                        NXC2600_LCD_CFG_HSP |
                                        NXC2600_LCD_CFG_VSP,
                .fclk           =       60,
                .hsw            =       41,
                .vsw            =       10,
                .elw            =       2,
                .blw            =       2,
                .efw            =       2,
                .bfw            =       2,
                .max_xres       =       480,
                .min_xres       =       480,
                .max_yres       =       272,
                .min_yres       =       272,
                .xres           =       0,
                .yres           =       0,
                .bpp            =       16,
#endif
#ifdef CONFIG_NXC2600_PANEL_TD028THED1

				.name = "240x320",
				.ctrl			=		NXC2600_LCD_CTRL_BPP_16,
				.cfg			=		NXC2600_LCD_CFG_MODE_TFT |
										NXC2600_LCD_CFG_HSP |
										NXC2600_LCD_CFG_VSP,
				.fclk			=		60,
				.hsw			=		10,
				.vsw			=		2,
				.elw			=		10,
				.blw			=		10,
				.efw			=		2,
				.bfw			=		2,
				.max_xres		=		240,
				.min_xres		=		240,
				.max_yres		=		320,
				.min_yres		=		320,
				.xres			=		0,
				.yres			=		0,
				.bpp			=		16,
#endif
#ifdef CONFIG_NXC2600_PANEL_FG0700A0DSSWAGT1
		.name = "800x480",
                .ctrl           =       NXC2600_LCD_CTRL_BPP_16,
                .cfg            =       NXC2600_LCD_CFG_MODE_TFT |
                                        NXC2600_LCD_CFG_HSP |
                                        NXC2600_LCD_CFG_VSP,
                .fclk           =       27,
                .hsw            =       100,
                .vsw            =       20,
                .elw            =       0,
                .blw            =       0,
                .efw            =       0,
                .bfw            =       0,
                .max_xres       =       800,
                .min_xres       =       800,
                .max_yres       =       480,
                .min_yres       =       480,
                .xres           =       0,
                .yres           =       0,
                .bpp            =       16,
#endif
        };


	nxc2600_lcd_init(&panel);

        return 0;
}

extern int nxc2600_dma_get_chno(struct dma_chan *chan);
static enum dma_state_client
nxc2600_dma_event( struct dma_client *client,
                    struct dma_chan *chan,
                    enum dma_state state)
{
        struct nxc2600_fb_dma *dma = (struct nxc2600_fb_dma *)client;
        enum dma_state_client ack = DMA_NAK;

        spin_lock(&(dma->lock));
        switch (state)
        {
                case DMA_RESOURCE_AVAILABLE:
                        if( dma->channel == NULL )
                        {
                                ack = DMA_ACK;
                                dma->channel = chan;
                                dma->chno = nxc2600_dma_get_chno(chan);
                        }
                        break;
                case DMA_RESOURCE_REMOVED:
                        if( dma->channel == chan )
                        {
                                ack = DMA_ACK;
                                dma->channel = NULL;
                                dma->chno = -1;
                        }
                        break;
                default:
                        break;
        }
        spin_unlock(&(dma->lock));

        return ack;
}

static __inline__ int
nxc2600_fb_start_dma(struct nxc2600_fb_dma *dma,
			uint32_t src,
			uint32_t dest,
			uint32_t size)
{
        struct dma_async_tx_descriptor *tx;

        tx = dma->channel->device->device_prep_dma_memcpy(dma->channel, size, 0);
        if (!tx)
                return -ENOMEM;

        tx->ack = 1;
        tx->callback = NULL; //nxc2600_fb_dma_callback;
        tx->callback_param = dma;
        tx->tx_set_src(src, tx, 0);
        tx->tx_set_dest(dest, tx, 0);
        tx->tx_submit(tx);
	dma->channel->device->device_issue_pending(dma->channel);
        return 0;

}

static int nxc2600fb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
			  u_int transp, struct fb_info *info)
{
	struct lcd_cfb_info *cfb = (struct lcd_cfb_info *)info;
	unsigned short *ptr, ctmp;

	if (regno >= NR_PALETTE)
		return 1;

	red	>>= 8;
	green	>>= 8;
	blue	>>= 8;
	
	red	&= 0xff;
	green	&= 0xff;
	blue	&= 0xff;

	switch (cfb->fb.var.bits_per_pixel) {
	case 1:
	case 2:
	case 4:
	case 8:
		if (((nxc2600fb.cfg & NXC2600_LCD_CFG_MODE_MASK) == NXC2600_LCD_CFG_MODE_DUAL_MONO_STN) ||
		    ((nxc2600fb.cfg & NXC2600_LCD_CFG_MODE_MASK) == NXC2600_LCD_CFG_MODE_DUAL_MONO_STN)) {
			ctmp = (77L * red + 150L * green + 29L * blue) >> 8;
			ctmp = ((ctmp >> 3) << 11) | ((ctmp >> 2) << 5) |
				(ctmp >> 3);
		} else {
			/* RGB 565 */
			if (((red >> 3) == 0) && ((red >> 2) != 0))
				red = 1 << 3;
			if (((blue >> 3) == 0) && ((blue >> 2) != 0))
				blue = 1 << 3;
			ctmp = ((red >> 3) << 11) 
				| ((green >> 2) << 5) | (blue >> 3);
		}

		ptr = (unsigned short *)lcd_palette;
		ptr = (unsigned short *)(((u32)ptr)|0xa0000000);
		ptr[regno] = ctmp;

		break;
		
	case 15:
		if (regno < 16)
			((u32 *)cfb->fb.pseudo_palette)[regno] =
				((red >> 3) << 10) | 
				((green >> 3) << 5) |
				(blue >> 3);
		break;
	case 16:
		if (regno < 16) {
			((u32 *)cfb->fb.pseudo_palette)[regno] =
				((red >> 3) << 11) | 
				((green >> 2) << 5) |
				(blue >> 3); 
		}
		break;
	case 24:
	case 32:
		if (regno < 16)
			((u32 *)cfb->fb.pseudo_palette)[regno] =
				regno | regno << 8 | regno << 16;
		break;
	}
	return 0;
}

static int nxc2600fb_ioctl (struct fb_info *info, unsigned int cmd, unsigned long arg ) //marvin+
{
	int blevel;
	struct nxc2600_backlight *bl = (struct nxc2600_backlight *)info; 
    uint32_t clock;

	
	if (!info || !bl)
		return -ENODEV;
	
	switch(cmd) {
		case NXC2600_SET_BRIGHTNESS:
			copy_from_user((void *)&blevel, (void *)arg, sizeof(blevel));
           	
	        if (bl->current_intensity != blevel) {
		        if (bl->powermode == FB_BLANK_UNBLANK) {
					clock = 28800*blevel;
					nxc2600_pwm_disable(CONFIG_BACKLIGHT_NXC2600_PWM);
                 	nxc2600_pwm_init(CONFIG_BACKLIGHT_NXC2600_PWM, clock);
                	if(blevel)
                	   nxc2600_pwm_enable(CONFIG_BACKLIGHT_NXC2600_PWM);
		        }
		        if (blevel > 0 && blevel <= NXC2600BL_MAX_INTENSITY ) 
 		            bl->current_intensity = blevel;
	        }                     
			break;
		case NXC2600_GET_BRIGHTNESS:
			blevel=bl->current_intensity;
		    copy_to_user((void *)arg, (void *)&blevel, 1);
			break;
		}

	return 0;
}

/* Use mmap /dev/fb can only get a non-cacheable Virtual Address. */
static int nxc2600fb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	struct lcd_cfb_info *cfb = (struct lcd_cfb_info *)info;
	unsigned long start;
	unsigned long off;
	u32 len;

	off = vma->vm_pgoff << PAGE_SHIFT;
	//fb->fb_get_fix(&fix, PROC_CONSOLE(info), info);

	/* frame buffer memory */
	start = cfb->fb.fix.smem_start;
	len = PAGE_ALIGN((start & ~PAGE_MASK) + cfb->fb.fix.smem_len);
	start &= PAGE_MASK;

	if ((vma->vm_end - vma->vm_start + off) > len)
		return -EINVAL;
	off += start;

	vma->vm_pgoff = off >> PAGE_SHIFT;
	vma->vm_flags |= VM_IO;

#if defined(CONFIG_MIPS32)
	pgprot_val(vma->vm_page_prot) &= ~_CACHE_MASK;
	pgprot_val(vma->vm_page_prot) |= _CACHE_UNCACHED;
//	pgprot_val(vma->vm_page_prot) |= _CACHE_CACHABLE_NONCOHERENT;
#endif

	if (io_remap_pfn_range(vma, vma->vm_start, off >> PAGE_SHIFT,
			       vma->vm_end - vma->vm_start,
			       vma->vm_page_prot)) {
		return -EAGAIN;
	}
	
	return 0;
}

/* checks var and eventually tweaks it to something supported,
 * DO NOT MODIFY PAR */
static int nxc2600fb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	return 0;
}


/* 
 * set the video mode according to info->var
 */
static int nxc2600fb_set_par(struct fb_info *info)
{
	nxc2600_lcd_on();
	return 0;
}


/*
 * (Un)Blank the display.
 * Fix me: should we use VESA value?
 */
static int nxc2600fb_blank(int blank_mode, struct fb_info *info)
{
	static volatile int idle_mode = 0;
	switch (blank_mode) {

	case FB_BLANK_UNBLANK:
			/* Turn on panel */
		nxc2600_lcd_on();
		idle_mode = !idle_mode;
		break;

	case FB_BLANK_NORMAL:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_POWERDOWN:
		nxc2600_lcd_off();
		idle_mode = 0;
		break;
	default:
		break;

	}
	return 0;
}

/* 
 * pan display
 */
static int nxc2600fb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct lcd_cfb_info *cfb = (struct lcd_cfb_info *)info;
	int dy;

	if (!var || !cfb) {
		return -EINVAL;
	}

	if (var->xoffset - cfb->fb.var.xoffset) {
		/* No support for X panning for now! */
		return -EINVAL;
	}

	dy = var->yoffset - cfb->fb.var.yoffset;
	if (dy) {

		lcd_frame_desc0.databuf += (cfb->fb.fix.line_length * dy);
		/* TODO: Wait for current frame to finished */

		//if ( dual_panel) {
		//printk("Fix me: dual_panel in nxc2600fb_pan_display.");
		//}
	}

	return 0;
}

static void 
nxc2600_fb_copyarea(struct fb_info *p, 
		    const struct fb_copyarea *area)
{
	struct lcd_cfb_info *fb = (struct lcd_cfb_info *)p;
        u32 dx = area->dx, dy = area->dy, sx = area->sx, sy = area->sy;
        u32 height = area->height, width = area->width*p->var.bits_per_pixel/8;
        unsigned long dst_idx = 0, src_idx = 0, src_off, dst_off;

	if(fb->dma.channel==NULL)
	{
		cfb_copyarea(p,area);
		return;
	}
        if (p->state != FBINFO_STATE_RUNNING)
                return;

        dst_idx = src_idx = ((unsigned long)p->screen_base);
        // add offset of source and target area
        dst_off = (dy*p->var.bits_per_pixel*p->var.xres + dx*p->var.bits_per_pixel);

        src_off = (sy*p->var.bits_per_pixel*p->var.xres + sx*p->var.bits_per_pixel);

	if((dst_idx&3) || (src_idx&3) || (dst_off&7) || (src_off&7))
        {
		cfb_copyarea(p,area);
                return;
        }

	src_idx += src_off/8;
	dst_idx += dst_off/8;

	if(p->fix.line_length==width)
	{
		nxc2600_fb_start_dma(&(fb->dma),src_idx,dst_idx,width*height);

	}
	else
	{
	        while (height--) 
		{
                	nxc2600_fb_start_dma(&(fb->dma),src_idx,dst_idx,width);
                	dst_idx += width;
                        src_idx += width;
	        }
	}
}

static void 
nxc2600_fb_fillrect(struct fb_info *p, 
		    const struct fb_fillrect *rect)
{
	/* TODO */

	cfb_fillrect(p,rect);
}

static void 
nxc2600_fb_imageblit(struct fb_info *p, const struct fb_image *image)
{
	struct lcd_cfb_info *fb = (struct lcd_cfb_info *)p;

        u32 width = image->width*p->var.bits_per_pixel/8;
	u32 height = image->height;
        u32 dx = image->dx, dy = image->dy;
	unsigned long dst_idx = 0, src_idx = 0;
	u32 off;

        dst_idx = ((unsigned long)p->screen_base);
        src_idx = ((unsigned long)image->data);
        // add offset of source and target area
        off = (dy*p->var.bits_per_pixel*p->var.xres + dx*p->var.bits_per_pixel);

        if((fb->dma.channel==NULL) || (image->depth!=15 && image->depth!=16) ||
	   (off&7) || (src_idx&3) || (dst_idx&3))
        {
                cfb_imageblit(p,image);
                return;
        }

        if (p->state != FBINFO_STATE_RUNNING)
                return;

	dst_idx += (off/8);

        if(p->fix.line_length==width)
        {
                nxc2600_fb_start_dma(&(fb->dma),src_idx,dst_idx,width*height);
        }
        else
        {
		while(height--)
		{
                	while(nxc2600_fb_start_dma(&(fb->dma),src_idx,dst_idx,width))
				udelay(1);
			src_idx += width;
			dst_idx += width;
		}
	}
}

/* use default function cfb_fillrect, cfb_copyarea, cfb_imageblit */
static struct fb_ops nxc2600fb_ops = {
	.owner			= THIS_MODULE,
	.fb_setcolreg		= nxc2600fb_setcolreg,
	.fb_check_var 		= nxc2600fb_check_var,
	.fb_set_par 		= nxc2600fb_set_par,
	//.fb_setcmap		= nxc2600fb_setcmap,
	.fb_blank		= nxc2600fb_blank,
	.fb_pan_display		= nxc2600fb_pan_display,
	.fb_fillrect		= nxc2600_fb_fillrect,
	.fb_copyarea		= nxc2600_fb_copyarea,
	.fb_imageblit		= nxc2600_fb_imageblit,
	.fb_mmap		= nxc2600fb_mmap,
	//.fb_cursor		= nxc2600fb_cursor,
	.fb_ioctl		= nxc2600fb_ioctl,
};

static int nxc2600fb_set_var(struct fb_var_screeninfo *var, int con,
			struct fb_info *info)
{
	struct lcd_cfb_info *cfb = (struct lcd_cfb_info *)info;
	//struct display *display;
	int chgvar = 0;

	var->height	            = nxc2600fb.h ;
	var->width	            = nxc2600fb.w ;
	var->bits_per_pixel	    = nxc2600fb.bpp;

	var->vmode                  = FB_VMODE_NONINTERLACED;
	var->activate               = cfb->fb.var.activate;
	var->xres                   = var->width;
	var->yres                   = var->height;
	var->xres_virtual           = var->width;
	var->yres_virtual           = var->height;
	var->xoffset                = 0;
	var->yoffset                = 0;
	var->pixclock               = 0;
	var->left_margin            = 0;
	var->right_margin           = 0;
	var->upper_margin           = 0;
	var->lower_margin           = 0;
	var->hsync_len              = 0;
	var->vsync_len              = 0;
	var->sync                   = 0;
	var->activate              &= ~FB_ACTIVATE_TEST;
    
	/*
	 * CONUPDATE and SMOOTH_XPAN are equal.  However,
	 * SMOOTH_XPAN is only used internally by fbcon.
	 */
	if (var->vmode & FB_VMODE_CONUPDATE) {
		var->vmode |= FB_VMODE_YWRAP;
		var->xoffset = cfb->fb.var.xoffset;
		var->yoffset = cfb->fb.var.yoffset;
	}

	if (var->activate & FB_ACTIVATE_TEST)
		return 0;

	if ((var->activate & FB_ACTIVATE_MASK) != FB_ACTIVATE_NOW)
		return -EINVAL;

	if (cfb->fb.var.xres != var->xres)
		chgvar = 1;
	if (cfb->fb.var.yres != var->yres)
		chgvar = 1;
	if (cfb->fb.var.xres_virtual != var->xres_virtual)
		chgvar = 1;
	if (cfb->fb.var.yres_virtual != var->yres_virtual)
		chgvar = 1;
	if (cfb->fb.var.bits_per_pixel != var->bits_per_pixel)
		chgvar = 1;

	//display = fb_display + con;

	var->red.msb_right	= 0;
	var->green.msb_right	= 0;
	var->blue.msb_right	= 0;

	switch (var->bits_per_pixel) {
	case 1:	/* Mono */
		cfb->fb.fix.visual	= FB_VISUAL_MONO01;
		cfb->fb.fix.line_length	= (var->xres * var->bits_per_pixel) / 8;
		break;
	case 2:	/* Mono */
		var->red.offset		= 0;
		var->red.length		= 2;
		var->green.offset	= 0;
		var->green.length	= 2;
		var->blue.offset	= 0;
		var->blue.length	= 2;

		cfb->fb.fix.visual	= FB_VISUAL_PSEUDOCOLOR;
		cfb->fb.fix.line_length	= (var->xres * var->bits_per_pixel) / 8;
		break;
	case 4:	/* PSEUDOCOLOUR*/
		var->red.offset		= 0;
		var->red.length		= 4;
		var->green.offset	= 0;
		var->green.length	= 4;
		var->blue.offset	= 0;
		var->blue.length	= 4;

		cfb->fb.fix.visual	= FB_VISUAL_PSEUDOCOLOR;
		cfb->fb.fix.line_length	= var->xres / 2;
		break;
	case 8:	/* PSEUDOCOLOUR, 256 */
		var->red.offset		= 0;
		var->red.length		= 8;
		var->green.offset	= 0;
		var->green.length	= 8;
		var->blue.offset	= 0;
		var->blue.length	= 8;

		cfb->fb.fix.visual	= FB_VISUAL_PSEUDOCOLOR;
		cfb->fb.fix.line_length	= var->xres ;
		break;
	case 15: /* DIRECTCOLOUR, 32k */
		var->bits_per_pixel	= 15;
		var->red.offset		= 10;
		var->red.length		= 5;
		var->green.offset	= 5;
		var->green.length	= 5;
		var->blue.offset	= 0;
		var->blue.length	= 5;

		cfb->fb.fix.visual	= FB_VISUAL_TRUECOLOR;
		cfb->fb.fix.line_length	= var->xres_virtual * 2;
		break;
	case 16: /* DIRECTCOLOUR, 64k */
		var->bits_per_pixel	= 16;
		var->red.offset		= 11;
		var->red.length		= 5;
		var->green.offset	= 5;
		var->green.length	= 6;
		var->blue.offset	= 0;
		var->blue.length	= 5;

		cfb->fb.fix.visual	= FB_VISUAL_TRUECOLOR;
		cfb->fb.fix.line_length	= var->xres_virtual * 2;
		break;
	default: /* in theory this should never happen */
		printk(KERN_WARNING "%s: no support for %dbpp\n",
		       cfb->fb.fix.id, var->bits_per_pixel);
		//cfb->dispsw = &fbcon_dummy;
		break;
	}

	//display->dispsw = cfb->dispsw;
	//cfb->fb.fix.line_length	=  var->xres*x ;

	//display->screen _base	= cfb->fb.screen_base;
	//display->ypanstep	= cfb->fb.fix.ypanstep;
	//display->inverse	= 0;

	cfb->fb.var = *var;
	cfb->fb.var.activate &= ~FB_ACTIVATE_ALL;

	/*
	 * Update the old var.  The fbcon drivers still use this.
	 * Once they are using cfb->fb.var, this can be dropped.
	 *					--rmk
	 */
	//display->var = cfb->fb.var;
	/*
	 * If we are setting all the virtual consoles, also set the
	 * defaults used to create new consoles.
	 */
	fb_set_cmap(&cfb->fb.cmap, &cfb->fb);
	nxc2600_lcd_on();
	return 0;
}

static struct lcd_cfb_info * nxc2600fb_alloc_fb_info(void)
{
 	struct lcd_cfb_info *cfb;

	//cfb = kmalloc(sizeof(struct lcd_cfb_info) + sizeof(struct display) +
	//sizeof(u32) * 16, GFP_KERNEL);
	cfb = kmalloc(sizeof(struct lcd_cfb_info) + sizeof(u32) * 16, GFP_KERNEL);

	if (!cfb)
		return NULL;

	nxc2600lcd_info = cfb;

	//memset(cfb, 0, sizeof(struct lcd_cfb_info) + sizeof(struct display));
	memset(cfb, 0, sizeof(struct lcd_cfb_info) );

	cfb->currcon		= -1;


	strlcpy(cfb->fb.fix.id, "NXC2600-FB", sizeof(cfb->fb.fix.id));
	cfb->fb.fix.type	= FB_TYPE_PACKED_PIXELS;
	cfb->fb.fix.type_aux	= 0;
	//cfb->fb.fix.xpanstep	= 0;
	//cfb->fb.fix.ypanstep	= 0;
	cfb->fb.fix.xpanstep	= 1;
	cfb->fb.fix.ypanstep	= 1;
	cfb->fb.fix.ywrapstep	= 0;
	cfb->fb.fix.accel	= FB_ACCEL_NONE;

	cfb->fb.var.nonstd	= 0;
	cfb->fb.var.activate	= FB_ACTIVATE_NOW;
	cfb->fb.var.height	= -1;
	cfb->fb.var.width	= -1;
	cfb->fb.var.accel_flags	= FB_ACCELF_TEXT;


	cfb->fb.fbops		= &nxc2600fb_ops;
	//cfb->fb.changevar	= NULL;
	//cfb->fb.switch_con	= nxc2600fb_switch;
	//cfb->fb.updatevar	= nxc2600fb_updatevar;
	//cfb->fb.blank		= nxc2600fb_blank;
	cfb->fb.flags		= FBINFO_FLAG_DEFAULT;
	//cfb->fb.disp		= (struct display *)(cfb + 1);

	cfb->fb.pseudo_palette	= (void *)(cfb + 1);

	switch (nxc2600fb.bpp) {
	case 1:
		fb_alloc_cmap(&cfb->fb.cmap, 4, 0);
		break;
	case 2:
		fb_alloc_cmap(&cfb->fb.cmap, 8, 0);
		break;
	case 4:
		fb_alloc_cmap(&cfb->fb.cmap, 32, 0);
		break;
	case 8:
	default:
		fb_alloc_cmap(&cfb->fb.cmap, 256, 0);
		break;
	}

	return cfb;
}

/*
 * Map screen memory
 */
static int nxc2600fb_map_smem(struct lcd_cfb_info *cfb)
{
	struct page * map = NULL;
	unsigned char *tmp;
	unsigned int page_shift, needroom, t;

	if (nxc2600fb.bpp == 15)
		t = 16;
	else
		t = nxc2600fb.bpp;

	needroom = ((nxc2600fb.w * t + 7) >> 3) * nxc2600fb.h;
	for (page_shift = 0; page_shift < 12; page_shift++)
		if ((PAGE_SIZE << page_shift) >= needroom)
			break;

	lcd_palette = (unsigned char *)__get_free_pages(GFP_KERNEL, 0);
	lcd_frame = (unsigned char *)__get_free_pages(GFP_KERNEL, page_shift);

	if ((!lcd_palette) || (!lcd_frame))
		return -ENOMEM;

	memset((void *)lcd_palette, 0, PAGE_SIZE);
	memset((void *)lcd_frame, 0, PAGE_SIZE << page_shift);

	map = virt_to_page(lcd_palette);
	set_bit(PG_reserved, &map->flags);

	for (tmp=(unsigned char *)lcd_frame;
	     tmp < lcd_frame + (PAGE_SIZE << page_shift);
	     tmp += PAGE_SIZE) {
		map = virt_to_page(tmp);
		set_bit(PG_reserved, &map->flags);
	}

	cfb->fb.fix.smem_start = virt_to_phys((void *)lcd_frame);

	cfb->fb.fix.smem_len = (PAGE_SIZE << page_shift);

	cfb->fb.screen_base =
		(unsigned char *)(((unsigned int)lcd_frame & 0x1fffffff) | 0xa0000000);

	if (!cfb->fb.screen_base) {
		printk("%s: unable to map screen memory\n", cfb->fb.fix.id);
		return -ENOMEM;
	}

	return 0;
}

static void nxc2600fb_free_fb_info(struct lcd_cfb_info *cfb)
{
	if (cfb) {
		fb_alloc_cmap(&cfb->fb.cmap, 0, 0);
		kfree(cfb);
	}
}

static void nxc2600fb_unmap_smem(struct lcd_cfb_info *cfb)
{
	struct page * map = NULL;
	unsigned char *tmp;
	unsigned int page_shift, needroom, t;

	if (nxc2600fb.bpp == 15)
		t = 16;
	else
		t = nxc2600fb.bpp;
	needroom = ((nxc2600fb.w * t + 7) >> 3) * nxc2600fb.h;
	for (page_shift = 0; page_shift < 12; page_shift++)
		if ((PAGE_SIZE << page_shift) >= needroom)
			break;

	if (cfb && cfb->fb.screen_base) {
		iounmap(cfb->fb.screen_base);
		cfb->fb.screen_base = NULL;
		release_mem_region(cfb->fb.fix.smem_start,
				   cfb->fb.fix.smem_len);
	}

	if (lcd_palette) {
		map = virt_to_page(lcd_palette);
		clear_bit(PG_reserved, &map->flags);
		free_pages((int)lcd_palette, 0);
	}

	if (lcd_frame) {

		for (tmp=(unsigned char *)lcd_frame; 
		     tmp < lcd_frame + (PAGE_SIZE << page_shift); 
		     tmp += PAGE_SIZE) {
			map = virt_to_page(tmp);
			clear_bit(PG_reserved, &map->flags);
		}

		free_pages((int)lcd_frame, page_shift);
	}
}

static void lcd_descriptor_init(void)
{
	int i;
	unsigned int pal_size;
	unsigned int frm_size, ln_size;
	unsigned char dual_panel = 0;

	i = nxc2600fb.bpp;
	if (i == 15)
		i = 16;
	frm_size = (nxc2600fb.w*nxc2600fb.h*i)>>3;
	ln_size = (nxc2600fb.w*i)>>3;

	if (((nxc2600fb.cfg & NXC2600_LCD_CFG_MODE_MASK) == NXC2600_LCD_CFG_MODE_DUAL_MONO_STN) ||
	    ((nxc2600fb.cfg & NXC2600_LCD_CFG_MODE_MASK) == NXC2600_LCD_CFG_MODE_DUAL_MONO_STN)) {
		dual_panel = 1;
		frm_size >>= 1;
	}

	frm_size = frm_size / 4;
	ln_size = ln_size / 4;

	switch (nxc2600fb.bpp) {
	case 1:
		pal_size = 4;
		break;
	case 2:
		pal_size = 8;
		break;
	case 4:
		pal_size = 32;
		break;
	case 8:
	default:
		pal_size = 512;
		break;
	}

	pal_size /= 4;

	/* Palette Descriptor */
	lcd_palette_desc.next_desc = (int)virt_to_phys(&lcd_frame_desc0);
	lcd_palette_desc.databuf = (int)virt_to_phys((void *)lcd_palette);
	lcd_palette_desc.frame_id = (unsigned int)0xdeadbeaf;
	lcd_palette_desc.cmd = pal_size|NXC2600_LCD_CMD_PAL; /* Palette Descriptor */

	/* Frame Descriptor 0 */
	if (nxc2600fb.bpp <= 8)
		lcd_frame_desc0.next_desc = (int)virt_to_phys(&lcd_palette_desc);
	else
		lcd_frame_desc0.next_desc = (int)virt_to_phys(&lcd_frame_desc0);
	lcd_frame_desc0.databuf = virt_to_phys((void *)lcd_frame);
	lcd_frame_desc0.frame_id = (unsigned int)0xbeafbeaf;
	lcd_frame_desc0.cmd = NXC2600_LCD_CMD_SOFINT|NXC2600_LCD_CMD_EOFINT | frm_size;
	dma_cache_wback((unsigned int)(&lcd_palette_desc),0x10);
	dma_cache_wback((unsigned int)(&lcd_frame_desc0),0x10);

	if (!(dual_panel))
		return;

	/* Frame Descriptor 1 */
	lcd_frame_desc1.next_desc = (int)virt_to_phys(&lcd_frame_desc1);
	lcd_frame_desc1.databuf = virt_to_phys((void *)(lcd_frame + frm_size * 4));
	lcd_frame_desc1.frame_id = (unsigned int)0xdeaddead;
	lcd_frame_desc1.cmd = NXC2600_LCD_CMD_SOFINT|NXC2600_LCD_CMD_EOFINT | frm_size;
	dma_cache_wback((unsigned int)(&lcd_frame_desc1),0x10);
}


static irqreturn_t lcd_interrupt_handler(int irq, void *dev_id)
{
	unsigned int state;

	state = NXC2600_LCD_STATE;

	if (state & NXC2600_LCD_STATE_EOF) /* End of frame */
		REG32(NXC2600_LCD_STATE) = state & ~NXC2600_LCD_STATE_EOF;

	if (state & NXC2600_LCD_STATE_IFU0) {
		printk("InFiFo0 underrun\n");
		REG32(NXC2600_LCD_STATE) = state & ~NXC2600_LCD_STATE_IFU0;
	}

	if (state & NXC2600_LCD_STATE_OUF) { /* Out fifo underrun */
		REG32(NXC2600_LCD_STATE) = state & ~NXC2600_LCD_STATE_OUF;
		printk("Out FiFo underrun.\n");
	}
	return IRQ_HANDLED;
}

#ifdef CONFIG_PM

/*
 * Suspend the LCDC.
 */
static int nxc2600fb_suspend(struct device *dev, pm_message_t state)
{
	nxc2600_lcd_off();
	return 0;
}

/*
 * Resume the LCDC.
 */
static int nxc2600fb_resume(struct device *dev)
{

	lcd_hw_init();

	if (nxc2600fb.bpp <= 8)
		REG32(NXC2600_LCD_DA(0)) = virt_to_phys(&lcd_palette_desc);
	else
		REG32(NXC2600_LCD_DA(0)) = virt_to_phys(&lcd_frame_desc0);

	if (((nxc2600fb.cfg & NXC2600_LCD_CFG_MODE_MASK) == NXC2600_LCD_CFG_MODE_DUAL_MONO_STN) ||
	    ((nxc2600fb.cfg & NXC2600_LCD_CFG_MODE_MASK) == NXC2600_LCD_CFG_MODE_DUAL_COLOR_STN))
		REG32(NXC2600_LCD_DA(1)) = virt_to_phys(&lcd_frame_desc1);

	nxc2600_lcd_on();
	return 0;
}

/*
 * Power management hook.  Note that we won't be called from IRQ context,
 * unlike the blank functions above, so we may sleep.
 */
static int nxc2600lcd_pm_callback(struct pm_dev *pm_dev, pm_request_t req, void *data)
{
	int ret;
	struct lcd_cfb_info *cfb = pm_dev->data;

	if (!cfb) return -EINVAL;

	switch (req) {
	case PM_SUSPEND:
		ret = nxc2600fb_suspend(cfb, (int)data);
		break;
	case PM_RESUME:
		ret = nxc2600fb_resume(cfb);
		break;

	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}
#else
#define nxc2600fb_suspend      NULL
#define nxc2600fb_resume       NULL
#endif /* CONFIG_PM */


static int nxc2600fb_init(void)
{
	struct lcd_cfb_info *cfb;
	int err = 0;

	cfb = nxc2600fb_alloc_fb_info();
	if (!cfb)
		goto failed;

	err = nxc2600fb_map_smem(cfb);
	if (err)
		goto failed;

	nxc2600fb_set_var(&cfb->fb.var, -1, &cfb->fb);

	lcd_descriptor_init();

	err = lcd_hw_init();
	if (err)
		goto failed;

	if (nxc2600fb.bpp <= 8)
		REG32(NXC2600_LCD_DA(0)) = virt_to_phys(&lcd_palette_desc);
	else
		REG32(NXC2600_LCD_DA(0)) = virt_to_phys(&lcd_frame_desc0);

        if (((nxc2600fb.cfg & NXC2600_LCD_CFG_MODE_MASK) == NXC2600_LCD_CFG_MODE_DUAL_MONO_STN) ||
            ((nxc2600fb.cfg & NXC2600_LCD_CFG_MODE_MASK) == NXC2600_LCD_CFG_MODE_DUAL_COLOR_STN))
                REG32(NXC2600_LCD_DA(1)) = virt_to_phys(&lcd_frame_desc1);

        spin_lock_init(&cfb->dma.lock);
	cfb->dma.client.event_callback = nxc2600_dma_event;
	cfb->dma.channel = NULL;
        dma_cap_set(DMA_MEMCPY, cfb->dma.client.cap_mask);
	//dma_cap_set(DMA_MEMSET, cfb->dma.client.cap_mask);
        dma_async_client_register(&cfb->dma.client);
        dma_async_client_chan_request(&cfb->dma.client);


	if (request_irq(NXC2600_IRQ_LCD, lcd_interrupt_handler, IRQF_DISABLED,
			"lcd", 0)) {
		err = -EBUSY;
		goto failed;
	}


	err = register_framebuffer(&cfb->fb);
	if (err < 0) {
		printk("nxc2600fb_init(): register framebuffer err.\n");
		goto failed;
	}
#ifdef CONFIG_PM
	/*
	 * Note that the console registers this as well, but we want to
	 * power down the display prior to sleeping.
	 */
	cfb->pm = pm_register(PM_SYS_DEV, PM_SYS_VGA, nxc2600lcd_pm_callback);
	if (cfb->pm)
		cfb->pm->data = cfb;
#endif
	nxc2600_lcd_on();
	return 0;

failed:
	nxc2600fb_unmap_smem(cfb);
	nxc2600fb_free_fb_info(cfb);

	return err;

}
#if 0
static int nxc2600fb_remove(struct device *dev)
{
	struct lcd_cfb_info *cfb = dev_get_drvdata(dev);
	nxc2600fb_unmap_smem(cfb);
	nxc2600fb_free_fb_info(cfb);
	return 0;
}
#endif
void __exit nxc2600fb_cleanup(void)
{
}

module_init(nxc2600fb_init);
module_exit(nxc2600fb_cleanup);

MODULE_DESCRIPTION("NXC2600 frame buffer driver");
MODULE_AUTHOR("Hardy Weng <hardy.weng@icnexus.com.tw>");
MODULE_LICENSE("GPL");

