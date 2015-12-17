/*
 * linux/drivers/media/dvb/stbx25xxfb.c - Framebuffer driver for IBM STBx25xx (part of the DVB driver)
 *
 *  Created 1 Mar 2009 by Tomasz Figa <tomasz.figa@gmail.com>
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 */

/*
#define DEBUG		10
#define DBG_LEVEL	1
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/pci.h>

#include "stbx25xx.h"
#include "stbx25xx_video.h"

#pragma pack(1)
struct stbx25xx_osd_hdr {
	unsigned ctu	:1;
	unsigned width	:8;
	unsigned shade	:4;
	unsigned high	:1;
	unsigned top	:9;
	unsigned left	:9;
	unsigned link	:16;
	unsigned cr	:1;
	unsigned height	:9;
	unsigned pr	:1;
	unsigned blend	:4;
	unsigned transp	:1;
};

struct stbx25xx_osd_hdr_ext {
	unsigned bitmap	:19;
	unsigned bm_ext	:4;
	unsigned hscrl	:2;
	unsigned ext2	:1;
	unsigned uni_sh	:1;
	unsigned sh_lvl	:2;
	unsigned uni_bl	:1;
	unsigned bl_lvl	:2;
	unsigned hscale	:4;
	unsigned tile	:2;
	unsigned af	:2;
	unsigned	:1;
	unsigned col_bl	:1;
	unsigned ext3	:1;
	unsigned	:1;
	unsigned chr_en	:1;
	unsigned chroma	:19;
};

struct stbx25xx_color_entry {
	unsigned y	:6;
	unsigned cb	:4;
	unsigned cr	:4;
#define CLR_A_BL_EN	2
#define CLR_A_FILL_EN	1
	unsigned a	:2;
};
#pragma pack()

/*
 * Driver data
 */

#define FB_TYPE_GRP	0
#define FB_TYPE_IMG	1
#define FB_MODE_PAL	0
#define FB_MODE_NTSC	1
struct stbx25xx_par {
	struct	fb_info 		*info;
	int				type;
	int				mode;
	struct stbx25xx_osd_hdr		*hdr;
	struct stbx25xx_osd_hdr_ext	*ext;
	struct stbx25xx_color_entry	*pal;
	size_t				mem_size;
	struct stbx25xx_video_data	*video;
};

static struct fb_fix_screeninfo stbx25xxfb_fix[STBx25xx_FB_COUNT] = {
	[0] = {
		.id =		"Graphic Plane", 
		.type =		FB_TYPE_PACKED_PIXELS,
		.visual =	FB_VISUAL_PSEUDOCOLOR,
		.xpanstep =	0,
		.ypanstep =	0,
		.ywrapstep =	0, 
		.accel =	FB_ACCEL_NONE,
	},
	[1] = {
		.id =		"Image Plane", 
		.type =		FB_TYPE_PACKED_PIXELS,
		.visual =	FB_VISUAL_PSEUDOCOLOR,
		.xpanstep =	0,
		.ypanstep =	0,
		.ywrapstep =	0, 
		.accel =	FB_ACCEL_NONE,
	},
};

static struct fb_var_screeninfo stbx25xxfb_var[STBx25xx_FB_COUNT] = {
	[0] = {
		.height		= -1,
		.width		= -1,
		.xres		= SCR_WIDTH,
		.xres_virtual	= SCR_WIDTH,
#ifndef CONFIG_STBx25xx_NTSC
		.yres		= SCR_HEIGHT_PAL,
		.yres_virtual	= SCR_WIDTH,
#else
		.yres		= SCR_HEIGHT_NTSC,
		.yres_virtual	= SCR_HEIGHT_NTSC,
#endif
		.bits_per_pixel	= 8,
		.red		= { 0, 8, 0 },
		.green		= { 0, 8, 0 },
		.blue		= { 0, 8, 0 },
//		.activate 	= FB_ACTIVATE_NOW,
		.pixclock = 290000000,
		.left_margin = 64,
		.right_margin = 64,
		.upper_margin = 32,
		.lower_margin = 32,
		.hsync_len = 64,
		.vsync_len = 64,
		.vmode = FB_VMODE_INTERLACED,
		.sync = FB_SYNC_BROADCAST,
	},
	[1] = {
		.height		= -1,
		.width		= -1,
		.xres		= VID_WIDTH,
		.xres_virtual	= VID_WIDTH,
#ifndef CONFIG_STBx25xx_NTSC
		.yres		= VID_HEIGHT_PAL,
		.yres_virtual	= VID_HEIGHT_PAL,
#else
		.yres		= VID_HEIGHT_NTSC,
		.yres_virtual	= VID_HEIGHT_NTSC,
#endif
		.bits_per_pixel	= 8,
		.vmode		= FB_VMODE_INTERLACED,
		.red		= { 0, 8, 0 },
		.green		= { 0, 8, 0 },
		.blue		= { 0, 8, 0 },
//		.activate 	= FB_ACTIVATE_NOW,
		.pixclock = 290000000,
		.left_margin = 64,
		.right_margin = 64,
		.upper_margin = 32,
		.lower_margin = 32,
		.hsync_len = 64,
		.vsync_len = 64,
		.sync = FB_SYNC_BROADCAST,
	},
};

u32 pseudo_palette[32] = { 0, };

int stbx25xxfb_init(void);

static int osd_disable(struct stbx25xx_video_data *vid, int type)
{
	stbx25xx_video_val reg;
	
	if(mutex_lock_killable(&vid->osd_mode_mutex))
		return -ERESTARTSYS;
	
	reg = get_video_reg(OSD_MODE);
	if(type == FB_TYPE_GRP)
		reg.osd_mode.gle = 0;
	else
		reg.osd_mode.ile = 0;
	set_video_reg(OSD_MODE, reg);
	
	mutex_unlock(&vid->osd_mode_mutex);
	
	return 0;
}

static int osd_enable(struct stbx25xx_video_data *vid, int type)
{
	stbx25xx_video_val reg;
	
	if(mutex_lock_killable(&vid->osd_mode_mutex))
		return -ERESTARTSYS;	
	
	reg = get_video_reg(OSD_MODE);
	if(type == FB_TYPE_GRP)
		reg.osd_mode.gle = 1;
	else
		reg.osd_mode.ile = 1;
	set_video_reg(OSD_MODE, reg);
	
	mutex_unlock(&vid->osd_mode_mutex);
	
	return 0;
}

static int stbx25xxfb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	dprintk("%s\n", __func__);
	
	if(var->xres > VID_WIDTH)
		return -EINVAL;
	
	if(var->xres % 16)
		return -EINVAL;
	
	if(var->yres > VID_HEIGHT_PAL)
		return -EINVAL;
	
	if(var->yres % 16)
		return -EINVAL;
	
	if(var->xres != var->xres_virtual ||
	   var->yres != var->yres_virtual)
		return -EINVAL;

	if(var->xoffset | var->yoffset)
		return -EINVAL;
	
	if(var->bits_per_pixel != 8)
		return -EINVAL;
	
	var->accel_flags = 0;

	return 0;	   	
}

static void rgb2ycbcr(u8 r, u8 g,  u8 b,  u8 *y, u8 *cb, u8 *cr)
{
	// Y  =  0.257*R + 0.504*G + 0.098*B + 16
	// CB = -0.148*R - 0.291*G + 0.439*B + 128
	// CR =  0.439*R - 0.368*G - 0.071*B + 128
	*y  = (u8)((8432*(u32)r + 16425*(u32)g + 3176*(u32)b + 16*32768)>>15);
	*cb = (u8)((128*32768 + 14345*(u32)b - 4818*(u32)r -9527*(u32)g)>>15);
	*cr = (u8)((128*32768 + 14345*(u32)r - 12045*(u32)g-2300*(u32)b)>>15);
}

static int stbx25xxfb_setcolreg(unsigned regno, unsigned red, unsigned green,
			   unsigned blue, unsigned transp,
			   struct fb_info *info)
{
	struct stbx25xx_par *par = info->par;
	
	if (regno >= 256)  /* no. of hw registers */
		return -EINVAL;
	
	red >>= 8;
	green >>= 8;
	blue >>= 8;
	
	dprintk("%s: reg=%d, r=%d, g=%d, b=%d\n", __func__, regno, red, green, blue);

	if (info->fix.visual == FB_VISUAL_PSEUDOCOLOR) {
		u8 y, cb, cr;
		rgb2ycbcr(red, green, blue, &y, &cb, &cr);
		par->pal[regno].y = y >> 2;
		par->pal[regno].cb = cb >> 4;
		par->pal[regno].cr = cr >> 4;
		par->pal[regno].a = CLR_A_BL_EN;
		
		return 0;
	}

	if (info->fix.visual == FB_VISUAL_TRUECOLOR) {
		u32 v;

		if (regno >= 16)
			return -EINVAL;

		v = (red << info->var.red.offset) |
			(green << info->var.green.offset) |
			(blue << info->var.blue.offset) |
			(transp << info->var.transp.offset);

		((u32*)(info->pseudo_palette))[regno] = v;
	}

	return 0;
}

static int stbx25xxfb_set_par(struct fb_info *info)
{
	struct stbx25xx_par *par = info->par;
	struct stbx25xx_osd_hdr *hdr = par->hdr;
	struct stbx25xx_osd_hdr_ext *ext = par->ext;
	stbx25xx_video_val reg;
	int ret = 0, ena;
	
	dprintk("%s\n", __func__);
	
	if(mutex_lock_killable(&par->video->osd_mode_mutex))
		return -ERESTARTSYS;
	
	reg = get_video_reg(OSD_MODE);
	if(par->type == FB_TYPE_GRP) {
		ena = reg.osd_mode.gle;
		reg.osd_mode.gle = 0;
	} else {
		ena = reg.osd_mode.ile;
		reg.osd_mode.ile = 0;
	}
	set_video_reg(OSD_MODE, reg);

	switch(info->var.bits_per_pixel) {
		case 8:
			info->fix.line_length = info->var.xres_virtual * info->var.bits_per_pixel / 8;
			info->fix.visual = FB_VISUAL_PSEUDOCOLOR;
			/* Setup header */
			memset(hdr, 0, sizeof(*hdr));
			hdr->ctu	= 1;
			hdr->left	= (VID_WIDTH - info->var.xres) / 4;
			hdr->width	= info->var.xres / 4;
			hdr->high	= 1;
			if(par->mode == FB_MODE_PAL)
				hdr->top	= (VID_HEIGHT_PAL - info->var.yres) / 4;
			else
				hdr->top	= (VID_HEIGHT_NTSC - info->var.yres) / 4;
			hdr->height	= info->var.yres / 2;
			/* Setup extension */
			memset(ext, 0, sizeof(*ext));
			ext->bitmap	= ((void *)info->screen_base - (void *)hdr) / 4;
			ext->ext2	= 1;
			/* Erase the screen */
			stbx25xxfb_setcolreg(0, 0, 0, 0, 0, info);
			memset(info->screen_base, 0, info->var.xres * info->var.yres);
			break;
		default:
			printk(KERN_ERR "%s: Unsupported color depth (%d bpp)\n", __func__, info->var.bits_per_pixel);
			ret = -EINVAL;
			goto exit;
	}
	
	if(par->type == FB_TYPE_GRP) {
		reg.osd_mode.gle = ena;
	} else {
		reg.osd_mode.ile = ena;
	}
	set_video_reg(OSD_MODE, reg);
	
exit:
	mutex_unlock(&par->video->osd_mode_mutex);

	return ret;
}

static void osd_set_blending(struct fb_info *info, unsigned long arg)
{
	struct stbx25xx_par *par = info->par;
	
	if(arg > 63)
		arg = 63;
	
	par->hdr->blend = arg >> 2;
	par->ext->bl_lvl = arg & 3;
}

static void osd_set_shading(struct fb_info *info, unsigned long arg)
{
	struct stbx25xx_par *par = info->par;
	
	if(arg > 63)
		arg = 63;
	
	par->hdr->shade = arg >> 2;
	par->ext->sh_lvl = arg & 3;
}

static void osd_set_af(struct fb_info *info, unsigned long arg)
{
	struct stbx25xx_par *par = info->par;
	
	if(arg > 3)
		arg = 3;
	
	par->ext->af = arg;
}

static int stbx25xxfb_ioctl(struct fb_info *info, unsigned int cmd,
		       unsigned long arg)
{
	struct stbx25xx_par *par = info->par;
	
	switch(cmd) {
	case STB_FB_HIDE:
		return osd_disable(par->video, par->type);

	case STB_FB_SHOW:
		return osd_enable(par->video, par->type);

	case STB_FB_SETBLEND:
		if(par->type != FB_TYPE_GRP)
			return -EINVAL;
		osd_set_blending(info, arg);
		break;
		
	case STB_FB_SETSHADE:
		if(par->type != FB_TYPE_GRP)
			return -EINVAL;
		osd_set_shading(info, arg);
		break;
		
	case STB_FB_SETAF:
		osd_set_af(info, arg);
		break;
		
	default:
		return -ENOIOCTLCMD;
	}
	
	return 0;
}

    /*
     *  Frame buffer operations
     */

static struct fb_ops stbx25xxfb_ops = {
	.owner		= THIS_MODULE,
	.fb_check_var	= stbx25xxfb_check_var,
	.fb_set_par	= stbx25xxfb_set_par,
	.fb_setcolreg	= stbx25xxfb_setcolreg,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
	.fb_ioctl	= stbx25xxfb_ioctl,
};

/* ------------------------------------------------------------------------- */

    /*
     *  Initialization
     */

int stbx25xx_osd_init(struct stbx25xx_dvb_data *dvb)
{
	struct fb_info *info;
	struct stbx25xx_par *par;
	struct device *device = dvb->dev;
	struct stbx25xx_video_data *vid = &dvb->video;
	int i;	
	u32 addr;
		
	for(i = 0; i < STBx25xx_FB_COUNT; i++) {
		/*
		* Dynamically allocate info and par
		*/
		info = framebuffer_alloc(sizeof(struct stbx25xx_par), device);

		if (!info) {
			return -ENOMEM;
		}

		par = info->par;
		
		par->type = i;
#ifdef CONFIG_STBx25xx_NTSC
		par->mode = FB_MODE_NTSC;
#else
		par->mode = FB_MODE_PAL;
#endif
		par->video = vid;

		switch(par->type) {
			case FB_TYPE_GRP:
				info->screen_base = vid->osdg_data;
				par->mem_size = FB0_SIZE;
				break;
			case FB_TYPE_IMG:
				info->screen_base = vid->osdi_data;
				par->mem_size = FB1_SIZE;
				break;
		}
		
		info->fbops = &stbx25xxfb_ops;
		info->fix = stbx25xxfb_fix[i]; /* this will be the only time stbx25xxfb_fix will be
					* used, so mark it as __devinitdata
					*/
		info->pseudo_palette = &pseudo_palette[i*16]; /* The pseudopalette is an
							* 16-member array
							*/
		info->flags = FBINFO_HWACCEL_NONE;

		/********************* This stage is optional ******************************/
		/*
		* The struct pixmap is a scratch pad for the drawing functions. This
		* is where the monochrome bitmap is constructed by the higher layers
		* and then passed to the accelerator.  For drivers that uses
		* cfb_imageblit, you can skip this part.  For those that have a more
		* rigorous requirement, this stage is needed
		*/

		/* PIXMAP_SIZE should be small enough to optimize drawing, but not
		* large enough that memory is wasted.  A safe size is
		* (max_xres * max_font_height/8). max_xres is driver dependent,
		* max_font_height is 32.
		*/
		
/*#define PIXMAP_SIZE	(VID_WIDTH * (32/8))
		
		info->pixmap.addr = kmalloc(PIXMAP_SIZE, GFP_KERNEL);
		if (!info->pixmap.addr) {
			framebuffer_release(info);
			return -ENOMEM;
		}

		info->pixmap.size = PIXMAP_SIZE;
		info->pixmap.flags = FB_PIXMAP_SYSTEM;
		info->pixmap.scan_align = 4;
		info->pixmap.buf_align = 4;
		info->pixmap.access_align = 32;
*/

		/* This has to been done !!! */	
		fb_alloc_cmap(&info->cmap, 256, 0);
			
		/* 
		* The following is done in the case of having hardware with a static 
		* mode. If we are setting the mode ourselves we don't call this. 
		*/	
		info->var = stbx25xxfb_var[i];
		
		/*
		* Initialize the hardware
		*/
		par->hdr = (void *)info->screen_base;
		par->ext = (void *)info->screen_base + sizeof(*par->hdr);
		par->pal = (void *)info->screen_base + sizeof(*par->hdr) + sizeof(*par->ext);
		par->mem_size -= sizeof(*par->hdr) + sizeof(*par->ext) + (256 * sizeof(*par->pal));
		
		addr = (u32)((void *)info->screen_base + sizeof(*par->hdr) + sizeof(*par->ext) + (256 * sizeof(*par->pal)));
		par->mem_size -= PAGE_ALIGN(addr) - addr;
		addr = PAGE_ALIGN(addr);
		par->mem_size &= ~(4096 - 1);
		info->screen_base = (void *)addr;
		
		/* Needed for mmap() */
		info->fix.smem_start = VIDEO_OSD_BASE + ((void *)par->hdr - (void *)vid->osd_memory) + ((void *)info->screen_base - (void *)par->hdr);
		info->fix.smem_len = par->mem_size;

		
		dprintk("%s: Memory setup:\n-> Header @ %p\n-> Header extension @ %p\n"
		"-> Color table @ %p\n-> Video data: 0x%08x @ %p (%d bytes)\n",
			 info->fix.id, par->hdr, par->ext, par->pal, info->fix.smem_start, info->screen_base, par->mem_size);
		
//		set_video_reg_raw(VID0_GPBASE - i, (SEG2_ADDR + ((void *)par->hdr - (void *)vid->osd_memory)) / 128);
		set_video_reg_raw(VID0_GSLA + i, 0);

		/*
		* For drivers that can...
		*/
		stbx25xxfb_check_var(&info->var, info);
		stbx25xxfb_set_par(info);

		if (register_framebuffer(info) < 0) {
			kfree(info->pixmap.addr);
			framebuffer_release(info);
			return -EINVAL;
		}
		
		osd_enable(vid, i);
		
		printk(KERN_INFO "fb%d: %s frame buffer device (%s mode)\n", info->node,
			info->fix.id, (par->mode == FB_MODE_PAL) ? "PAL" : "NTSC");
		
		vid->fb_info[i] = info;
	}
		
	return 0;
}

    /*
     *  Cleanup
     */
void stbx25xx_osd_exit(struct stbx25xx_dvb_data *dvb)
{
	struct stbx25xx_video_data *vid = &dvb->video;
	struct fb_info *info;
	int i;
	
	for(i = 0; i < STBx25xx_FB_COUNT; i++) {
		info = vid->fb_info[i];
		
		osd_disable(vid, i);
		
		if (info) {
			unregister_framebuffer(info);
			fb_dealloc_cmap(&info->cmap);
			/* ... */
			framebuffer_release(info);
		}
	}
}
