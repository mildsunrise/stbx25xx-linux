/*
 * linux/drivers/media/dvb/stbx25xxfb.c - Framebuffer driver for IBM STBx25xx (part of the DVB driver)
 *
 *  Created 1 Mar 2009 by Tomasz Figa <tomasz.figa@gmail.com>
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 */

#ifdef CONFIG_FB

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

struct stbx25xx_osd_hdr {
	unsigned ctu	:1;
	union {
		struct {
			unsigned width	:7;
			unsigned shade	:4;
			unsigned	:1;
		} no_hc;
		
		struct {
			unsigned width	:8;
			unsigned shade	:4;
		} hc;
	};
	unsigned high	:1;
	unsigned top	:9;
	unsigned left	:9;
	unsigned link	:16;
	unsigned cr	:1;
	unsigned height	:9;
	unsigned pr	:1;
	unsigned blend	:4;
	unsigned transp	:1;
} __attribute__((packed));

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
} __attribute__((packed));

struct stbx25xx_color_entry {
	unsigned y	:6;
	unsigned cb	:4;
	unsigned cr	:4;
	unsigned blend	:1;
	unsigned fill	:1;
} __attribute__((packed));

/*
 * Driver data
 */

#define FB_TYPE_GRP	0
#define FB_TYPE_IMG	1
struct stbx25xx_par {
	struct	fb_info 		*info;
	int				type;
	struct stbx25xx_osd_hdr		*hdr;
	struct stbx25xx_osd_hdr_ext	*ext;
	struct stbx25xx_color_entry	*pal;
	size_t				mem_size;
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
		.xres		= VID_WIDTH,
#ifndef CONFIG_STBx25xx_NTSC
		.yres		= VID_HEIGHT_PAL,
		.yres_virtual	= VID_HEIGHT_PAL,
#else
		.yres		= VID_HEIGHT_NTSC,
		.yres_virtual	= VID_HEIGHT_NTSC,
#endif
		.bits_per_pixel	= 8,
	},
	[1] = {
		.xres		= VID_WIDTH,
#ifndef CONFIG_STBx25xx_NTSC
		.yres		= VID_HEIGHT_PAL,
		.yres_virtual	= VID_HEIGHT_PAL,
#else
		.yres		= VID_HEIGHT_NTSC,
		.yres_virtual	= VID_HEIGHT_NTSC,
#endif
		.bits_per_pixel	= 8,
	},
};

u32 pseudo_palette[16] = { 0, };

int stbx25xxfb_init(void);

struct mutex osd_mode_mutex;

static int stbx25xxfb_open(struct fb_info *info, int user)
{
	stbx25xx_video_val reg;
	struct stbx25xx_par *par = info->par;
	
	if(mutex_lock_interruptible(&osd_mode_mutex))
		return -EAGAIN;
	
	reg = get_video_reg(OSD_MODE);
	if(par->type == FB_TYPE_GRP)
		reg.osd_mode.gle = 1;
	else
		reg.osd_mode.ile = 1;
	set_video_reg(OSD_MODE, reg);
	
	mutex_unlock(&osd_mode_mutex);
	
	return 0;
}

static int stbx25xxfb_release(struct fb_info *info, int user)
{
	stbx25xx_video_val reg;
	struct stbx25xx_par *par = info->par;
	
	if(mutex_lock_interruptible(&osd_mode_mutex))
		return -EAGAIN;
	
	reg = get_video_reg(OSD_MODE);
	if(par->type == FB_TYPE_GRP)
		reg.osd_mode.gle = 0;
	else
		reg.osd_mode.ile = 0;
	set_video_reg(OSD_MODE, reg);
	
	mutex_unlock(&osd_mode_mutex);
	
	return 0;
}

static int stbx25xxfb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	*var = info->var;
	return 0;	   	
}

static int stbx25xxfb_set_par(struct fb_info *info)
{
	struct stbx25xx_par *par = info->par;
	struct stbx25xx_osd_hdr *hdr = par->hdr;
	struct stbx25xx_osd_hdr_ext *ext = par->ext;

	switch(info->var.bits_per_pixel) {
		case 8:
			/* Setup header */
			memset(hdr, 0, sizeof(*hdr));
			hdr->ctu	= 1;
			hdr->hc.width	= info->var.xres / 4;
			hdr->high	= 1;
			hdr->height	= info->var.yres / 2;
			/* Setup extension */
			memset(ext, 0, sizeof(*ext));
			ext->bitmap	= ((void *)info->screen_base - (void *)hdr) / 4;
			ext->ext2	= 1;
			/* Erase the screen */
			memset(info->screen_base, 0, info->var.xres * info->var.yres);
			/* Erase the color table */
			memset(par->pal, 0, 256 * sizeof(*par->pal));
			break;
		default:
			printk(KERN_ERR "%s: Unsupported color depth (%d bpp)\n", __func__, info->var.bits_per_pixel);
			return -EINVAL;
	}
	
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
	/*
	* Program hardware... do anything you want with transp
	*/

	/* grayscale works only partially under directcolor */
	if (info->var.grayscale) {
		/* grayscale = 0.30*R + 0.59*G + 0.11*B */
		red = green = blue = (red * 77 + green * 151 + blue * 28) >> 8;
	}

	/*
	* This is the point where the function feeds the color to the hardware
	* palette after converting the colors to something acceptable by
	* the hardware. Note, only FB_VISUAL_DIRECTCOLOR and
	* FB_VISUAL_PSEUDOCOLOR visuals need to write to the hardware palette.
	* If you have code that writes to the hardware CLUT, and it's not
	* any of the above visuals, then you are doing something wrong.
	*/
	if (info->fix.visual == FB_VISUAL_DIRECTCOLOR ||
		info->fix.visual == FB_VISUAL_PSEUDOCOLOR) {
		u8 y, cb, cr;
		rgb2ycbcr(red, green, blue, &y, &cb, &cr);
		par->pal[regno].y = y >> 2;
		par->pal[regno].cb = cb >> 4;
		par->pal[regno].cr = cr >> 4;
	}

	/* This is the point were you need to fill up the contents of
	* info->pseudo_palette. This structure is used _only_ by fbcon, thus
	* it only contains 16 entries to match the number of colors supported
	* by the console. The pseudo_palette is used only if the visual is
	* in directcolor or truecolor mode.  With other visuals, the
	* pseudo_palette is not used. (This might change in the future.)
	*
	* The contents of the pseudo_palette is in raw pixel format.  Ie, each
	* entry can be written directly to the framebuffer without any conversion.
	* The pseudo_palette is (void *).  However, if using the generic
	* drawing functions (cfb_imageblit, cfb_fillrect), the pseudo_palette
	* must be casted to (u32 *) _regardless_ of the bits per pixel. If the
	* driver is using its own drawing functions, then it can use whatever
	* size it wants.
	*/
	if (info->fix.visual == FB_VISUAL_TRUECOLOR ||
		info->fix.visual == FB_VISUAL_DIRECTCOLOR) {
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

#if 0
/**
 *      stbx25xxfb_blank - NOT a required function. Blanks the display.
 *      @blank_mode: the blank mode we want. 
 *      @info: frame buffer structure that represents a single frame buffer
 *
 *      Blank the screen if blank_mode != FB_BLANK_UNBLANK, else unblank.
 *      Return 0 if blanking succeeded, != 0 if un-/blanking failed due to
 *      e.g. a video mode which doesn't support it.
 *
 *      Implements VESA suspend and powerdown modes on hardware that supports
 *      disabling hsync/vsync:
 *
 *      FB_BLANK_NORMAL = display is blanked, syncs are on.
 *      FB_BLANK_HSYNC_SUSPEND = hsync off
 *      FB_BLANK_VSYNC_SUSPEND = vsync off
 *      FB_BLANK_POWERDOWN =  hsync and vsync off
 *
 *      If implementing this function, at least support FB_BLANK_UNBLANK.
 *      Return !0 for any modes that are unimplemented.
 *
 */
static int stbx25xxfb_blank(int blank_mode, struct fb_info *info)
{
	/* ... */
	return 0;
}
#endif

    /*
     *  Frame buffer operations
     */

static struct fb_ops stbx25xxfb_ops = {
	.owner		= THIS_MODULE,
	.fb_open	= stbx25xxfb_open,
	.fb_release	= stbx25xxfb_release,
	.fb_check_var	= stbx25xxfb_check_var,
	.fb_set_par	= stbx25xxfb_set_par,
	.fb_setcolreg	= stbx25xxfb_setcolreg,
//	.fb_blank	= stbx25xxfb_blank,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
//	.fb_mmap	= stbx25xxfb_mmap,
};

/* ------------------------------------------------------------------------- */

    /*
     *  Initialization
     */

int stbx25xx_osd_init(struct stbx25xx_dvb_dev *dvb)
{
	struct fb_info *info;
	struct stbx25xx_par *par;
	struct device *device = dvb->dev; /* or &pdev->dev */
	int i;	
	
	mutex_init(&osd_mode_mutex);
	
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

		switch(par->type) {
			case FB_TYPE_GRP:
				info->screen_base = dvb->osdg_data;
				par->mem_size = FB0_SIZE;
				break;
			case FB_TYPE_IMG:
				info->screen_base = dvb->osdi_data;
				par->mem_size = FB1_SIZE;
				break;
		}
		
		info->fbops = &stbx25xxfb_ops;
		info->fix = stbx25xxfb_fix[i]; /* this will be the only time stbx25xxfb_fix will be
					* used, so mark it as __devinitdata
					*/
		info->pseudo_palette = pseudo_palette; /* The pseudopalette is an
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
		
#define PIXMAP_SIZE	(VID_WIDTH * (32/8))
		
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
		
		info->screen_base = (void *)info->screen_base + sizeof(*par->hdr) + sizeof(*par->ext) + (256 * sizeof(*par->pal));
		par->mem_size -= sizeof(*par->hdr) + sizeof(*par->ext) + (256 * sizeof(*par->pal));
		
		set_video_reg_raw(VID0_GSLA + i, ((void *)par->hdr - (void *)info->screen_base) >> 5);

		/*
		* For drivers that can...
		*/
		stbx25xxfb_set_par(info);

		if (register_framebuffer(info) < 0) {
			kfree(info->pixmap.addr);
			framebuffer_release(info);
			return -EINVAL;
		}
		
		printk(KERN_INFO "fb%d: %s frame buffer device\n", info->node,
			info->fix.id);
		
		dvb->fb_info[i] = info;
	}
		
	return 0;
}

    /*
     *  Cleanup
     */
void stbx25xx_osd_exit(struct stbx25xx_dvb_dev *dvb)
{
	struct fb_info *info;
	int i;
	
	for(i = 0; i < STBx25xx_FB_COUNT; i++) {
		info = dvb->fb_info[i];
		
		if (info) {
			unregister_framebuffer(info);
			fb_dealloc_cmap(&info->cmap);
			/* ... */
			framebuffer_release(info);
		}
	}
}

#endif
