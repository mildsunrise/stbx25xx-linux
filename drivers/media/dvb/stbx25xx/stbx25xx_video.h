/*
 * This file is part of linux driver the digital TV devices equipped with IBM STBx25xx SoC
 *
 * stbx25xx_video.h - header file for demux hardware.
 *
 * see stbx25xx.c for copyright information.
 */

#ifndef __STBx25xx_VIDEO_H__
#define __STBx25xx_VIDEO_H__

//#include "stbx25xx_video_val.h"

/* Interrupts */
#define VIDEO_IRQ	3

/* Memory map */
#define VIDEO_MPEG_BASE		0xA0000000	/* MPEG Video Decoder Memory */
#define VIDEO_MPEG_SIZE		0x00400000	/* First 4 MB of the second memory bank */
#define VIDEO_FB_BASE		0x03E00000	/* OSD Framebuffer Memory */
#define VIDEO_FB_SIZE		0x00200000	/* Last 2 MB of the first memory bank */

#endif