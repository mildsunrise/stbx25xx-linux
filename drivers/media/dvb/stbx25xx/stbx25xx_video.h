/*
 * This file is part of linux driver the digital TV devices equipped with IBM STBx25xx SoC
 *
 * stbx25xx_video.h - header file for demux hardware.
 *
 * see stbx25xx.c for copyright information.
 */

#ifndef __STBx25xx_VIDEO_H__
#define __STBx25xx_VIDEO_H__

#include "stbx25xx_video_val.h"

/* Interrupts */
#define VIDEO_IRQ	3

/* Memory map */
#define VIDEO_MPEG_BASE		0xA0000000	/* MPEG Video Decoder Memory */
#define VIDEO_MPEG_SIZE		0x00400000	/* First 4 MB of the second memory bank */
#define VIDEO_FB_BASE		0x03E00000	/* OSD Framebuffer Memory */
#define VIDEO_FB_SIZE		0x00200000	/* Last 2 MB of the first memory bank */

/* Registers */
#define CLKGCRST		0x0122
#define VIDEO_CNTL		0x0140
#define VIDEO_MODE		0x0141
#define SYNC_STC0		0x0142
#define SYNC_STC1		0x0143
#define SYNC_PTS0		0x0144
#define SYNC_PTS1		0x0145
#define FIFO_DATA		0x0146
#define FIFO_STAT		0x0147
#define CMD			0x0148
#define CMD_DATA		0x0149
#define CMD_STAT		0x014a
#define CMD_ADDR		0x014b
#define PROC_IADDR		0x014c
#define PROC_IDATA		0x014d
#define OSD_MODE		0x0151
#define VIDEO_INT		0x0152
#define MASK			0x0153
#define DISP_MODE		0x0154
#define DISP_DLY		0x0155
#define VBI_CNTL		0x0156
#define TTX_CNTL		0x0157
#define DISP_BORDER		0x0158
#define VID0_GSLA		0x0159
#define VID0_ISLA		0x015a
#define RB_THRESHOLD		0x015b
#define VID0_CSLA		0x015c
#define PTS_DELTA		0x015e
#define PTS_CNTL		0x015f
#define WRT_PROT		0x0165
#define CLIP_ADDR		0x0167
#define CLIP_LEN		0x0168
#define BLK_SIZE		0x0169
#define SRC_ADDR		0x016a
#define USERDATA_BASE		0x016b
#define VBI_BASE		0x016c
#define VID0_IPBASE		0x016d
#define VID0_GPBASE		0x016e
#define RATEBUF_BASE		0x016f
#define MEM_ADDR		0x0170
#define MEM_DATA		0x0171
#define MEM_CTRL		0x0172
#define CLIP_WRK_ADDR		0x0173
#define CLIP_WRK_LEN		0x0174
#define MEM_SEG0		0x0175
#define MEM_SEG1		0x0176
#define MEM_SEG2		0x0177
#define MEM_SEG3		0x0178
#define FB_BASE			0x0179
#define VID0_CPBASE		0x017a
#define VID0_LBOXTB		0x017b
#define SCALE_BORDER		0x017d
#define VID0_ZOFFS		0x017e
#define RATEBUF_SIZE		0x017f

#endif