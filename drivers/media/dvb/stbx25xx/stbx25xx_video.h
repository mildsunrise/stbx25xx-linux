/*
 * This file is part of linux driver the digital TV devices equipped with IBM STBx25xx SoC
 *
 * stbx25xx_video.h - header file for demux hardware.
 *
 * see stbx25xx.c for copyright information.
 */

#ifndef __STBx25xx_VIDEO_H__
#define __STBx25xx_VIDEO_H__

#include <asm/dcr.h>
#include "stbx25xx_video_val.h"

/* Interrupts */
// #define VIDEO_IRQ			3
#define STBx25xx_VIDEO_IRQ_COUNT	32

/* Misc */
#define VIDEO_CMD_TIMEOUT_MS	100
#define VBI_LINES		16	/* VBI uses up to 16 video lines */
#define CLIP_COUNT		2	/* Clip mode uses two buffers working in ping-pong mode */
#define STBx25xx_VIDEO_CAPS	(VIDEO_CAP_MPEG1 | VIDEO_CAP_MPEG2)

/* Memory map - physical addresses of memory areas used by video decoder */
#define VIDEO_FB_BASE		0xA0000000	/* Video Framebuffers Memory */
#define VIDEO_FB_SIZE		0x200000	/* First 2 MB of the second memory bank */
#define VIDEO_MPEG_BASE		0xA0200000	/* MPEG Video Decoder Memory */
#define VIDEO_MPEG_SIZE		0x200000	/* Second 2 MB of the second memory bank */
#define VIDEO_OSD_BASE		0x03E00000	/* OSD Framebuffer Memory */
#define VIDEO_OSD_SIZE		0x200000	/* Last 2 MB of the first memory bank */

/* Video decoder segments - only SEG0 to SEG2 are used */
#define SEG0_BASE		VIDEO_FB_BASE	/* FB data */
#define SEG0_SIZE		1		/* 2 MegaBytes */
#define SEG0_ADDR		0x00000000	/* Beginning of decoder's address space */
#define SEG1_BASE		VIDEO_MPEG_BASE	/* MPEG data */
#define SEG1_SIZE		1		/* 2 MegaBytes */
#define SEG1_ADDR		(SEG0_ADDR + (0x100000 << SEG0_SIZE))	/* After SEG0 */
#define SEG2_BASE		VIDEO_OSD_BASE	/* OSD data */
#define SEG2_SIZE		1		/* 2 MegaBytes */
#define SEG2_ADDR		(SEG1_ADDR + (0x100000 << SEG1_SIZE))	/* After SEG1 */

/* Offset within Video memory for calculating addresses in both address spaces */
#define VFB_OFFSET		0x00000000	/* Start of VIDEO_FB_BASE */
#define VFB_SIZE		(2*1024*1024)	/* 2 MegaBytes */
#define PAL_OFFSET		(0x16900)
#define NTSC_OFFSET		(0x21d00)

/* Dynamically calculated offsets
 * to video framebuffers are stored
 * in array utilising followin indices */
#define BUF0_LUM		0	/* B-Frame buffer luma */
#define BUF0_CHR		1	/* B-Frame buffer chroma */
#define BUF1_LUM		2	/* Reference buffer 0 luma */
#define BUF1_CHR		3	/* Reference buffer 0 chroma */
#define BUF2_LUM		4	/* Reference buffer 1 luma */
#define BUF2_CHR		5	/* Reference buffer 1 chroma */

/* Offsets within OSD memory for calculating addresses in both address spaces */
#define FB0_OFFSET		0x00000000	/* Start of VIDEO_OSD_BASE - Graphic Plane */
#define FB0_SIZE		(1024*1024)	/* 1 MegaByte */
#define FB1_OFFSET		(FB0_SIZE)	/* Just after FB0 - Image Plane */
#define FB1_SIZE		(896*1024)	/* 896 KiloBytes */
#define CUR_OFFSET		(FB0_SIZE + FB1_SIZE)	/* Just after FB1 - Cursor Plane */
#define CUR_SIZE		(128*1024)	/* 128 KiloBytes */

/* Offsets within MPEG memory for calculating addresses in both address spaces */
#define	USER_OFFSET		0x00000000	/* Start of VIDEO_MPEG_BASE */
#define USER_SIZE		512		/* 512 bytes of User Data */
#define VBI0_OFFSET		(USER_SIZE)	/* Just after User Data */
#define VBI0_SIZE		(VBI_LINES*1440)	/* First VBI data */
#define VBI1_OFFSET		(USER_SIZE + VBI0_SIZE) /* After first VBI */
#define VBI1_SIZE		(VBI_LINES*1440)	/* Second VBI data */
#define RB_OFFSET		(USER_SIZE + VBI0_SIZE + VBI1_SIZE)	/* Rate buffer */
#define RB_SIZE			(VIDEO_MPEG_SIZE - USER_SIZE - VBI0_SIZE - VBI1_SIZE)	/* Rest of MPEG memory */

/* Video format definitions - self explanatory */
#define VID_WIDTH		720
#define VID_HEIGHT_PAL		576
#define VID_HEIGHT_NTSC		480
#define SCR_WIDTH		640
#define SCR_HEIGHT_PAL		480
#define SCR_HEIGHT_NTSC		400

/* Registers */
#define CICVCR			0x0033
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

/* Firmware commands */
#define CMD_CHAIN		1
#define CMD_PLAY		(0 << 1)
#define CMD_PAUSE		(1 << 1)
#define CMD_SINGLE		(2 << 1)
#define CMD_FAST_FWD		(3 << 1)
#define CMD_SLOW		(4 << 1)
#define CMD_MOVE		(5 << 1)
#define CMD_PANSCAN		(6 << 1)
#define CMD_FREEZE		(7 << 1)
#define CMD_RB_RST		(8 << 1)
#define CMD_CFG			(9 << 1)
#define CMD_SRV_INIT		(10 << 1)
#define CMD_SRV_DISP		(11 << 1)
#define CMD_SRV_DEL		(12 << 1)
#define CMD_FRM_SW		(13 << 1)
#define CMD_STILL		(14 << 1)
#define CMD_SKIP		(15 << 1)

/* Video encoder */
#define DENC0_CR1		0x131
#define DENC1_CR1		0x2e1
#define DENC0_RLSR              0x13D
#define DENC0_VSR               0x13F
#define DENC_NTSC		1
#define DENC_PAL		2
#define OUTFMT_CVBS		0
#define OUTFMT_RGB		1
#define OUTFMT_COMP		2
#define OUTFMT_SVIDEO		3

/* IOCTLs */
#define STB_FB_SETBLEND		_IO('o', 0)
#define STB_FB_HIDE		_IO('o', 2)
#define STB_FB_SHOW		_IO('o', 3)
#define STB_FB_SETSHADE		_IO('o', 6)
#define STB_FB_SETAF		_IO('o', 7)
#define STB_VID_SETAF		_IO('o', 64)

/* Register accessors */
#define set_video_reg(reg, val) \
	mtdcr(reg, val.raw)
	
#define set_video_reg_raw(reg, val) \
	mtdcr(reg, val)
	
#define get_video_reg(reg) \
	((stbx25xx_video_val)mfdcr(reg))
	
#define get_video_reg_raw(reg) \
	mfdcr(reg)

#endif