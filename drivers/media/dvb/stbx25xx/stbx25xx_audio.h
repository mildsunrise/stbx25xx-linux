/*
 * This file is part of linux driver the digital TV devices equipped with IBM STBx25xx SoC
 *
 * stbx25xx_video.h - header file for demux hardware.
 *
 * see stbx25xx.c for copyright information.
 */

#ifndef __STBx25xx_AUDIO_H__
#define __STBx25xx_AUDIO_H__

//#include "stbx25xx_video_val.h"

/* Interrupts */
#define AUDIO_IRQ	2

/* Memory map */
#define AUDIO_DATA_BASE		0xA0400000	/* MPEG Audio Decoder Memory */
#define AUDIO_DATA_SIZE		0x00400000	/* Second 4 MB of the second memory bank */

#endif
