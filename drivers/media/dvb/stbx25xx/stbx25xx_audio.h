/*
 * This file is part of linux driver the digital TV devices equipped with IBM STBx25xx SoC
 *
 * stbx25xx_audio.h - header file for audio hardware.
 *
 * see stbx25xx.c for copyright information.
 */

#ifndef __STBx25xx_AUDIO_H__
#define __STBx25xx_AUDIO_H__

#include "stbx25xx_audio_val.h"

/* Interrupts */
#define AUDIO_IRQ	2

/* Memory map */
#define AUDIO_DATA_BASE		0xA0400000	/* MPEG Audio Decoder Memory */
#define AUDIO_DATA_SIZE		0x00400000	/* Second 4 MB of the second memory bank */

/* Registers */
#define AUD_CTRL0		0x01a0
#define AUD_CTRL1		0x01a1
#define AUD_CTRL2		0x01a2
#define AUD_CMD			0x01a3
#define AUD_ISR			0x01a4
#define AUD_IMR			0x01a5
#define AUD_DSR			0x01a6
#define AUD_STC			0x01a7
#define AUD_CSR			0x01a8
#define AUD_QAR2		0x01a9
#define AUD_PTS			0x01aa
#define AUD_TONE_GEN		0x01ab
#define AUD_QLR2		0x01ac
#define AUD_ANC			0x01ad
#define AUD_STREAM_ID		0x01ae
#define AUD_QAR			0x01af
#define AUD_DSP_STAT		0x01b0
#define AUD_QLR			0x01b1
#define AUD_DSP_CTRL		0x01b2
#define AUD_WLR2		0x01b3
#define AUD_MDR			0x01b4
#define AUD_WAR			0x01b5
#define AUD_SEG1		0x01b6
#define AUD_SEG2		0x01b7
#define AUD_RB_FULL		0x01b8
#define AUD_ATT_FT		0x01b9
#define AUD_ATT_RR		0x01ba
#define AUD_ATT_CR		0x01bb
#define AUD_SEG3		0x01bc
#define AUD_OFFS		0x01bd
#define AUD_WLR			0x01be
#define AUD_WAR2		0x01bf

#endif
