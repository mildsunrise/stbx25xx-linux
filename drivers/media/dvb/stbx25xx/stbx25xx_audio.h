/*
 * This file is part of linux driver the digital TV devices equipped with IBM STBx25xx SoC
 *
 * stbx25xx_audio.h - header file for audio hardware.
 *
 * see stbx25xx.c for copyright information.
 */

#ifndef __STBx25xx_AUDIO_H__
#define __STBx25xx_AUDIO_H__

#include <asm/dcr.h>
#include "stbx25xx_audio_val.h"

/* Misc */
#define IRQ_BIT(bit)			(0x80000000UL >> (bit))
#define STBx25xx_AUDIO_IRQ_COUNT	32
#define AUDIO_PROC_IRQS_NAME		"audio_interrupts"
#define AUDIO_SEGMENT_SHIFT		(7)
#define STBx25xx_AUDIO_CAPS		(AUDIO_CAP_MP1 | AUDIO_CAP_MP2 | AUDIO_CAP_MP3)

/* Memory map */
#define AUDIO_DATA_BASE		(0xA0600000)	/* MPEG Audio Decoder Memory */
#define AUDIO_DATA_SIZE		(0x00200000)	/* 6-8MB of the second memory bank */

/* Segment 1 */
#define AUDIO_DAB1_OFFSET	(0x00000000)
#define AUDIO_DAB1_SIZE		(0x8000)
#define AUDIO_DAB2_OFFSET	(AUDIO_DAB1_OFFSET + AUDIO_DAB1_SIZE)
#define AUDIO_DAB2_SIZE		(0x8000)
#define AUDIO_AWA_OFFSET	(AUDIO_DAB2_OFFSET + AUDIO_DAB2_SIZE)
#define AUDIO_AWA_SIZE		(0x20000)

/* Segment 3 */
#define AUDIO_RB_OFFSET		(AUDIO_AWA_OFFSET + AUDIO_AWA_SIZE)
#define AUDIO_RB_SIZE		(0x10000)

/* Segment 2 */
#define AUDIO_PTS_OFFSET	(AUDIO_RB_OFFSET + AUDIO_RB_SIZE)
#define AUDIO_PTS_SIZE		(0x20000)

/* Memory segments */
#define SEG1_BASE		(AUDIO_DATA_BASE + AUDIO_DAB1_OFFSET)
#define SEG1_SIZE		(AUDIO_DAB1_SIZE + AUDIO_DAB2_SIZE + AUDIO_AWA_SIZE)
#define SEG2_BASE		(AUDIO_DATA_BASE + AUDIO_PTS_OFFSET)
#define SEG2_SIZE		(AUDIO_PTS_SIZE)
#define SEG3_BASE		(AUDIO_DATA_BASE + AUDIO_RB_OFFSET)
#define SEG3_SIZE		(AUDIO_RB_SIZE)

/* Clip mode buffers */
#define AUDIO_CLIP_OFFSET	(AUDIO_PTS_OFFSET + AUDIO_PTS_SIZE)
#define AUDIO_CLIP_SIZE		(AUDIO_DATA_SIZE - AUDIO_CLIP_OFFSET)
#define AUDIO_CLIP_ADDR		(AUDIO_DATA_BASE + AUDIO_CLIP_OFFSET)
#define AUDIO_CLIP_BLOCK_SIZE	(0x10000)
#define AUDIO_ALIGNMENT_MASK	((2 * AUDIO_CLIP_BLOCK_SIZE) - 1)

#if (AUDIO_CLIP_BLOCK_SIZE & ~PAGE_MASK)
#error Clip mode block size must be a multiple of 4KB
#endif

#if (AUDIO_DATA_BASE & AUDIO_ALIGNMENT_MASK)
#error Audio memory address unaligned!
#endif

#if (AUDIO_DATA_SIZE & AUDIO_ALIGNMENT_MASK)
#error Audio memory size unaligned!
#endif

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

/* Interrupts */
#define AUDIO_SYNC_IRQ		(12)
#define AUDIO_CCC_IRQ		(16)
#define AUDIO_RTBC_IRQ		(17)
#define AUDIO_BTF_IRQ		(18)
#define AUDIO_BTE_IRQ		(19)
#define AUDIO_AMSI_IRQ		(20)
#define AUDIO_PE_IRQ		(21)
#define AUDIO_BE_IRQ		(22)
#define AUDIO_BF_IRQ		(23)
#define AUDIO_PSE_IRQ		(24)
#define AUDIO_PTO_IRQ		(25)
#define AUDIO_ADO_IRQ		(26)
#define AUDIO_ADD_IRQ		(27)
#define AUDIO_CM2_IRQ		(30)
#define AUDIO_CM_IRQ		(31)

/* Register accessors */
#define set_audio_reg(reg, val) \
	mtdcr(reg, val.raw)

#define set_audio_reg_raw(reg, val) \
	mtdcr(reg, val)

#define get_audio_reg(reg) \
	((stbx25xx_audio_val)mfdcr(reg))

#define get_audio_reg_raw(reg) \
	mfdcr(reg)

#endif
