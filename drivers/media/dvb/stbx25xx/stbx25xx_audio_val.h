/*
 * This file is part of linux driver the digital TV devices equipped with IBM STBx25xx SoC
 *
 * stbx25xx_audio_val.h - header file for audio hardware.
 *
 * see stbx25xx.c for copyright information.
 */

#ifndef __STBx25xx_AUDIO_VAL_H__
#define __STBx25xx_AUDIO_VAL_H__

#if defined(__LITTLE_ENDIAN)
	#error Something went wrong, because STBx25xx is Big Endian only and system endianess is set to Little Endian!
#elseif !defined(__BIG_ENDIAN)
	#error Something went wrong, because no endianess is defined!
#endif

typedef union {
	u32 raw;
	
	struct {
		unsigned	:16;
		unsigned sm	:2;
		unsigned	:2;
		unsigned mix	:1;
		unsigned adiso	:1;
		unsigned as	:1;
		unsigned ep	:1;
		unsigned cm	:1;
		unsigned vmd	:1;
		unsigned	:2;
		unsigned md	:1;
		unsigned ei	:1;
		unsigned type	:2;
	} ctrl0;
	
	struct {
		unsigned	:17;
		unsigned drm	:2;
		unsigned	:2;
		unsigned sm	:1;
		unsigned	:4;
		unsigned dss	:1;
		unsigned dce	:1;
		unsigned ljm	:1;
		unsigned rjm	:1;
		unsigned bcr	:1;
		unsigned dcm	:1;
	} ctrl1;
	
	struct {
		unsigned	:16;
		unsigned hd	:1;
		unsigned mute	:1;
		unsigned prog2	:1;
		unsigned ip	:1;
		unsigned id	:1;
		union {
			unsigned raw	:9;
			struct {
				unsigned	:6;
				unsigned dualch	:1;
				unsigned fsz_en	:1;
				unsigned crc_en	:1;
			} mpeg;
			struct {
				unsigned mix	:3;
				unsigned ll	:1;
				unsigned hl	:1;
				unsigned op	:2;
				unsigned km	:2;
			} ac3;
			struct {
				unsigned	:6;
				unsigned le	:1;
				unsigned 	:2;
			} upcm;
		} prog3;
#define AUD_STRMTP_AC3		0x00000000
#define AUD_STRMTP_MPEG		0x00000001
#define AUD_STRMTP_PCM		0x00000003
#define AUD_STRMTP_DTS		0x00000002
#define AUD_STRMTP_LPCM		0x00000001
		unsigned strmtp	:2;
	} ctrl2;
	
	struct {
		unsigned	:12;
		unsigned sync	:1;
		unsigned	:3;
		unsigned ccc	:1;
		unsigned rtbc	:1;
		unsigned btf	:1;
		unsigned bte	:1;
		unsigned amsi	:1;
		unsigned pe	:1;
		unsigned be	:1;
		unsigned bf	:1;
		unsigned pse	:1;
		unsigned pto	:1;
		unsigned ado	:1;
		unsigned add	:1;
		unsigned	:2;
		unsigned cm2	:1;
		unsigned cm	:1;
	} irq;
	
	struct {
		unsigned	:16;
		unsigned ccp	:1;
		unsigned tbcp	:1;
		unsigned ams	:1;
		unsigned	:8;
		unsigned bne	:1;
		unsigned	:2;
		unsigned hv	:1;
		unsigned	:1;
	} dsr;
	
	struct {
		unsigned cm	:1;
		unsigned am	:1;
		unsigned cr	:1;
		unsigned preemp	:3;
		unsigned	:2;
		unsigned cc	:8;
		unsigned	:8;
		unsigned sfq	:2;
		unsigned	:2;
		unsigned hval	:1;
		unsigned crs	:1;
		unsigned croe	:1;
		unsigned cro	:1;
	} csr;
	
	struct {
		unsigned	:12;
		unsigned tr	:1;
		unsigned ba	:3;
		unsigned	:3;
		unsigned bd	:5;
		unsigned	:1;
		unsigned fi	:7;
	} tone_gen;
	
	struct {
		unsigned bv	:1;
		unsigned eoc	:1;
		unsigned sb	:1;
		unsigned	:8;
		unsigned len	:21;
	} qlr;
	
	struct {
		unsigned	:12;
		unsigned pdl	:4;
		unsigned stmie	:8;
		unsigned stmm	:8;
	} stream_id;
	
	struct {
		unsigned	:29;
		unsigned synci	:1;
		unsigned dcd	:1;
		unsigned tgc	:1;
	} dsp_stat;
	
	struct {
		unsigned	:10;
		unsigned mixvol	:6;
		unsigned	:1;
		unsigned mixch	:1;
		unsigned mixwd	:2;
		unsigned mixfs	:4;
		unsigned upcms	:1;
		unsigned pcmch	:1;
		unsigned pcmwd	:2;
		unsigned pcmfs	:4;
	} dsp_ctrl;
	
	struct {
		unsigned	:18;
		unsigned left	:6;
		unsigned	:2;
		unsigned right	:6;
	} att;
	
	struct {
		unsigned	:10;
		unsigned dab2	:6;
		unsigned	:2;
		unsigned dab1	:6;
		unsigned	:2;
		unsigned awa	:6;
	} offsets;
} stbx25xx_audio_val;

#endif
