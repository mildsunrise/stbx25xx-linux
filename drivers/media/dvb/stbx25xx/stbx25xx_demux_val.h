/*
 * This file is part of linux driver the digital TV devices equipped with IBM STBx25xx SoC
 *
 * stbx25xx_demux_val.h - header file for demux hardware.
 *
 * see stbx25xx.c for copyright information.
 */

#ifndef __STBx25xx_DEMUX_VAL_H__
#define __STBx25xx_DEMUX_VAL_H__

#if defined(__LITTLE_ENDIAN)
	#error Something went wrong, because STBx25xx is Big Endian only and endianess is set to Little Endian!
#elseif !defined(__BIG_ENDIAN)
	#error Something went wrong, because no endianess is defined!
#endif

typedef union {
	u32 raw;
	
	struct {
		unsigned	:8;
		unsigned de	:1;
		unsigned	:2;
		unsigned vpu	:1;
		unsigned apu	:1;
		unsigned apwma	:1;
		unsigned tstoe	:1;
		unsigned	:5;
		unsigned tsclkp	:1;
		unsigned tsdp	:1;
		unsigned	:2;
		unsigned tsvp	:1;
		unsigned	:1;
		unsigned syncd	:2;
		unsigned bb	:1;
		unsigned syncl	:3;
	} config1;
	
	struct {
		unsigned	:25;
		unsigned sbe	:1;
		unsigned pbe	:1;
		unsigned	:1;
		unsigned se	:1;
		unsigned sdop	:1;
		unsigned	:1;
		unsigned swr	:1;
	} control1;
	
	struct {
		unsigned	:17;
		unsigned fpcr	:1;
		unsigned	:1;
		unsigned mpfm	:1;
		unsigned qpo	:1;
		unsigned apo	:1;
		unsigned vpo	:1;
		unsigned tse	:1;
		unsigned	:2;
		unsigned pbo	:1;
		unsigned ms	:1;
		unsigned tshe	:1;
		unsigned	:1;
		unsigned slost	:1;
		unsigned slock	:1;
	} festat;
	
	struct {
		unsigned	:21;
		#define DEMUX_IN_CI0	0
		#define DEMUX_IN_TSDMA	3
		unsigned insel	:2;
		unsigned	:5;
		unsigned dm	:1;
		unsigned	:3;
	} config3;
	
	struct {
		unsigned	:20;
		signed val	:12;
	} pwm;
	
	struct {
		unsigned	:12;
		unsigned ovfl	:1;
		unsigned sign	:1;
		unsigned delta	:18;
	} pcrstcd;
	
	struct {
		unsigned	:26;
		unsigned tls	:1;
		unsigned ipsc	:1;
		unsigned	:1;
		unsigned afp	:1;
		unsigned ts	:1;
		unsigned its	:1;
	} dsstat;
	
	struct {
		unsigned	:14;
		unsigned de	:1;
		unsigned pesl	:1;
		unsigned kid	:3;
		unsigned pidv	:13;
	} avcchng;
	
	struct {
		unsigned	:12;
		unsigned ved	:1;
		unsigned	:3;
		unsigned acpm	:1;
		unsigned vcpm	:1;
		unsigned	:4;
		unsigned mwe	:1;
		unsigned sa	:1;
		unsigned	:1;
		unsigned atsed	:1;
		unsigned atbd	:1;
		unsigned accd	:1;
		unsigned	:1;
		unsigned vtsed	:1;
		unsigned vtbd	:1;
		unsigned vccd	:1;
	} config2;
	
	struct {
		unsigned qpt	:4;
		unsigned apt	:4;
		unsigned vpt	:4;
		unsigned	:4;
		unsigned ml	:4;
		unsigned	:2;
		unsigned cvp	:10;
	} pbuflvl;
	
	struct {
		unsigned fes	:1;
		unsigned que	:1;
		unsigned aud	:1;
		unsigned vid	:1;
		unsigned	:1;
		unsigned stcl	:1;
		unsigned pcr	:1;
		unsigned stcc	:1;
		unsigned plbe	:1;
		unsigned	:1;
		unsigned accc	:1;
		unsigned vccc	:1;
		unsigned sfll	:1;
		unsigned ds	:1;
		unsigned mqstk	:1;
		unsigned	:17;
	} intmask;
	
	struct {
		unsigned	:14;
		unsigned pri	:2;
		unsigned	:8;
		unsigned lc	:8;
	} plbcfg;
	
	struct {
		unsigned	:22;
		unsigned fp	:1;
		unsigned afle	:1;
		unsigned tsp	:1;
		unsigned pse	:1;
		unsigned pusip	:1;
		unsigned afp	:1;
		unsigned afpdp	:1;
		unsigned dip	:1;
		unsigned spp	:1;
		unsigned raip	:1;
	} avstatus;
	
	struct {
		unsigned	:23;
		#define QUEUE_BUCKET_ADAP	0
		#define QUEUE_BUCKET_TS		1
		unsigned bqdt	:1;
		unsigned	:2;
		unsigned bv	:1;
		unsigned idx	:5;
	} bkt1q;
	
	struct {
		unsigned	:4;
		unsigned val	:1;
		unsigned ovf	:1;
		unsigned eme	:1;
		unsigned qin	:1;
		unsigned adr	:24;
	} qaddrst;
	
	struct {
		unsigned	:25;
		unsigned ece	:1;
		unsigned sse	:1;
		unsigned idx	:5;
	} strcfg;
	
	struct {
	/* Following fields are only valid for PIDs 24-27 */
		unsigned	:3;
		unsigned tei	:1;
		unsigned pusi	:1;
		unsigned tpi	:1;
		unsigned tsc	:2;
		unsigned afc	:2;
		unsigned cc	:4;
	/* Following fields are valid for all PIDs */
		unsigned de	:1;
		unsigned pesl	:1;
		unsigned kid	:3;
		unsigned pidv	:13;
	} pid;
	
	struct {
		unsigned	:3;
		unsigned tshmb	:11;
		unsigned	:15;
		unsigned dteic	:1;
		unsigned dafcc	:1;
		unsigned ddpc	:1;
	} tshm;
	
	struct {
		unsigned stopa	:12;
		unsigned strta	:12;
		unsigned bt	:8;
	} qcfga;
	
	struct {
		unsigned rp	:16;
		unsigned	:1;
		unsigned scpc	:1;
		unsigned fsf	:6;
		unsigned	:2;
		unsigned apus	:1;
		unsigned enbl	:1;
		#define QCFG_DT_TSPKT			0
		#define QCFG_DT_TSH_ADP			1
		#define QCFG_DT_ADP_PRV			2
		#define QCFG_DT_PAYLOAD			3
		#define QCFG_DT_PAYL_BKT		4
		#define QCFG_DT_BUCKET			5
		#define QCFG_DT_TSPKT_BKT		6
		#define QCFG_DT_TBSEC			8
		#define QCFG_DT_TBSEC_FLT		9
		#define QCFG_DT_TBSEC_CRC		10
		#define QCFG_DT_TBSEC_FLT_CRC		11
		#define QCFG_DT_TBSEC_BKT		12
		#define QCFG_DT_TBSEC_FLT_BKT		13
		#define QCFG_DT_TBSEC_CRC_BKT		14
		#define QCFG_DT_TBSEC_FLT_CRC_BKT	15
		unsigned dt	:4;
	} qcfgb;
	
	struct {
		unsigned	:3;
		unsigned sfid	:5;
		unsigned	:14;
		unsigned enbl	:1;
		unsigned nc	:1;
		unsigned	:2;
		unsigned nf	:6;
	} sfctrl;
} stbx25xx_demux_val;

/* Transport DMA registers */
#define TSDMA_CONFIG_ENA	(1 << 0)

#define TSDMA_START_START	(1 << 0)
#define TSDMA_START_LINE	(1 << 1)

#define TSDMA_INT_COMPL		(1 << 0)

#define TSDMA_STAT_FULL		(1 << 0)
#define TSDMA_STAT_NOT_EMPTY	(1 << 1)

#define TSDMA_INTMSK_COMPL	(1 << 0)

/* Interrupt status registers */
#define QUEUE_RPI	(0x80000000 >> 16)
#define QUEUE_BTI	(0x80000000 >> 17)
#define QUEUE_PCSC	(0x80000000 >> 18)
#define QUEUE_CRCE	(0x80000000 >> 19)
#define QUEUE_TSLE	(0x80000000 >> 20)
#define QUEUE_SFUC	(0x80000000 >> 21)
#define QUEUE_FP	(0x80000000 >> 22)
#define QUEUE_AFLE	(0x80000000 >> 23)
#define QUEUE_TSP	(0x80000000 >> 24)
#define QUEUE_PSE	(0x80000000 >> 25)
#define QUEUE_PUSIP	(0x80000000 >> 26)
#define QUEUE_AFP	(0x80000000 >> 27)
#define QUEUE_AFPDP	(0x80000000 >> 28)
#define QUEUE_DIP	(0x80000000 >> 29)
#define QUEUE_SPP	(0x80000000 >> 30)
#define QUEUE_RAIP	(0x80000000 >> 31)

#endif
