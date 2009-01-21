/*
 * This file is part of linux driver the digital TV devices equipped with IBM STBx25xx SoC
 *
 * stbx25xx_demux.h - header file for demux hardware.
 *
 * see stbx25xx.c for copyright information.
 */

#ifndef __STBx25xx_DEMUX_H__
#define __STBx25xx_DEMUX_H__

#include "stbx25xx_demux_val.h"

/* Interrupts */
#define DEMUX_IRQ	1

/* Memory map */
#define DEMUX_QUEUES_BASE	0x03C00000	/* Allocable region begins at 60 MB */
#define DEMUX_QUEUES_SIZE	0x00200000	/* Allocable region ends before 62 MB */

#define DEMUX_QUEUE_BLOCK_SHFT	12
#define DEMUX_QUEUE_BLOCK_SIZE	(1 << 12)	/* 4096 bytes per block */
#define DEMUX_QUEUE_BLOCK_ALIGN	(1 << 12)	/* Aligned on 4096 bytes boundary */
#define DEMUX_QUEUE_BLOCK_MASK	(DEMUX_QUEUE_BLOCK_ALIGN - 1)

#define DEMUX_QUEUE_SEG_MASK	0xFF000000

#define DEMUX_AUDIO_QUEUE	30
#define DEMUX_VIDEO_QUEUE	31

/* DCR registers */
#define DEMUX_ADDR	0x180
#define DEMUX_DATA	0x181
#define DEMUX_INT	0x182

/* General Configuration, Control and Front-End */
#define CONFIG1		0x0000
#define CONTROL1	0x0001
#define FESTAT		0x0002
#define FEIMASK		0x0003
#define CONFIG3		0x0006

/* Clock Recovery */
#define PCRHI		0x0010
#define PCRLO		0x0011
#define LSTCHI		0x0012
#define LSTCLO		0x0013
#define STCHI		0x0014
#define STCLO		0x0015
#define PWM		0x0016
#define PCRSTCT		0x0017
#define PCRSTCD		0x0018
#define STCCOMP		0x0019
#define STCCMPD		0x001a

/* Desclambler Status */
#define DSSTAT		0x0048
#define DSIMASK		0x0049

/* TS Header Mask */
#define TSHM_FIRST_ID	24
#define TSHM_BASE	0x0138
#define TSHM(id)	(TSHM_BASE + (id - 24))

/* Additional PID */
#define VCCHNG		0x01f0
#define ACCHNG		0x01f1
#define PCRPID		0x01ff

/* Back-End Configuration and Control */
#define CONFIG2		0x1000
#define PBUFLVL		0x1002
#define INTMASK		0x1003
#define PLBCFG		0x1004
#define QSTMSK		0x1005
#define QINT		0x1010
#define QINTMSK		0x1011
#define ASTATUS		0x1012
#define AINTMSK		0x1013
#define VSTATUS		0x1014
#define VINTMSK		0x1015

/* Queues Configuration and Control */
#define QBASE		0x1020
#define BKT1Q		0x1021
#define QADDRST		0x1023
#define QSTOPS		0x1024
#define QRESETS		0x1025
#define SFCHNG		0x1026
#define STR0CFG		0x1028
#define STR0		0x1029
#define STR0MSK		0x102a
#define STR1CFG		0x102b
#define STR1		0x102c
#define STR1MSK		0x102d

/* Descrambler */
#define DESCRAMBLER_BASE	0x2500
#define DESCRAMBLER(set, reg)	(DESCRAMBLER_BASE + (8*(set)) + (reg))
#define IVA		0
#define IVB		1
#define ECWA		2
#define ECWB		3
#define OCWA		4
#define OCWB		5
/* For example: DESCRAMBLER(0, IVA) = IVA0 and DESCRAMBLER(7, OCWB) = OCWB7  */

/* PID Filters and Queues */
#define PID_FILTER_BASE		0x0100
#define PIDFLT(pid)		(PID_FILTER_BASE + (pid))

#define QCFG_BASE		0x2200
#define QCFGA(queue)		(QCFG_BASE + (2*(queue)))
#define QCFGB(queue)		(QCFG_BASE + (2*(queue)) + 1)

#define QSTAT_BASE		0x2600
#define QSTATA(queue)		(QSTAT_BASE + (queue))

#define QSTATBCD_BASE		0x2800
#define QSTATB(queue)		(QSTATBCD_BASE + (4*(queue)))
#define QSTATC(queue)		(QSTATBCD_BASE + (4*(queue)) + 1)
#define QSTATD(queue)		(QSTATBCD_BASE + (4*(queue)) + 2)

/* Section Filters */
#define SEC_FILTER_BASE		0x2300
#define SECFLT_CTRL(block)	(SEC_FILTER_BASE + (4*(block)))
#define SECFLT_VAL(block)	(SEC_FILTER_BASE + (4*(block)) + 1)
#define SECFLT_MASK(block)	(SEC_FILTER_BASE + (4*(block)) + 2)
#define SECFLT_POS(block)	(SEC_FILTER_BASE + (4*(block)) + 3)

/* Transport DMA - DCR regs */
#define TSDMA_IRQ		23

#define TSDMA_CONFIG		0x02c0
#define TSDMA_START		0x02c1
#define TSDMA_ADDR		0x02c4
#define TSDMA_COUNT		0x02c7
#define TSDMA_INT		0x02ca
#define TSDMA_STAT		0x02cb
#define TSDMA_INTMSK		0x02ce

#endif
