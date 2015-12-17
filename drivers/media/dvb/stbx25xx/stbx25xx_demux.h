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
#define DEMUX_IRQ		1

#define DEMUX_IRQ_FRONTEND	0
#define DEMUX_IRQ_QUEUES	1
#define DEMUX_IRQ_AUDIO		2
#define DEMUX_IRQ_VIDEO		3
#define DEMUX_IRQ_STC		5
#define DEMUX_IRQ_PCR		6
#define DEMUX_IRQ_STC_COMP	7
#define DEMUX_IRQ_PLB_ERR	8
#define DEMUX_IRQ_ACCHNG_DONE	10
#define DEMUX_IRQ_VCCHNG_DONE	11
#define DEMUX_IRQ_SEC_FLT_ERR	12
#define DEMUX_IRQ_DESCRAMBLER	13
#define DEMUX_IRQ_STR_FOUND	14

/* Memory map */
#if defined(CONFIG_DM500)
#define DEMUX_QUEUES_BASE	0xA0800000
#define DEMUX_QUEUES_SIZE	0x00200000
#else
#define DEMUX_QUEUES_BASE	0x03C00000	/* Allocable region begins at 60 MB */
#define DEMUX_QUEUES_SIZE	0x00200000	/* Allocable region ends before 62 MB */
#endif

#define DEMUX_QUEUE_BLOCK_SHFT	12
#define DEMUX_QUEUE_BLOCK_SIZE	(1 << 12)	/* 4096 bytes per block */
#define DEMUX_QUEUE_BLOCK_ALIGN	(1 << 12)	/* Aligned on 4096 bytes boundary */
#define DEMUX_QUEUE_BLOCK_MASK	(DEMUX_QUEUE_BLOCK_ALIGN - 1)

#define DEMUX_QUEUE_SEG_MASK	0xFF000000

#define DEMUX_BUCKET_QUEUE	29
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

#define QSTATBCD_BASE		0x2801
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

/* Clock recovery */
#define PWM_CLAMP               50       /* Sets the maximum rate of change  */
                                         /* of the PWM value                 */
					 
#define STC_LOAD_PWM_CLAMP      100      /* Sets the maximum rate of change  */
                                         /* just after the PWM value is read */
					 
#define PCRS_HIGH_GAIN          10       /* Number of PCRs after the STC is  */
                                         /* loaded which will use            */
                                         /* STC_LOAD_PWM_CLAMP               */
					 
#define RATE_GAIN               4        /* The calculated rate difference   */
                                         /* is divided by RATE_GAIN ^ 2      */
					 
#define DELTA_GAIN              4        /* The calculated delta             */
                                         /* is divided by DELTA_GAIN ^ 2     */
					 
#define LOW_THRESHOLD           32       /* Lower Delta Threshold for the    */
                                         /* s/w clock recovery algorithm     */
					 
#define HIGH_THRESHOLD          256      /* Upper Delta Threshold for the    */
                                         /* s/w clock recovery algorithm     */
					 
#define DECODER_SHIFT		((45000/1000) * 60 * -1)

/* Structures */

struct demux_queue {
	u32 handle;
	u32 pid;
	phys_addr_t phys_addr;
	void *addr;
	void *ptr;
	size_t size; /* 0 = Queue disabled */
#define QUEUE_CONFIG_TYPE_MASK	((1 << 4) - 1)
#define QUEUE_CONFIG_PESL	(1 << 4)
#define QUEUE_CONFIG_DE		(1 << 5)
#define QUEUE_CONFIG_SCPC	(1 << 6)
#define QUEUE_CONFIG_APUS	(1 << 7)
#define QUEUE_CONFIG_SYSTEM	(1 << 8)
/* ^^^ Video and Audio queues use no memory and have interrupts disabled ^^^ */
#define QUEUE_CONFIG_SECFLT	(1 << 9)
#define QUEUE_CONFIG_SWDEMUX	(1 << 10)
//#define QUEUE_CONFIG_STOPPING	(1 << 11)
//#define QUEUE_CONFIG_SUSPENDED	(1 << 14)
//#define QUEUE_CONFIG_ACTIVE	(1 << 15)
	u16 config;
	u8 key;
#define QUEUE_STATE_FREE	0
#define QUEUE_STATE_STARTING	1
#define QUEUE_STATE_ACTIVE	2
#define QUEUE_STATE_STOPPING	3
#define QUEUE_STATE_SUSPENDED	4
#define QUEUE_STATE_SYSTEM	5
	volatile u8 state;
	u32 data_count;
	spinlock_t lock;
	struct work_struct work;
	struct dvb_demux_feed *feed;
	struct dvb_demux *demux;
	struct list_head filters;
	struct list_head list;
	size_t (*cb)(struct demux_queue *, void *, size_t, void *, size_t);
	struct stbx25xx_demux_data *dmx;
};

struct filter_block {
	int index;
	u32 value;
	u32 mask;
	u32 positive;
	u32 sfid;
	struct list_head list;
};

#endif
