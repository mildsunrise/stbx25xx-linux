/*
 * stbx25xx_demux.c - Transport Stream Demultiplexer driver
 * for digital TV devices equipped with IBM STBx25xx SoC
 *
 * Copyright (C) 2009 Tomasz Figa <tomasz.figa@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/semaphore.h>
#include <asm/dcr.h>
#include "stbx25xx_common.h"
#include "stbx25xx_demux.h"

/**
	Queue memory manager
**/

/* Data types */
struct queues_pool_node {
	void *addr;
	phys_addr_t phys_addr;
	size_t size;
	struct queues_pool_node *next;
	struct queues_pool_node *prev;
};

struct queues_pool_head {
	void *mem_start;
	phys_addr_t mem_phys_start;
	size_t mem_size;
	size_t free_mem;
	struct queues_pool_node *free_blocks;
	struct queues_pool_node *last_free_block;
};

/* Static data */
static struct queues_pool_head qp_head;
static struct kmem_cache *qp_node_cache;
static struct semaphore qp_sem;

/* Memory management routines */
static void queues_pool_free(void *addr, size_t size)
{
	struct queues_pool_node *node;
	struct queues_pool_node *new_node;
	struct queues_pool_node *obsolete_node;
	
	BUG_ON(!addr);
	BUG_ON(size & DEMUX_QUEUE_BLOCK_MASK);
	
	if(down_killable(&qp_sem)) {
		err("Received a fatal signal, exiting...\n");
		return;
	}
	
	node = qp_head.free_blocks;
	
	if(!node) {
		new_node = kmem_cache_alloc(qp_node_cache, GFP_KERNEL);
		new_node->addr = addr;
		new_node->phys_addr = (phys_addr_t)(addr - qp_head.mem_start) + qp_head.mem_phys_start;
		new_node->size = size;
		new_node->next = NULL;
		new_node->prev = NULL;
		qp_head.free_blocks = new_node;
		qp_head.last_free_block = new_node;
		qp_head.free_mem += size;
		
		goto exit;
	}
	
	while(node) {
		if(node->addr == addr) { /* Honestly, this should never happen */
			err("Tried to free already freed queue block");
			goto exit;
		}
		
		if(node->addr + node->size == addr)
			break;
		
		if(node->addr > addr)
			break;
		
		node = node->next;
	}
	
	if(!node) { /* new_node is placed after the last node */
		new_node = kmem_cache_alloc(qp_node_cache, GFP_KERNEL);
		new_node->addr = addr;
		new_node->phys_addr = (phys_addr_t)(addr - qp_head.mem_start) + qp_head.mem_phys_start;
		new_node->size = size;
		new_node->prev = qp_head.last_free_block;
		new_node->next = NULL;
		new_node->prev->next = new_node;
		qp_head.last_free_block = new_node;
	} else {
		if(node->addr + node->size == addr) { /* New node is merged with the found node on the right */
			if(node->next && (addr + size == node->next->addr)) { /* ...and also on the left */
				obsolete_node = node->next;
				node->size += size + obsolete_node->size;
				
				if(obsolete_node->next)
					obsolete_node->next->prev = node;
				else
					qp_head.last_free_block = node;
				
				node->next = obsolete_node->next;
				
				kmem_cache_free(qp_node_cache, obsolete_node);
			} else { /* ...only */
				node->size += size;
			}
		} else if (addr + size == node->addr) { /* New node is merged with the found node on the left */
			node->addr = addr;
			node->phys_addr = (phys_addr_t)(addr - qp_head.mem_start) + qp_head.mem_phys_start;
			node->size += size;
		} else { /* New node is placed before the found node */
			new_node = kmem_cache_alloc(qp_node_cache, GFP_KERNEL);
			new_node->addr = addr;
			new_node->phys_addr = (phys_addr_t)(addr - qp_head.mem_start) + qp_head.mem_phys_start;
			new_node->size = size;
			new_node->prev = node->prev;
			new_node->next = node;
			node->prev = new_node;
		
			if(new_node->prev)
				new_node->prev->next = new_node;
			else
				qp_head.free_blocks = new_node;
		}
	}
	
	qp_head.free_mem += size;
	
exit:
	up(&qp_sem);
}

static void *queues_pool_alloc(size_t size, phys_addr_t *phys_addr_ret)
{
	struct queues_pool_node *node;
	void *addr = NULL;
	
	BUG_ON(size & DEMUX_QUEUE_BLOCK_MASK);
	
	if(down_killable(&qp_sem)) {
		err("Received a fatal signal, exiting...\n");
		return NULL;
	}
	
	node = qp_head.free_blocks;
	
	while(node) {
		if(node->size >= size)
			break;
		
		node = node->next;
	}
	
	if(!node) /* Oops... not enough free memory */
		goto exit;
	
	if(node->size > size) {
		addr = node->addr + (node->size - size);
		
		if(phys_addr_ret)
			*phys_addr_ret = node->phys_addr + (node->size - size);
		
		node->size -= size;
	} else {
		addr = node->addr;
		
		if(phys_addr_ret)
			*phys_addr_ret = node->phys_addr;
		
		if(node->prev)
			node->prev->next = node->next;
		else
			qp_head.free_blocks = node->next;
		
		if(node->next)
			node->next->prev = node->prev;
		else
			qp_head.last_free_block = node->prev;
		
		kmem_cache_free(qp_node_cache, node);
	}
	
	qp_head.free_mem -= size;
	
exit:
	up(&qp_sem);
	return addr;
}

static int queues_pool_init(phys_addr_t phys, void *virt, size_t size)
{
	BUG_ON(!virt);
	BUG_ON(!size);
	
	qp_node_cache = kmem_cache_create("queues_pool_node_cache", sizeof(struct queues_pool_node), 0, SLAB_HWCACHE_ALIGN, NULL);
	if(!qp_node_cache) {
		err("Failed to create the nodes SLAB cache");
		return -ENOMEM;
	}
	
	qp_head.mem_start	= virt;
	qp_head.mem_phys_start	= phys;
	qp_head.mem_size	= size;
	qp_head.free_mem	= size;
	qp_head.free_blocks	= kmem_cache_alloc(qp_node_cache, GFP_KERNEL);
	qp_head.last_free_block	= qp_head.free_blocks;
	
	qp_head.free_blocks->addr	= virt;
	qp_head.free_blocks->phys_addr	= phys;
	qp_head.free_blocks->size	= size;
	qp_head.free_blocks->next	= NULL;
	qp_head.free_blocks->prev	= NULL;
	
	sema_init(&qp_sem, 1);
	
	return 0;
}

static void queues_pool_deinit(void)
{
	int force = 0;
	
	if(down_killable(&qp_sem)) {
		warn("Received a fatal signal, forcing deinitialization...\n");
		force = 1;
	}
	
	if(qp_node_cache)
		kmem_cache_destroy(qp_node_cache);
	
	if(!force)
		up(&qp_sem);
}

/*
-------------------------------------------------------------------------------
 */

/**
	Low-level hardware access routines
**/

static void *queues_base = NULL;

static spinlock_t demux_dcr_lock;

/* set_demux_reg */
static inline void set_demux_reg(u16 reg, stbx25xx_demux_val val)
{
	unsigned long flags;
	
	spin_lock_irqsave(&demux_dcr_lock, flags);
	
	mtdcr(DEMUX_ADDR, reg);
	mtdcr(DEMUX_DATA, val.raw);
	
	spin_unlock_irqrestore(&demux_dcr_lock, flags);
}

/* set_demux_reg_raw */
static inline void set_demux_reg_raw(u16 reg, u32 val)
{
	unsigned long flags;
	
	spin_lock_irqsave(&demux_dcr_lock, flags);
	
	mtdcr(DEMUX_ADDR, reg);
	mtdcr(DEMUX_DATA, val);
	
	spin_unlock_irqrestore(&demux_dcr_lock, flags);
}

/* get_demux_reg */
static inline stbx25xx_demux_val get_demux_reg(u16 reg)
{
	stbx25xx_demux_val val;
	unsigned long flags;

	spin_lock_irqsave(&demux_dcr_lock, flags);
	
	mtdcr(DEMUX_ADDR, reg);
	val.raw = mfdcr(DEMUX_DATA);
	
	spin_unlock_irqrestore(&demux_dcr_lock, flags);
	
	return val;
}

/* get_demux_reg_raw */
static inline u32 get_demux_reg_raw(u16 reg)
{
	u32 val;
	unsigned long flags;

	spin_lock_irqsave(&demux_dcr_lock, flags);
	
	mtdcr(DEMUX_ADDR, reg);
	val = mfdcr(DEMUX_DATA);
	
	spin_unlock_irqrestore(&demux_dcr_lock, flags);
	
	return val;
}

/*
-------------------------------------------------------------------------------
 */

/**
	Queue management
**/

static void demux_stop_queue(int queue)
{
	u32 reg;
	
	reg = get_demux_reg_raw(QSTOPS);
	reg |= (1 << 31) >> queue;
	set_demux_reg_raw(QSTOPS, reg);
}

static void demux_reset_queue(int queue)
{
	u32 reg;
	
	reg = get_demux_reg_raw(QRESETS);
	reg |= (1 << 31) >> queue;
	set_demux_reg_raw(QRESETS, reg);
}

/*
-------------------------------------------------------------------------------
 */

/**
	PID filter management
**/

/* TODO: To be implemented... */

/*
-------------------------------------------------------------------------------
 */

/**
	PCR clock management
**/

static void demux_clk_mgmt_start(void)
{
	
}

static void demux_clk_mgmt_stop(void)
{
	
}

/*
-------------------------------------------------------------------------------
 */

/**
	Hardware configuration routines
**/

static stbx25xx_demux_val config1_def = {
	.config1 = {
		.syncl	= 2,
	},
};
static stbx25xx_demux_val config1_cur;

static stbx25xx_demux_val config2_def = {
	.config2 = {
		.mwe	= 1,
		.sa	= 1,
	},
};
static stbx25xx_demux_val config2_cur;

static stbx25xx_demux_val config3_def = {
	.config3 = {
		.insel	= DEMUX_IN_CI0,
	},
};
static stbx25xx_demux_val config3_cur;

static stbx25xx_demux_val control1_def = {
	.control1 = {
		.se	= 1,
	},
};
static stbx25xx_demux_val control1_cur;

static stbx25xx_demux_val pbuflvl_def = {
	.pbuflvl = {
		.qpt	= 8,
		.apt	= 4,
		.vpt	= 8,
	},
};
static stbx25xx_demux_val pbuflvl_cur;

/* reset_queue_config */
static void reset_queue_config(int queue)
{
	set_demux_reg_raw(QCFGA(queue), 0);
	set_demux_reg_raw(QCFGB(queue), 0);
}

/* demux_load_config */
static void demux_load_config(void)
{
	stbx25xx_demux_val config1, config2, config3, control1, pbuflvl;

/* Read the registers */
	config1 = get_demux_reg(CONFIG1);
	config2 = get_demux_reg(CONFIG2);
	config3 = get_demux_reg(CONFIG3);
	control1 = get_demux_reg(CONTROL1);
	pbuflvl = get_demux_reg(PBUFLVL);

/* CONFIG1 */
	config1.config1.vpu	= config1_cur.config1.vpu;
	config1.config1.apu	= config1_cur.config1.apu;
	config1.config1.tstoe	= config1_cur.config1.tstoe;
	config1.config1.tsclkp	= config1_cur.config1.tsclkp;
	config1.config1.tsdp	= config1_cur.config1.tsdp;
	config1.config1.tsvp	= config1_cur.config1.tsvp;
	config1.config1.syncd	= config1_cur.config1.syncd;
	config1.config1.bb	= config1_cur.config1.bb;
	config1.config1.syncl	= config1_cur.config1.syncl;

/* CONFIG2 */
	config2.config2.ved	= config2_cur.config2.ved;
	config2.config2.acpm	= config2_cur.config2.acpm;
	config2.config2.vcpm	= config2_cur.config2.vcpm;
	config2.config2.mwe	= config2_cur.config2.mwe;
	config2.config2.sa	= config2_cur.config2.sa;
	config2.config2.atsed	= config2_cur.config2.atsed;
	config2.config2.atbd	= config2_cur.config2.atbd;
	config2.config2.accd	= config2_cur.config2.accd;
	config2.config2.vtsed	= config2_cur.config2.vtsed;
	config2.config2.vtbd	= config2_cur.config2.vtbd;
	config2.config2.vccd	= config2_cur.config2.vccd;

/* CONFIG3 */
	config3.config3.insel	= config3_cur.config3.insel;
	
/* CONTROL1 */
	control1.control1.sbe	= control1_cur.control1.sbe;
	control1.control1.pbe	= control1_cur.control1.pbe;
	control1.control1.sdop	= control1_cur.control1.sdop;
	
/* PBUFLVL */
	pbuflvl.pbuflvl.qpt	= pbuflvl_cur.pbuflvl.qpt;
	pbuflvl.pbuflvl.apt	= pbuflvl_cur.pbuflvl.apt;
	pbuflvl.pbuflvl.vpt	= pbuflvl_cur.pbuflvl.vpt;
	
/* Write the registers with Sync Enable bit cleared */
	control1.control1.se	= 0;
	set_demux_reg(CONFIG1, config1);
	set_demux_reg(CONFIG2, config2);
	set_demux_reg(CONFIG3, config3);
	set_demux_reg(CONTROL1, control1);
	set_demux_reg(PBUFLVL, pbuflvl);
	
/* Set Sync Enable bit if wanted to */
	control1.control1.se	= control1_cur.control1.se;
	set_demux_reg(CONTROL1, control1);
}

/* demux_reset */
static int demux_reset(void)
{
	stbx25xx_demux_val reg;
	int i;
	
	config1_cur	= config1_def;
	config2_cur	= config2_def;
	config3_cur	= config3_def;
	control1_cur	= control1_def;
	pbuflvl_cur	= pbuflvl_def;
	
	printk(KERN_INFO STBx25xx_LOG_PREFIX ": resetting demux...");

/* Soft-reset the hardware */
	reg.raw 		= 0;
	reg.control1.swr 	= 1;
	set_demux_reg(CONTROL1, reg);
	
	i = 0;
	while(get_demux_reg(CONTROL1).control1.swr) {
		if(i++ == 200) {
			printk(" failed (timeout)\n");
			return -1;
		}
		mdelay(1);
	}
	
	printk(" done (took %d ms)\n", i);
	
/* Set the registers to a known state */
	set_demux_reg_raw(CONFIG1, 0);
	set_demux_reg_raw(CONTROL1, 0);
	set_demux_reg_raw(FEIMASK, 0);
	set_demux_reg_raw(CONFIG3, 0);
	set_demux_reg_raw(PCRSTCT, 0);
	set_demux_reg_raw(STCCOMP, 0);
	set_demux_reg_raw(DSIMASK, 0);
	set_demux_reg_raw(CONFIG2, 0);
	set_demux_reg_raw(PBUFLVL, 0);
	set_demux_reg_raw(INTMASK, 0);
	
	reg.raw 	= 0;
	reg.plbcfg.pri	= 0x03;
	reg.plbcfg.lc	= 0x10;
	set_demux_reg(PLBCFG, reg);
	
	set_demux_reg_raw(QSTMSK, 0);
	
/* Reset queues configuration */	
	for(i=0; i<STBx25xx_QUEUE_COUNT; i++) {
		reset_queue_config(i);
	}
	
/* Load selected (in this case default) configuration */
	demux_load_config();
	
	return 0;
}

static void demux_set_queues_base_ptr(phys_addr_t addr)
{
	set_demux_reg_raw(QBASE, addr & DEMUX_QUEUE_SEG_MASK);
}

/*
-------------------------------------------------------------------------------
 */

/**
	Interrupts management
**/

static u32 demux_irq_mask;
static u32 demux_queues_irq_mask;
static u32 demux_festat_irq_mask;
static u32 demux_audio_irq_mask;
static u32 demux_video_irq_mask;
static u32 demux_descr_irq_mask;

typedef void (*demux_irq_handler_t)(struct stbx25xx_dvb_dev *dvb);
static demux_irq_handler_t irq_handler_table[15];

static irqreturn_t demux_irq_handler(int irq, void *dev_id)
{
	int i;
	struct stbx25xx_dvb_dev *dvb = dev_id;
	u32 first_stat = mfdcr(DEMUX_INT) & demux_irq_mask;	/* AND with mask to be safe... */
	mtdcr(DEMUX_INT, first_stat);				/* Clear instantly to minimize interrupt loss */
	
	for(i = 0; i < 15 && first_stat; i++, first_stat <<= 1) {
		if(first_stat & (1 << 31) && irq_handler_table[i])
			irq_handler_table[i](dvb);
	}
	
	return IRQ_HANDLED;
}

static int demux_irq_init(struct stbx25xx_dvb_dev *dvb)
{
	memset(irq_handler_table, 0, sizeof(demux_irq_handler_t) * ARRAY_SIZE(irq_handler_table));
	
	demux_irq_mask		= 0;
	demux_queues_irq_mask	= 0;
	demux_festat_irq_mask	= 0;
	demux_audio_irq_mask	= 0;
	demux_video_irq_mask	= 0;
	demux_descr_irq_mask	= 0;
	
	set_demux_reg_raw(INTMASK, demux_irq_mask);
	set_demux_reg_raw(QINTMSK, demux_queues_irq_mask);
	set_demux_reg_raw(FEIMASK, demux_festat_irq_mask);
	set_demux_reg_raw(AINTMSK, demux_audio_irq_mask);
	set_demux_reg_raw(VINTMSK, demux_video_irq_mask);
	set_demux_reg_raw(DSIMASK, demux_descr_irq_mask);
	
	return request_irq(DEMUX_IRQ, demux_irq_handler, IRQF_SHARED, DRIVER_NAME, dvb);
}

/*
-------------------------------------------------------------------------------
 */

/**
	DVB API interface
**/

/* stbx25xx_demux_check_crc32 */
u32 stbx25xx_demux_check_crc32(struct dvb_demux_feed *feed,
			    const u8 *buf, size_t len)
{
	return 0;
}

/* stbx25xx_demux_memcopy */
void stbx25xx_demux_memcopy(struct dvb_demux_feed *feed, u8 *dst,
			 const u8 *src, size_t len)
{
	
}

/* stbx25xx_demux_connect_frontend */
int stbx25xx_demux_connect_frontend(struct dmx_demux* demux,
				 struct dmx_frontend* frontend)
{
	return 0;
}

/* stbx25xx_demux_disconnect_frontend */
int stbx25xx_demux_disconnect_frontend(struct dmx_demux* demux)
{
	return 0;
}

/* stbx25xx_demux_get_stc */
int stbx25xx_demux_get_stc(struct dmx_demux* demux, unsigned int num,
			u64 *stc, unsigned int *base)
{
	return 0;
}

/* stbx25xx_demux_start_feed */
int stbx25xx_demux_start_feed(struct dvb_demux_feed *feed)
{
	return 0;
}

/* stbx25xx_demux_stop_feed */
int stbx25xx_demux_stop_feed(struct dvb_demux_feed *feed)
{
	return 0;
}

/* stbx25xx_demux_write_to_decoder */
int stbx25xx_demux_write_to_decoder(struct dvb_demux_feed *feed,
				 const u8 *buf, size_t len)
{
	return 0;
}

/*
-------------------------------------------------------------------------------
 */

/**
	Module-related routines
**/

/* stbx25xx_demux_init */
int stbx25xx_demux_init(struct stbx25xx_dvb_dev *dvb)
{
	int ret;
	
	spin_lock_init(&demux_dcr_lock);
	
	if((ret = demux_reset()) != 0)
		goto error_reset;
	
	if((queues_base = ioremap_nocache(DEMUX_QUEUES_BASE, DEMUX_QUEUES_SIZE)) == NULL) {
		err("Failed to map demux memory");
		ret = -ENOMEM;
		goto error_ioremap;
	}
	
	info("Mapped %d bytes of demux memory at 0x%p", DEMUX_QUEUES_SIZE, queues_base);

	/* Disable all interrupts */
	if((ret = demux_irq_init(dvb)) != 0) {
		err("Failed to request demux irq: error %d", ret);
		goto error_irq;
	}
	
	if((ret = queues_pool_init(DEMUX_QUEUES_BASE, queues_base, DEMUX_QUEUES_SIZE)) != 0)
		goto error_pool;
	
	demux_set_queues_base_ptr(DEMUX_QUEUES_BASE);
	demux_clk_mgmt_start();
	
	return 0;
	
error_pool:
	free_irq(DEMUX_IRQ, dvb);
error_irq:
	iounmap(queues_base);
	queues_base = NULL;
error_ioremap:
error_reset:
	return ret;
}

/* stbx25xx_demux_exit */
void stbx25xx_demux_exit(struct stbx25xx_dvb_dev *dvb)
{
	free_irq(DEMUX_IRQ, dvb);
	
	demux_clk_mgmt_stop();
	demux_set_queues_base_ptr(0);	
	queues_pool_deinit();
	
	if(queues_base)
		iounmap(queues_base);
}

/*
-------------------------------------------------------------------------------
 */
