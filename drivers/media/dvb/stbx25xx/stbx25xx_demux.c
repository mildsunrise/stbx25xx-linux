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

/* Uncommend the following lines to show debugging messages */
/*
#define DEBUG	10
#define DBG_LEVEL 1
*/

/* Comment out the following line to enable hardware section filtering */
#define STBx25xx_SW_SECT_FILT

#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <asm/dcr.h>
#include "stbx25xx.h"
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

/*
 * queues_pool_freemem()
 * Get the amount of free memory. (The largest block could be smaller!)
 */
static int queues_pool_freemem(void)
{
	int ret;
	
	if(down_killable(&qp_sem)) {
		err("Received a fatal signal, exiting...\n");
		return 0;
	}
	
	ret = qp_head.free_mem;
	
	up(&qp_sem);
	
	return ret;
}

/*
 * queues_pool_free()
 * Free allocated block of queue memory
 */
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

/*
 * queues_pool_alloc() 
 * Allocate block of queue memory. Block size must be a multiple of 4 kB.
 */
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

/* queues_pool_init */
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

/* queues_pool_deinit */
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

typedef void (*demux_irq_handler_t)(struct stbx25xx_demux_data *dmx, int irq);
static void demux_set_handler(int demux_irq, demux_irq_handler_t demux_handler);
static void demux_disable_irq(int demux_irq);
static void demux_enable_irq(int demux_irq);

/**
	Low-level hardware access routines
**/

static void *queues_base = NULL;

/* set_demux_reg */
static inline void set_demux_reg(u16 reg, stbx25xx_demux_val val)
{
	unsigned long flags;
	
	local_irq_save(flags);
	
	mtdcr(DEMUX_ADDR, reg);
	mtdcr(DEMUX_DATA, val.raw);
	
	local_irq_restore(flags);
}

/* set_demux_reg_raw */
static inline void set_demux_reg_raw(u16 reg, u32 val)
{
	unsigned long flags;
	
	local_irq_save(flags);
	
	mtdcr(DEMUX_ADDR, reg);
	mtdcr(DEMUX_DATA, val);
	
	local_irq_restore(flags);
}

/* get_demux_reg */
static inline stbx25xx_demux_val get_demux_reg(u16 reg)
{
	stbx25xx_demux_val val;
	unsigned long flags;

	local_irq_save(flags);
	
	mtdcr(DEMUX_ADDR, reg);
	val.raw = mfdcr(DEMUX_DATA);
	
	local_irq_restore(flags);
	
	return val;
}

/* get_demux_reg_raw */
static inline u32 get_demux_reg_raw(u16 reg)
{
	u32 val;
	unsigned long flags;

	local_irq_save(flags);
	
	mtdcr(DEMUX_ADDR, reg);
	val = mfdcr(DEMUX_DATA);
	
	local_irq_restore(flags);
	
	return val;
}

/*
-------------------------------------------------------------------------------
 */

/**
	Hardware management
**/

static stbx25xx_demux_val config1_def = {
	.config1 = {
		.syncl	= 2,
	},
};

static stbx25xx_demux_val config2_def = {
	.config2 = {
		.mwe	= 1,
		.sa	= 1,
	},
};

static stbx25xx_demux_val config3_def = {
	.config3 = {
		.insel	= DEMUX_IN_CI0,
	},
};

static stbx25xx_demux_val control1_def = {
	.control1 = {
		.se	= 0,
	},
};

static stbx25xx_demux_val pbuflvl_def = {
	.pbuflvl = {
		.qpt	= 8,
		.apt	= 4,
		.vpt	= 8,
	},
};

/*
 * demux_change_input()
 * Switch between CI0 and TSDMA inputs. Sync Enable bit state is preserved.
 */
static void demux_change_input(u32 input)
{
	stbx25xx_demux_val reg;
	u32 sync_enabled;
	
	reg = get_demux_reg(CONTROL1);
	sync_enabled = reg.control1.se;
	reg.control1.se = 0;
	set_demux_reg(CONTROL1, reg);
	
	reg = get_demux_reg(CONFIG3);
	reg.config3.insel = input & 3;
	set_demux_reg(CONFIG3, reg);
	
	reg = get_demux_reg(CONTROL1);
	reg.control1.se = sync_enabled & 1;
	set_demux_reg(CONTROL1, reg);	
}

/*
 * demux_enable_sync()
 * Enable data acquisition by the Transport Interface
 */
static void demux_enable_sync(void)
{
	stbx25xx_demux_val reg;
	
	dprintk("%s: Enabling data acquisition\n", __func__);
	
	reg = get_demux_reg(CONTROL1);
	reg.control1.se = 1;
	set_demux_reg(CONTROL1, reg);
}

/*
 * demux_disable_sync()
 * Disable data acquisition by the Transport Interface
 */
static void demux_disable_sync(void)
{
	stbx25xx_demux_val reg;
	
	dprintk("%s: Disabling data acquisition\n", __func__);
	
	reg = get_demux_reg(CONTROL1);
	reg.control1.se = 0;
	set_demux_reg(CONTROL1, reg);
}

/*
-------------------------------------------------------------------------------
 */

/**
	Hardware part of queues code
**/

static struct list_head queues_list;
static struct demux_queue demux_queues[STBx25xx_QUEUE_COUNT];
static volatile u32 demux_queues_irq_mask = 0;
static volatile u32 demux_qstat_irq_mask = 0;

#if 0
/*
 * queue_to_virt()
 * Translate queue virtual address to physical one
 */
static void *queue_to_virt(struct demux_queue *queue, phys_addr_t addr)
{
	void *ret;
	
	addr += queues_base
	ret = queue->addr;
	ret += (addr - queue->phys_addr);
	
	return ret;
}
#endif

/*
 * queue_to_phys()
 * Translate queue physical address to virtual one
 */
static phys_addr_t queue_to_phys(struct demux_queue *queue, void *addr)
{
	phys_addr_t ret;
	
	ret = queue->phys_addr;
	ret += (addr - queue->addr);
	ret &= 0xFFFFFF;
	
	return ret;
}

/*
 * demux_unmask_qstat_irq()
 * Enable causing interrupts by given status bits
 */
static void demux_unmask_qstat_irq(u32 mask)
{
	unsigned long flags;
	
	local_irq_save(flags);
	
	mask &= 0xFFFF;
	demux_qstat_irq_mask |= mask;
	set_demux_reg_raw(QSTMSK, demux_qstat_irq_mask);
	
	local_irq_restore(flags);
}

/*
 * demux_mask_qstat_irq()
 * Disable causing interrupts by given status bits
 */
static void demux_mask_qstat_irq(u32 mask)
{
	unsigned long flags;
	
	local_irq_save(flags);
	
	mask &= 0xFFFF;
	demux_qstat_irq_mask &= ~mask;
	set_demux_reg_raw(QSTMSK, demux_qstat_irq_mask);
	
	local_irq_restore(flags);
}

/*
 * demux_enable_queue_irq()
 * Enable queue interrupt
 */
static void demux_enable_queue_irq(int queue)
{
	unsigned long flags;

	BUG_ON(queue > 31);
	
	local_irq_save(flags);
	
	demux_queues_irq_mask |= 1 << (31 - queue);
	set_demux_reg_raw(QINTMSK, demux_queues_irq_mask);
	
	local_irq_restore(flags);
}

/*
 * demux_disable_queue_irq()
 * Disable queue interrupt
 */
static void demux_disable_queue_irq(int queue)
{
	unsigned long flags;
	
	BUG_ON(queue > 31);
	
	local_irq_save(flags);
	
	demux_queues_irq_mask &= ~(1 << (31 - queue));
	set_demux_reg_raw(QINTMSK, demux_queues_irq_mask);
	
	local_irq_restore(flags);
}

/* demux_scpc_enable */
static void demux_scpc_enable(int queue)
{
	stbx25xx_demux_val reg;
	
	reg = get_demux_reg(QCFGB(queue));
	reg.qcfgb.scpc = 1;
	set_demux_reg(QCFGB(queue), reg);
}

/* demux_scpc_disable */
static void demux_scpc_disable(int queue)
{
	stbx25xx_demux_val reg;
	
	reg = get_demux_reg(QCFGB(queue));
	reg.qcfgb.scpc = 0;
	set_demux_reg(QCFGB(queue), reg);
}

/*
 * demux_set_queues_base_ptr()
 * Set the physical base address of the 16 MB queues region
 */
static void demux_set_queues_base_ptr(phys_addr_t addr)
{
	set_demux_reg_raw(QBASE, addr & DEMUX_QUEUE_SEG_MASK);
}

/*
 * reset_queue_config()
 * Reset configuration of chosen queue (CONFIGA and CONFIGB)
 */
static void reset_queue_config(int queue)
{
	set_demux_reg_raw(QCFGA(queue), 0);
	set_demux_reg_raw(QCFGB(queue), 0);
	set_demux_reg_raw(PIDFLT(queue), 0x1fff);
}

/*
 * demux_stop_queue()
 * Stop the chosen queue
 */
static void demux_stop_queue(int queue)
{
	u32 reg;
	
	reg = 1 << (31 - queue);
	set_demux_reg_raw(QSTOPS, reg);
}

/*
 * demux_reset_queue()
 * Reset the chosen queue
 */
static void demux_reset_queue(int queue)
{
	u32 reg;
	
	reg = 1 << (31 - queue);
	set_demux_reg_raw(QRESETS, reg);
}

/* demux_get_write_ptr */
static phys_addr_t demux_get_write_ptr(struct demux_queue *queue)
{
	return (get_demux_reg_raw(QSTATD(queue->handle)) & 0xFFFFFF);
}

/* demux_set_read_ptr */
static void demux_set_read_ptr(struct demux_queue *queue, u32 rp)
{
	stbx25xx_demux_val reg;
	
	rp -= queue->phys_addr;
	if(!rp)
		rp += queue->size;
	rp >>= 8;
	
	reg = get_demux_reg(QCFGB(queue->handle));
	reg.qcfgb.rp = rp;
	set_demux_reg(QCFGB(queue->handle), reg);
}

/*
 * demux_config_queue()
 * Configure the queue and related PID filter according to stored settings
 */
static void demux_config_queue(struct demux_queue *queue)
{
	stbx25xx_demux_val reg;
	struct filter_block *block;
	int num = queue->handle;
	
	if(queue->state == QUEUE_STATE_FREE) {
		reg.raw		= 0;
		reg.qcfga.strta	= (queue->phys_addr & 0xFFFFFF) >> DEMUX_QUEUE_BLOCK_SHFT;
		reg.qcfga.stopa = ((queue->phys_addr + queue->size) & 0xFFFFFF) >> DEMUX_QUEUE_BLOCK_SHFT;
		set_demux_reg(QCFGA(num), reg);
		dprintk("%s: Set QCFGA(%d) to 0x%08x\n", __func__, num, reg.raw);
		
		if(num > 23 && num < 28)
			set_demux_reg_raw(TSHM(num), 0);
		
//		set_demux_reg_raw(QSTATB(queue), demux_queues[queue].phys_addr & 0xFFFFFF);
		
		reg.raw		= 0;
		reg.qcfgb.rp	= (queue->size) >> 8;
		reg.qcfgb.apus	= (queue->config & QUEUE_CONFIG_APUS) != 0;
		reg.qcfgb.dt	= queue->config & QUEUE_CONFIG_TYPE_MASK;
		reg.qcfgb.enbl	= (queue->config & QUEUE_CONFIG_SYSTEM) == 0;
		if(!(queue->config & QUEUE_CONFIG_SYSTEM))
			reg.qcfgb.scpc	= 1;
		
		if(queue->config & QUEUE_CONFIG_SECFLT) {
			block = list_first_entry(&queue->filters, struct filter_block, list);
			reg.qcfgb.fsf = block->index;
		}
		
		set_demux_reg(QCFGB(num), reg);
		dprintk("%s: Set QCFGB(%d) to 0x%08x\n", __func__, num, reg.raw);
		
		set_demux_reg_raw(QSTATA(num), 0xFFFF);
		
		if(queue->config & QUEUE_CONFIG_SECFLT) {
			set_demux_reg_raw(SFCHNG, 1 << (31 - queue->handle));
			while(get_demux_reg_raw(SFCHNG) & (1 << (31 - queue->handle)))
				udelay(10);
		}
		
		if(!(queue->config & QUEUE_CONFIG_SYSTEM)) {
			queue->state = QUEUE_STATE_STARTING;
			demux_enable_queue_irq(num);
		} else {
			queue->state = QUEUE_STATE_SYSTEM;
		}
		
		reg.raw		= 0;
		reg.pid.de	= (queue->config & QUEUE_CONFIG_DE) != 0;
		reg.pid.pesl	= (queue->config & QUEUE_CONFIG_PESL) != 0;
		reg.pid.kid	= queue->key & 7;
		reg.pid.pidv	= queue->pid & 0x1fff;
		set_demux_reg(PIDFLT(num), reg);
		dprintk("%s: Set PIDFLT(%d) to 0x%08x\n", __func__, num, reg.raw);
	}
}

static void demux_config_bucket_queue(struct demux_queue *queue, u8 data_type)
{
	stbx25xx_demux_val reg;
	
	reg.raw = 0;
	reg.bkt1q.bqdt	= data_type;
	reg.bkt1q.bv	= 1;
	reg.bkt1q.idx	= queue->handle;
	
	set_demux_reg(BKT1Q, reg);
}

/*
 * demux_audio_channel_change()
 * Change audio channel (PID)
 * TODO: Move all feeds with the old PID to another queue
 */
static int demux_audio_channel_change(u16 pid)
{
	stbx25xx_demux_val reg;
	unsigned long flags;
	struct demux_queue *queue = &demux_queues[DEMUX_AUDIO_QUEUE];
	int ret = 0;
	
	dprintk("%s: Audio channel PID = %d\n", __func__, pid);
	
	if(down_interruptible(&queue->dmx->acc_wait)) {
		ret = -ERESTARTSYS;
		goto exit;
	}

	spin_lock_irqsave(&queue->lock, flags);
	
	queue->pid = pid;
	
	reg = get_demux_reg(PIDFLT(DEMUX_AUDIO_QUEUE));
	reg.avcchng.pidv = pid;
	set_demux_reg(ACCHNG, reg);

	spin_unlock_irqrestore(&queue->lock, flags);

exit:	
	return ret;
}

static void demux_audio_channel_changed(struct stbx25xx_demux_data *dmx, int irq)
{	
	up(&dmx->acc_wait);
}

/*
 * demux_video_channel_change()
 * Change video channel (PID)
 */
static int demux_video_channel_change(u16 pid)
{
	stbx25xx_demux_val reg;
	struct demux_queue *queue = &demux_queues[DEMUX_VIDEO_QUEUE];
	unsigned long flags;
	int ret = 0;
	
	dprintk("%s: Video channel PID = %d\n", __func__, pid);
		
	if(down_interruptible(&queue->dmx->vcc_wait)) {
		ret = -ERESTARTSYS;
		goto exit;
	}
	
	spin_lock_irqsave(&queue->lock, flags);
	
	queue->pid = pid;
		
	reg = get_demux_reg(PIDFLT(DEMUX_VIDEO_QUEUE));
	reg.avcchng.pidv = pid;
	set_demux_reg(VCCHNG, reg);
	
	spin_unlock_irqrestore(&queue->lock, flags);
	
exit:
	return ret;
}

static void demux_video_channel_changed(struct stbx25xx_demux_data *dmx, int irq)
{	
	up(&dmx->vcc_wait);
}

/* demux_process_queue_irq */
static void demux_process_queue_irq(int num, u32 stat)
{	
	struct demux_queue *queue = &demux_queues[num];
	
	if(stat & QUEUE_RPI)
		warn("%s: Queue %d Read Pointer Interrupt (stopped to avoid overflow)\n", __func__, num);
	/* TODO: Handle the RPI */
	
	if(stat & QUEUE_FP)
		queue->state = QUEUE_STATE_ACTIVE;

	if((stat & QUEUE_PCSC) || (stat & QUEUE_BTI))
		if(queue->state == QUEUE_STATE_ACTIVE) {
			dprintk("%s: Queuing queue %d work\n", __func__, num);
			queue_work(queue->dmx->workqueue, &queue->work);
		}
}

/*
-------------------------------------------------------------------------------
 */

/**
	Software part of queues code
*/

static struct mutex queues_mutex;

//#define DEBUG_LISTS

static struct demux_queue *demux_alloc_queue(void)
{
	struct demux_queue *entry;
	
	if(list_empty(&queues_list))
		return NULL;
	
	entry = list_first_entry(&queues_list, struct demux_queue, list);	
	list_del(&entry[0].list);
	
	return entry;
}

static void demux_free_queue(struct demux_queue *queue)
{
#ifdef DEBUG_LISTS
	struct demux_queue *pos;
	
	list_for_each_entry(pos, &queues_list, list)
		BUG_ON(pos == queue);
#endif

	queue->pid = 0x1fff;
	queue->size = 0;
	queue->config = 0;

	list_add_tail(&queue->list, &queues_list);
}

/* struct sec_hdr used to determine section length */
struct sec_hdr {
	u8 sect_id;
	u8 size1;
	u8 size2;
	u8 zero;
} __attribute__((packed));

/*
 * demux_section_callback()
 * CALLBACK used for section data
 * (streaming-ready)
 */ 
static size_t demux_section_callback(struct demux_queue *queue, void *buf, size_t size, void *buf2, size_t size2)
{
	struct sec_hdr hdr;
	size_t psize;
	
	if(!size2)
		buf2 = NULL;
	
	if(size + size2 < 7)
		return 0;
	
	/* Skip the match word for now */
	if(size > 4) {
		buf += 4;
		size -= 4;
	} else {
		size2 -= 4 - size;
		buf2 += 4 - size;
		buf = buf2;
		size = size2;
		buf2 = NULL;
		size2 = 0;
	}
	/* TODO: Implement multiple filters support and match word handling */
	
	if(size >= 3) {
		memcpy(&hdr, buf, 3);
	} else {
		if(size)
			memcpy(&hdr, buf, size);
		memcpy(&hdr + size, buf2, 3 - size);
	}
	
	psize = ((hdr.size1 & 0x0f) << 8) | hdr.size2;
	psize += 3;
	
	if(size + size2 < psize)
		return 0;
	
	if(size >= psize) {
		queue->feed->cb.sec(buf, psize, NULL, 0, &queue->feed->filter->filter, DMX_OK);
	} else {
		queue->feed->cb.sec(buf, size, buf2, psize - size, 
				     &queue->feed->filter->filter, DMX_OK);
	}
	
	if(psize % 4) {
		psize -= psize % 4;
		psize += 4;
	}
	
	return psize + 4;
}

/*
 * demux_ts_pes_callback()
 * CALLBACK used for TS/PES data
 * (Boundary Threshold Interrupt ready)
 */
static size_t demux_ts_pes_callback(struct demux_queue *queue, void *buf, size_t size, void *buf2, size_t size2)
{
	if(!size2)
		buf2 = NULL;
	
	queue->feed->cb.ts(buf, size, buf2, size2, &queue->feed->feed.ts, DMX_OK);
	
	return size + size2;
}

/* 
 * demux_swdemux_callback()
 * CALLBACK used for software demuxing
 * (Boundary Threshold Interrupt ready)
 */
static size_t demux_swdemux_callback(struct demux_queue *queue, void *buf, size_t size, void *buf2, size_t size2)
{
	u8 packet[188];
	
	if(size + size2 < 188)
		return 0;

	if(size >= 188) {
		dvb_dmx_swfilter_packets(queue->demux, buf, 1);
	} else {
		memcpy(packet, buf, size);
		memcpy(packet + size, buf2, 188 - size);
		dvb_dmx_swfilter_packets(queue->demux, packet, 1);
	}
	
	return 188;
}

/*
 * demux_get_data()
 * Function unloading data from a single queue (so up to 32 tasks)
 * NOTE: Running as a work in a workqueue shared with all 32 queues
 */
static void demux_get_data(struct work_struct *work)
{
	struct demux_queue * queue = container_of(work, struct demux_queue, work);
	int error = 0;
	phys_addr_t write_ptr, read_ptr;
	size_t data_avl, data_avl2, data_read = 0;
	unsigned long flags;

	dprintk("%s: Processing queue %d\n", __func__, queue->handle);

	read_ptr = queue_to_phys(queue, queue->ptr);
	write_ptr = demux_get_write_ptr(queue);

	if(read_ptr > ((queue->phys_addr + queue->size) & 0xFFFFFF))
		error = 1;
	else if(read_ptr < (queue->phys_addr & 0xFFFFFF))
		error = 1;
	else if(write_ptr > ((queue->phys_addr + queue->size) & 0xFFFFFF))
		error = 1;
	else if(write_ptr < (queue->phys_addr & 0xFFFFFF))
		error = 1;

	if(error) {
		err("%s: Queue %d fatal error: read_ptr = 0x%08x, write_ptr = 0x%08x",
				__func__, queue->handle, read_ptr, write_ptr);
		return;
	}

	if(read_ptr <= write_ptr) {
		data_avl = write_ptr - read_ptr;
		data_avl2 = 0;
	} else {
		data_avl2 = write_ptr - (queue->phys_addr & 0xFFFFFF);
		data_avl = ((queue->phys_addr + queue->size) & 0xFFFFFF) - read_ptr;
	}

	/* Check if there is data ready in the queue */
	if(data_avl) {
		dprintk("%s: Queue %d: %d + %d bytes available\n", __func__, queue->handle, data_avl, data_avl2);
		/* Data available, so get it */
		if(queue->cb)
			data_read = queue->cb(queue, queue->ptr, data_avl, queue->addr, data_avl2);
		else
			data_read = data_avl + data_avl2;
		
		queue->ptr += data_read;
		queue->data_count += data_read;

		if(queue->ptr >= queue->addr + queue->size)
			queue->ptr -= queue->size;

		read_ptr = queue_to_phys(queue, queue->ptr);			
		demux_set_read_ptr(queue, read_ptr);
		
		if(data_read < (data_avl + data_avl2)) {
			/* Requeue the work */
			spin_lock_irqsave(&queue->lock, flags);
			
			if(queue->state == QUEUE_STATE_ACTIVE)
				queue_work(queue->dmx->workqueue, &queue->work);
			
			spin_unlock_irqrestore(&queue->lock, flags);
		}
	}
}

/*
 * demux_remove_queue()
 * Disable the queue and related filter, flush all data and free the memory
 */
static int demux_remove_queue(int num)
{	
	struct demux_queue *queue = &demux_queues[num];
	unsigned long flags;
	
	BUG_ON(queue->state == QUEUE_STATE_FREE);
	
	dprintk("%s: Removing queue %d\n", __func__, num);
	
	/* Stop the queue */
	demux_stop_queue(num);
	demux_disable_queue_irq(num);
	
	if(queue->state == QUEUE_STATE_ACTIVE) {
		spin_lock_irqsave(&queue->lock, flags);
		queue->state = QUEUE_STATE_STOPPING;
		spin_unlock_irqrestore(&queue->lock, flags);
	
		dprintk("%s: Cancelling queue %d work...\n", __func__, num);
		cancel_work_sync(&queue->work);
		dprintk("%s: Queue %d work cancelled.\n", __func__, num);
	}
	
	set_demux_reg_raw(PIDFLT(num), 0x1fff);	
	
	if(queue->state != QUEUE_STATE_SYSTEM)
		queues_pool_free(queue->addr, queue->size);
	
	queue->state = QUEUE_STATE_FREE;
	
	dprintk("%s: Queue %d removed (transfered %d bytes of data)\n",
			__func__, num, queue->data_count);
	
	return 0;
}

/*
 * demux_remove_user_queue()
 * Remove filter as requested from DVB API and remove the queue
 * when there is no feed associated with it
 */
static int demux_remove_user_queue(struct dvb_demux_feed *feed)
{
	int queue, ret = 0;
	struct dvb_demux_feed *f;
	
	if(!feed)
		return -EINVAL;
	
	queue = feed->index;
	
	if(queue > 31)
		return -EINVAL;
	
	if(mutex_lock_killable(&queues_mutex))
		return -ERESTARTSYS;
	
	f = demux_queues[queue].feed;
	
	if(f == feed) {
		demux_queues[queue].feed = feed->priv;
	} else {
		while(f) {
			if(f->priv == feed) {
				f->priv = feed->priv;
				break;
			}
			f = f->priv;
		}
	}
	
	if(!demux_queues[queue].feed && queue < 30) {
		ret = demux_remove_queue(queue);
		demux_free_queue(&demux_queues[queue]);
	}
	
	mutex_unlock(&queues_mutex);
	
	return ret;
}

/*
 * demux_install_queue()
 * Utility function to setup a queue
 */
static int demux_install_queue(int qid, u32 pid, u32 blocks, u16 config, u16 key)
{
	int ret = 0;
	struct demux_queue *queue = &demux_queues[qid];
	
	dprintk("%s: Installing queue %d, pid = %d, size = %d blocks, config = 0x%04x, key = %d\n",
		 __func__, qid, pid, blocks, config, key);
		
	if(queue->state != QUEUE_STATE_FREE) {
		dprintk("%s: Queue %d already installed\n", __func__, queue);
		ret = -EBUSY;
		goto exit;
	}
	
	if(config & QUEUE_CONFIG_SYSTEM) {
		blocks = 0;
		queue->addr = NULL;
		queue->phys_addr = 0;
	} else {
		if(!blocks) {
			err("%s: Invalid queue size (%d blocks)", __func__, blocks);
			ret = -EINVAL;
			goto exit;
		}
		
		if(queues_pool_freemem() < blocks * DEMUX_QUEUE_BLOCK_SIZE) {
			err("%s: Not enough memory for queue", __func__);
			ret = -ENOMEM;
			goto exit;
		}
		
		queue->addr = queues_pool_alloc(blocks * DEMUX_QUEUE_BLOCK_SIZE, &queue->phys_addr);
		if(queue->addr == NULL) {
			err("%s: Couldn't allocate memory for queue", __func__);
			ret = -ENOMEM;
			goto exit;
		}
		
		dprintk("%s: Allocated queue %d to 0x%08x mapped at %p\n", __func__, qid, queue->phys_addr, queue->addr);
	}
		
	queue->size		= blocks * DEMUX_QUEUE_BLOCK_SIZE;
	queue->config		= config;
	queue->pid		= pid;
	queue->key		= key;
	queue->ptr		= queue->addr;
	queue->data_count	= 0;
	
	if(config & QUEUE_CONFIG_SWDEMUX)
		queue->cb = demux_swdemux_callback;
	else if(config & QUEUE_CONFIG_SECFLT)
		queue->cb = demux_section_callback;
	else if(!(config & QUEUE_CONFIG_SYSTEM))
		queue->cb = demux_ts_pes_callback;
	else
		queue->cb = NULL;
	
	demux_stop_queue(qid);
	demux_reset_queue(qid);
	demux_config_queue(queue);
	
	dprintk("%s: Queue installed successfully\n", __func__);
	
exit:
	return ret;
}

/*
 * demux_install_system_queue()
 * Install system filter - for video or audio PIDs
 */
static int demux_install_system_queue(int queue, u32 pid, u16 config, u16 key)
{
	int ret = 0;
	
	if(queue < 30 || queue > 31)
		return -EINVAL;
	
	if(pid & ~0x1FFF)
		return -EINVAL;
	
	if(key > 7)
		return -EINVAL;
	
	if(mutex_lock_killable(&queues_mutex)) {
			err("demux_install_system_queue: Received a fatal signal, exiting...");
			return -ERESTARTSYS;
	}
	
	if(!demux_queues[queue].size)
		ret = demux_install_queue(queue, pid, 0, config | QUEUE_CONFIG_SYSTEM, key);
	else
		err("demux_install_system_queue: Tried to configure an already configured system queue nr %d", queue);
	
	mutex_unlock(&queues_mutex);
	
	return ret;
}

static int demux_install_bucket_queue(int qid, u8 data_type, u32 blocks)
{
	int ret;
	struct demux_queue *queue = &demux_queues[qid];
	
	if(qid != DEMUX_BUCKET_QUEUE)
		return -EINVAL;
	
	if(mutex_lock_killable(&queues_mutex))
		return -ERESTARTSYS;
	
	ret = demux_install_queue(queue->handle, 0x1fff, blocks, QCFG_DT_TSPKT | QUEUE_CONFIG_SWDEMUX, 0);
	if(!ret) {
		demux_config_bucket_queue(queue, data_type);
	}
	
	mutex_unlock(&queues_mutex);
	
	return ((ret) ? ret : queue->handle);
}

/*
 * demux_install_user_queue()
 * Install filter as requested by DVB API
 */
static int demux_install_user_queue(struct dvb_demux_feed *feed, u32 blocks, u16 config)
{
	int i, ret;
	struct demux_queue *queue;
#if 0
	unsigned long flags;
#endif
	
	if(feed->pid >= 0x1fff)
		return -EINVAL;
	
	if(mutex_lock_killable(&queues_mutex))
		return -ERESTARTSYS;
	
	for(i=0; i<STBx25xx_QUEUE_COUNT; i++) {
		if(demux_queues[i].pid == feed->pid) {
			queue = &demux_queues[i];
			goto found;
		}
	}
	
	queue = demux_alloc_queue();
	if(!queue) {
		ret = -EBUSY;
		goto exit;
	}
	
found:
	if(queue->pid == 0x1fff) {
		feed->priv = NULL;
		queue->feed = feed;
		ret = demux_install_queue(queue->handle, feed->pid, blocks, config, 0);
		if(ret)
			demux_free_queue(queue);
	} else {
#if 0		
		spin_lock_irqsave(&queue->lock, flags);
		
		feed->priv = queue->feed;
		queue->feed = feed;
		
		spin_unlock_irqrestore(&queue->lock, flags);
		
		ret = 0;
#else
		dprintk("%s: Duplicating PID values unallowed.\n", __func__);
		ret = -EINVAL;
#endif
	}
	
exit:
	mutex_unlock(&queues_mutex);
	
	return ((ret) ? ret : queue->handle);
}

#define DEMUX_WORKQUEUE "stbdemuxd"

/* demux_init_queues */
static int demux_init_queues(struct stbx25xx_demux_data *dmx)
{
	int i, ret;
		
	memset(demux_queues, 0, sizeof(struct demux_queue) * STBx25xx_QUEUE_COUNT);
	INIT_LIST_HEAD(&queues_list);
	mutex_init(&queues_mutex);
	sema_init(&dmx->vcc_wait, 1);
	sema_init(&dmx->acc_wait, 1);
	dmx->workqueue = create_workqueue(DEMUX_WORKQUEUE);
	
	for(i=0; i<STBx25xx_QUEUE_COUNT; i++) {
		spin_lock_init(&demux_queues[i].lock);
		demux_queues[i].handle = i;
		demux_queues[i].demux = &dmx->demux;
		demux_queues[i].pid = 0x1fff;
		demux_queues[i].cb = NULL;
		demux_queues[i].dmx = dmx;
		
		if(i < STBx25xx_MAX_FEED)
			list_add_tail(&demux_queues[i].list, &queues_list);
		
		INIT_LIST_HEAD(&demux_queues[i].filters);
		INIT_WORK(&demux_queues[i].work, demux_get_data);
	}
	
	if((ret = queues_pool_init(DEMUX_QUEUES_BASE, queues_base, DEMUX_QUEUES_SIZE)) != 0)
		return ret;
	
	demux_set_handler(DEMUX_IRQ_ACCHNG_DONE, demux_audio_channel_changed);
	demux_enable_irq(DEMUX_IRQ_ACCHNG_DONE);
	demux_set_handler(DEMUX_IRQ_VCCHNG_DONE, demux_video_channel_changed);
	demux_enable_irq(DEMUX_IRQ_VCCHNG_DONE);
		
	demux_set_queues_base_ptr(DEMUX_QUEUES_BASE);
	demux_unmask_qstat_irq(QUEUE_RPI | QUEUE_PCSC | QUEUE_FP);
	
	demux_install_system_queue(DEMUX_AUDIO_QUEUE, 0x1FFF, 0, 0);
	demux_install_system_queue(DEMUX_VIDEO_QUEUE, 0x1FFF, 0, 0);
	demux_install_bucket_queue(DEMUX_BUCKET_QUEUE, QUEUE_BUCKET_TS, 64);
	
	return 0;
}

/* demux_queues_deinit */
static void demux_queues_deinit(struct stbx25xx_demux_data *dmx)
{
	int i;
	
	demux_mask_qstat_irq(0xFFFFFFFF);
	
	for(i=0; i<STBx25xx_QUEUE_COUNT; i++) {		
		if(demux_queues[i].state != QUEUE_STATE_FREE) {
			demux_remove_queue(i);
			demux_free_queue(&demux_queues[i]);
		}
	}
	
	destroy_workqueue(dmx->workqueue);
	
	demux_set_queues_base_ptr(0);
	queues_pool_deinit();
}

/*
-------------------------------------------------------------------------------
 */

/**
	Section filters
**/

static struct semaphore filter_block_list_sem;
static struct list_head filter_block_list;
static struct filter_block filter_blocks[STBx25xx_FILTER_BLOCK_COUNT];

static struct filter_block *demux_alloc_filter_block(void)
{
	struct filter_block *entry;
	
	if(down_killable(&filter_block_list_sem)) {
			err("%s: Received a fatal signal, exiting...", __func__);
			return NULL;
	}
	
	if(list_empty(&filter_block_list)) {
		entry = NULL;
		goto exit;
	}
	
	entry = list_first_entry(&filter_block_list, struct filter_block, list);	
	list_del(&entry->list);
	INIT_LIST_HEAD(&entry->list);
	
exit:
	up(&filter_block_list_sem);
	
	return entry;
}

static void demux_free_filter_block(struct filter_block *block)
{
#ifdef DEBUG_LISTS
	struct filter_block *pos;
#endif
	if(down_killable(&filter_block_list_sem)) {
			err("%s: Received a fatal signal, exiting...", __func__);
			return;
	}
	
#ifdef DEBUG_LISTS
	list_for_each_entry(pos, &filter_block_list, list)
		BUG_ON(pos == block);
#endif

	list_add_tail(&block->list, &filter_block_list);
	
	up(&filter_block_list_sem);
}

static void demux_config_section_filters(struct demux_queue *queue)
{
	struct filter_block *block, *prev;
	stbx25xx_demux_val reg;
	
	block = list_first_entry(&queue->filters, struct filter_block, list);
	reg.raw = 0;
	reg.sfctrl.sfid	= block->sfid;
	reg.sfctrl.enbl	= 1;
	reg.sfctrl.nc	= 1;
	
	list_for_each_entry_from(block, &queue->filters, list) {
		prev = list_entry(block->list.prev, struct filter_block, list);
		reg.sfctrl.nf = prev->index;
		set_demux_reg(SECFLT_CTRL(prev->index), reg);
		
		reg.raw = 0;
		reg.sfctrl.sfid	= block->sfid;
		reg.sfctrl.enbl	= 1;
		reg.sfctrl.nc	= 1;
		reg.sfctrl.nf	= block->index;
	}
	set_demux_reg(SECFLT_CTRL(reg.sfctrl.nf), reg);
	
	list_for_each_entry(block, &queue->filters, list) {
		set_demux_reg_raw(SECFLT_VAL(block->index), block->value);
		set_demux_reg_raw(SECFLT_MASK(block->index), block->mask);
		set_demux_reg_raw(SECFLT_POS(block->index), block->positive);
	}
}

static int demux_install_section_filter(struct dvb_demux_feed *feed)
{
	int i, ret = 0;
	struct filter_block *block;
	struct demux_queue *queue;
	
	if(feed->pid >= 0x1fff)
		return -EINVAL;
	
	if(mutex_lock_killable(&queues_mutex)) {
		return -ERESTARTSYS;
	}
	
	for(i=0; i<STBx25xx_QUEUE_COUNT; i++) {
		if(demux_queues[i].pid == feed->pid) {
			queue = &demux_queues[i];
			goto found;
		}
	}
	
	queue = demux_alloc_queue();
	if(!queue) {
		ret = -EBUSY;
		goto exit;
	}
	
found:
	if(queue->pid != 0x1fff) {
		err("%s: Duplicating PID values unallowed.", __func__);
		ret = -EINVAL;
		goto exit;
	}

#ifdef STBx25xx_SW_SECT_FILT
	
	feed->priv = NULL;
	queue->feed = feed;
	
	ret = demux_install_queue(queue->handle, feed->pid, 15, QCFG_DT_TSPKT | QUEUE_CONFIG_SWDEMUX, 0);
	if(ret)
		demux_free_queue(queue);
	
	goto exit;
		
#endif
	
	INIT_LIST_HEAD(&queue->filters);
	
	for(i = 0; i < 4; i++) {
		block = demux_alloc_filter_block();
		if(!block)
			break;
		list_add_tail(&block->list, &queue->filters);
	}
	
	if(i != 4) {
		while(!list_empty(&queue->filters)) {
			block = list_first_entry(&queue->filters, struct filter_block, list);
			list_del(&block->list);
			demux_free_filter_block(block);
		}
		
		dprintk("%s: No more hardware section filters available, falling back to software\n", __func__);
		
		feed->priv = NULL;
		queue->feed = feed;
		
		ret = demux_install_queue(queue->handle, feed->pid, 15, QCFG_DT_TSPKT | QUEUE_CONFIG_SWDEMUX, 0);
		if(ret)
			demux_free_queue(queue);
		
		goto exit;
	}
	
	block = list_first_entry(&queue->filters, struct filter_block, list);
	block->value = (feed->filter->filter.filter_value[0] << 24) |
			(feed->filter->filter.filter_value[3] << 16) | 
			(feed->filter->filter.filter_value[4] << 8) | 
			(feed->filter->filter.filter_value[5]);
			
	block->mask = (feed->filter->filter.filter_mask[0] << 24) |
			(feed->filter->filter.filter_mask[3] << 16) | 
			(feed->filter->filter.filter_mask[4] << 8) | 
			(feed->filter->filter.filter_mask[5]);
			
	block->positive = (feed->filter->maskandmode[0] << 24) |
			(feed->filter->maskandmode[3] << 16) | 
			(feed->filter->maskandmode[4] << 8) | 
			(feed->filter->maskandmode[5]);
			
	block->positive = ~block->positive;
			
	block->sfid = feed->filter->index;
	
	i = 6;
	list_for_each_entry_from(block, &queue->filters, list) {
		block->value = (feed->filter->filter.filter_value[i] << 24) |
				(feed->filter->filter.filter_value[i+1] << 16) | 
				(feed->filter->filter.filter_value[i+2] << 8) | 
				(feed->filter->filter.filter_value[i+3]);
				
		block->mask = (feed->filter->filter.filter_mask[i] << 24) |
				(feed->filter->filter.filter_mask[i+1] << 16) | 
				(feed->filter->filter.filter_mask[i+2] << 8) | 
				(feed->filter->filter.filter_mask[i+3]);
				
		block->positive = (feed->filter->maskandmode[i] << 24) |
				(feed->filter->maskandmode[i+1] << 16) | 
				(feed->filter->maskandmode[i+2] << 8) | 
				(feed->filter->maskandmode[i+3]);
				
		block->positive = ~block->positive;
				
		block->sfid = feed->filter->index;
		i += 4;
	}
	
	feed->priv = NULL;
	queue->feed = feed;
	
	demux_config_section_filters(queue);
	ret = demux_install_queue(queue->handle, feed->pid, 15, 
				   QCFG_DT_TBSEC_FLT | QUEUE_CONFIG_SECFLT, 0);
	
	if(ret) {
		while(!list_empty(&queue->filters)) {
			block = list_first_entry(&queue->filters, struct filter_block, list);
			list_del(&block->list);
			demux_free_filter_block(block);
		}
		demux_free_queue(queue);
	}
		
exit:
	mutex_unlock(&queues_mutex);
	
	return ((ret) ? ret : queue->handle);
}

/*
 * demux_remove_user_queue()
 * Remove filter as requested from DVB API and remove the queue
 * when there is no feed associated with it
 */
static int demux_remove_section_filter(struct dvb_demux_feed *feed)
{
	int queue, ret = 0;
	struct dvb_demux_feed *f;
	struct filter_block *block;
	unsigned long flags;
	
	if(!feed)
		return -EINVAL;
	
	queue = feed->index;
	
	if(queue > 31)
		return -EINVAL;
	
	if(mutex_lock_killable(&queues_mutex))
		return -ERESTARTSYS;
	
	f = demux_queues[queue].feed;
	
	if(f == feed) {
		spin_lock_irqsave(&demux_queues[queue].lock, flags);
		demux_queues[queue].feed = feed->priv;
		spin_unlock_irqrestore(&demux_queues[queue].lock, flags);
	} else {
		while(f) {
			if(f->priv == feed) {
				spin_lock_irqsave(&demux_queues[queue].lock, flags);
				f->priv = feed->priv;
				spin_unlock_irqrestore(&demux_queues[queue].lock, flags);
				break;
			}
			f = f->priv;
		}
	}
	
	if(!demux_queues[queue].feed && queue < 30) {
		ret = demux_remove_queue(queue);
	
		if(!(demux_queues[queue].config & QUEUE_CONFIG_SWDEMUX))
			while(!list_empty(&demux_queues[queue].filters)) {
					block = list_first_entry(&demux_queues[queue].filters, struct filter_block, list);
					list_del(&block->list);
					demux_free_filter_block(block);
			}
			
		demux_free_queue(&demux_queues[queue]);
	}
	
	mutex_unlock(&queues_mutex);
	
	return ret;
}

static void demux_init_section_filters(void)
{
	int i;
	
	sema_init(&filter_block_list_sem, 1);
	
	INIT_LIST_HEAD(&filter_block_list);
	for(i = 0; i < STBx25xx_FILTER_BLOCK_COUNT; i++) {
		list_add_tail(&filter_blocks[i].list, &filter_block_list);
		filter_blocks[i].index = i;
	}
}

/*
-------------------------------------------------------------------------------
 */

/**
	PCR clock management
**/

/*
 * demux_set_pcr_pid()
 * Set the PID of stream containing PCR
 */
static void demux_set_pcr_pid(u16 pid)
{
	set_demux_reg_raw(PCRPID, pid);
}

/*
 * demux_get_last_prc()
 * Get the value of the last found PCR
 */
static u64 demux_get_last_pcr(void)
{
	u32 lo, hi;

	hi = get_demux_reg_raw(PCRHI);
	lo = get_demux_reg_raw(PCRLO);
	
	return ((hi << 10) | lo);
}

/*
 * demux_get_stc()
 * Get the current value of STC
 */
static u64 demux_get_stc(void)
{
	u32 lo, hi;

	hi = get_demux_reg_raw(STCHI);
	lo = get_demux_reg_raw(STCLO);
	
	return ((hi << 10) | lo);
}

/*
 * demux_get_latched_stc()
 * Get the value of latched STC
 */
static u64 demux_get_latched_stc(void)
{
	u32 lo, hi;
	
	hi = get_demux_reg_raw(LSTCHI);
	lo = get_demux_reg_raw(LSTCLO);
	
	return ((hi << 10) | lo);
}

int demux_get_stc_for_sync(u32 *val)
{
	short rc;
	short i;
	unsigned long stc_high=0;
	unsigned long lstc_high=0;
	unsigned long lstc_high2=0;
	unsigned long diff;

	for(i=0; i<2; i++)
	{
		lstc_high  = get_demux_reg_raw(LSTCHI);
		stc_high   = get_demux_reg_raw(STCHI);
		lstc_high2 = get_demux_reg_raw(LSTCHI);;
		if(lstc_high == lstc_high2)
			break;
	}
	
	/*------------------------------------------------------------------------+
	|  Compare the Two Latched STC Values Read, A PCR must have been
	|  Received if their Different
	+------------------------------------------------------------------------*/
	rc = (lstc_high != lstc_high2) ? -1 : 0;

	/*------------------------------------------------------------------------+
	|   The STC is considered valid if a recent PCR has been received.
	|   Each PCR received is loaded into the STC Latched register.  So, we
	|   compare the values of the STC Latched and the STC registers whose
	|   difference should be within the expected arrival rate of PCR's.
	|
	|   The calculation is as follows:
	|       STC clock is 27 MHZ, PCR arrival rate=100ms
	|       The upper 32 bits of the STC are modulo 600
	|
	|       difference    =  2.7E6 / 600
	|                     =  4500
	|
	|   Add a 1% margin so we don't hit the edge condition exactly
	|   and are somewhat tolerant of non-compliant streams.
	|
	|      4500 x  1.01  = 4545
	|
	|   STC is valid if | STC - STCHOLD | < 4545
	|
	|   Note: We have to account for the STC wrapping.
	+------------------------------------------------------------------------*/
	/*------------------------------------------------------------------------+
	|  Since these are both unsigned, there is not a Wrapping Issue
	+------------------------------------------------------------------------*/
	if (rc == 0) {
		diff = stc_high - lstc_high;
		if (diff > 4545)
			rc = -1;
	}

	*val = stc_high;

	return(rc);
}

/*
 * demux_sync_av
 * A/V STC synchronization routine
 */
static void demux_sync_av(struct stbx25xx_demux_data *dmx, int irq)
{
	u32 stc_high;
	
	if(demux_get_stc_for_sync(&stc_high))
		return;
	
	info("Updating A/V STC with PCR 0x%08x0", stc_high);
	
	stbx25xx_audio_sync_stc(0, stc_high + DECODER_SHIFT);
	stbx25xx_video_sync_stc(0, stc_high + DECODER_SHIFT);
	
//	printk("Video sync => %d\n", stc_high);
}

/*
 * demux_adjust_clock
 * Clock adjustment routine for PCR/STC synchronization
 */
static void demux_adjust_clock(struct stbx25xx_demux_data *dmx, int irq)
{
	unsigned        overflow;         /* Delta Overflow bit                  */
	unsigned long   delta;            /* Delta or Delta Magnitude            */

	short           rate_adjust;      /* Adjustment to PWM based on frequency*/
	short           pwm_adjust;       /* The amount to adjust the pwm        */
	unsigned long   pwm;              /* Current PWM value                   */
	unsigned long   new_pwm;          /* Updated PWM value                   */
	short           soft_overflow;    /* software algorithm overflow         */
	long            delta_pcr;        /* Difference between the current      */
	long            delta_stc;        /* and previous values                 */

	long            temp1;
	long            temp2;

	long            value_adjust;     /* Adjustment to PWM based on delta    */

	unsigned long   stc_high;
	unsigned long   stc_low;

	unsigned long   pcr_high;
	unsigned long   pcr_low;

	unsigned long   stc;
	unsigned long   pcr;

	stbx25xx_demux_val p_delta;

	/*------------------------------------------------------------------------+
	|  Read the STC, PCR, and delta values
	+------------------------------------------------------------------------*/

	pcr_high = get_demux_reg_raw(PCRHI);
	pcr_low  = get_demux_reg_raw(PCRLO);

	stc_high = get_demux_reg_raw(LSTCHI);
	stc_low  = get_demux_reg_raw(LSTCLO);

	p_delta   = get_demux_reg(PCRSTCD);
	delta    = p_delta.pcrstcd.delta;
	overflow = p_delta.pcrstcd.ovfl;

	if(overflow)
		delta = ~0;

	/*------------------------------------------------------------------------+
	|   Make sure the clock recovery registers are consistent
	|   Later the value of Incons_data can be returned as
	|   part of the status of the driver.
	|   This is a candidate to be removed later.
	+------------------------------------------------------------------------*/

	if(pcr_low != get_demux_reg_raw(PCRLO))
		return;

	/*------------------------------------------------------------------------+
	|  Track number of PCRS Since STC loaded
	+------------------------------------------------------------------------*/

	if(irq == DEMUX_IRQ_STC)
		dmx->XpClkNpcrs = 0;                        /* Initialize to 0         */

	if(dmx->XpClkNpcrs <= PCRS_HIGH_GAIN)
		dmx->XpClkNpcrs++;

	/*------------------------------------------------------------------------+
	|   Normalize the STC and PCR
	|   This drops the upper 10 bits of both the PCR and the STC
	|   This also detects if either the PCR or STC has wrapped
	|   and other has not.  In either case the software algorithm
	|   doesn't run.
	+------------------------------------------------------------------------*/

	stc = (((stc_high & 0x003FFFFF) * 2) + ((stc_low & 0x200) >> 9)) * 300;
	stc = stc + (stc_low & 0x1FF);

	pcr = (((pcr_high & 0x003FFFFF) * 2) + ((pcr_low & 0x200) >> 9)) * 300;
	pcr = pcr + (pcr_low & 0x1FF);


	/*------------------------------------------------------------------------+
	|  Check if the upper 10 bits are the same.
	|  The software clock recovery algorithm is not used if
	|  soft_overflow occurs.
	|  Also check if hardware overflow has occured and treat
	|  like a software overflow - pag 7/16/99
	+------------------------------------------------------------------------*/

	if(overflow || (stc_high & 0xFFC00000) != (pcr_high & 0xFFC00000)) {
		stc = stc_high & 0xFFC00000;
		pcr = pcr_high & 0xFFC00000;

		soft_overflow = 1;

		dmx->XpClkErrs++;                  /* Increment the global error count */

		/*--------------------------------------------------------------------+
		|  if there were two consecutive errors, force a PCR reload
		+--------------------------------------------------------------------*/

		if(dmx->wXpClkPrevErr == 1) {
			set_demux_reg_raw(PCRPID, get_demux_reg_raw(PCRPID));
			dmx->wXpClkPrevErr = 0;
		} else {
			dmx->wXpClkPrevErr = 1;
		}
	} else {
		soft_overflow   = 0;
		dmx->wXpClkPrevErr = 0;
	}

	/*------------------------------------------------------------------------+
	|   Run the software algorithm if there are no errors
	|   Adjust auto PWM if the delta is large
	+------------------------------------------------------------------------*/
	if(soft_overflow == 0) {
		delta_pcr = pcr - dmx->XpClkPrevPcr;
		delta_stc = stc - dmx->XpClkPrevStc;

		temp1 = delta_pcr - delta_stc;
		temp2 = 27000000 / delta_pcr;

		rate_adjust = (temp1 * temp2) >> RATE_GAIN;
	//lingh changed it
		value_adjust = (pcr - stc) >> DELTA_GAIN;

		pwm_adjust = rate_adjust + value_adjust;

		/*--------------------------------------------------------------------+
		|  Once the delta is small enough increase the threshold
		|  and use only autopwm until this function is called
		|  again, then increase the threshold.
		+--------------------------------------------------------------------*/

		if((delta < LOW_THRESHOLD) && (dmx->lXpClkPrevDelta < LOW_THRESHOLD)) {
			dmx->uwXpClkThreshold = HIGH_THRESHOLD;
			set_demux_reg_raw(PCRSTCT, dmx->uwXpClkThreshold);
		} else if(dmx->uwXpClkThreshold != 0) {
			dmx->uwXpClkThreshold = 0;
			set_demux_reg_raw(PCRSTCT, dmx->uwXpClkThreshold);
		}

		dmx->XpClkPrevStc   = stc;
		dmx->XpClkPrevPcr   = pcr;
		dmx->lXpClkPrevDelta = delta;

		/*--------------------------------------------------------------------+
		|  Restrict the rate of change of the PWM value.
		|  For the first few PCRs received after the STC is
		|  loaded the clamp value is larger so that the local
		|  clock moves closer to the multiplexor clock quickly.
		|  After that reduce use a smaller clamp value.
		+--------------------------------------------------------------------*/

		if (dmx->XpClkNpcrs <= PCRS_HIGH_GAIN) {
			/* clamp adjustment */
			if(pwm_adjust > STC_LOAD_PWM_CLAMP)
				pwm_adjust = STC_LOAD_PWM_CLAMP;
			else if(pwm_adjust < -STC_LOAD_PWM_CLAMP)
				pwm_adjust = -STC_LOAD_PWM_CLAMP;
		} else {
			if(pwm_adjust > PWM_CLAMP)
				pwm_adjust = PWM_CLAMP;
			else if(pwm_adjust < -PWM_CLAMP)
				pwm_adjust = -PWM_CLAMP;
		}

		pwm     = get_demux_reg_raw(PWM);

		new_pwm = pwm + pwm_adjust;

		/*--------------------------------------------------------------------+
		|  PWM range is actually a signed 12 bit number
		|  whose range is 0x800 - 0x7ff.  So after calculating
		|  the new pwm, make sure we don't exceed the above bounds
		+--------------------------------------------------------------------*/

		if((pwm <= 0x07ff) && (new_pwm > 0x07ff) && (pwm_adjust > 0))
			new_pwm = 0x07ff;
		else if((pwm >= 0x0800) && (new_pwm < 0x0800) && (pwm_adjust < 0))
			new_pwm = 0x0800;

		set_demux_reg_raw(PWM, new_pwm); /* Update the PWM register  */
	}
}

/* demux_pcr_stc_irq() */
static void demux_pcr_stc_irq(struct stbx25xx_demux_data *dmx, int irq)
{	
	if(dmx->adjust_clk)
		demux_adjust_clock(dmx, irq);
	
	if(dmx->sync_av)
		demux_sync_av(dmx, irq);
}

static void demux_pcr_sync_start(struct stbx25xx_demux_data *dmx)
{
	unsigned long flags;
	
//	demux_sync_av(dmx, DEMUX_IRQ_PCR);
	
	local_irq_save(flags);
	
	dmx->sync_av = 1;
	demux_enable_irq(DEMUX_IRQ_STC);
	demux_enable_irq(DEMUX_IRQ_PCR);
	
	local_irq_restore(flags);
	
	info("A/V PCR to STC sync enabled.");
}

static void demux_pcr_sync_stop(struct stbx25xx_demux_data *dmx)
{
	unsigned long flags;
	
//	stbx25xx_video_disable_sync();
	
	local_irq_save(flags);
	
	dmx->sync_av = 0;
	if(!dmx->adjust_clk) {
		demux_disable_irq(DEMUX_IRQ_STC);
		demux_disable_irq(DEMUX_IRQ_PCR);
	}
	
	local_irq_restore(flags);
	
	info("A/V PCR to STC sync disabled.");
}

/* demux_init_clk_mgmt() */
static void demux_clk_mgmt_start(struct stbx25xx_demux_data *dmx)
{
	stbx25xx_demux_val reg;
	unsigned long flags;
	
	dmx->XpClkErrs = 0;
        dmx->XpClkNpcrs = PCRS_HIGH_GAIN;
	
	set_demux_reg_raw(PCRSTCT, 0);
	
	local_irq_save(flags);

	reg = get_demux_reg(CONFIG1);
	reg.config1.apwma = 1;
	set_demux_reg(CONFIG1, reg);
	
	dmx->adjust_clk = 1;
	demux_enable_irq(DEMUX_IRQ_STC);
	demux_enable_irq(DEMUX_IRQ_PCR);
	
	local_irq_restore(flags);
}

/* demux_clk_mgmt_deinit() */
static void demux_clk_mgmt_stop(struct stbx25xx_demux_data *dmx)
{
	unsigned long flags;
	stbx25xx_demux_val reg;
	
	local_irq_save(flags);
	
	reg = get_demux_reg(CONFIG1);
	reg.config1.apwma = 0;
	set_demux_reg(CONFIG1, reg);
	
	dmx->adjust_clk = 0;
	if(!dmx->sync_av) {
		demux_disable_irq(DEMUX_IRQ_PCR);
		demux_disable_irq(DEMUX_IRQ_STC);
	}
	
	local_irq_restore(flags);
}

/*
-------------------------------------------------------------------------------
 */

/**
	Transport DMA (clip mode) support
**/

static int clipmode = 0;

static void demux_suspend_queue(struct demux_queue *queue)
{
	unsigned long flags;
		
	if(queue->state == QUEUE_STATE_FREE)
		return;
	
	if(queue->state == QUEUE_STATE_SUSPENDED)
		return;
	
	demux_disable_queue_irq(queue->handle);
	demux_stop_queue(queue->handle);
	
	spin_lock_irqsave(&queue->lock, flags);
	queue->state = QUEUE_STATE_STOPPING;
	spin_unlock_irqrestore(&queue->lock, flags);
	
	cancel_work_sync(&queue->work);
	
	queue->state = QUEUE_STATE_SUSPENDED;
}

static void demux_resume_queue(struct demux_queue *queue)
{
	if(queue->state != QUEUE_STATE_SUSPENDED)
		return;
	
	queue->state = QUEUE_STATE_ACTIVE;
	
	queue_work(queue->dmx->workqueue, &queue->work);
	
	demux_enable_queue_irq(queue->handle);
	set_demux_reg_raw(PIDFLT(queue->handle), get_demux_reg_raw(PIDFLT(queue->handle)));
}

static int demux_enable_clip_mode(void)
{
	int i;
	
	if(mutex_lock_killable(&queues_mutex))
		return -ERESTARTSYS;
	
	demux_disable_sync();
	demux_change_input(DEMUX_IN_TSDMA);
	
	for(i=0; i<STBx25xx_MAX_FEED; i++)
		demux_suspend_queue(&demux_queues[i]);
	
	demux_enable_sync();
	
	mutex_unlock(&queues_mutex);
	
	return 0;
}

static int demux_disable_clip_mode(void)
{
	int i;
	
	if(mutex_lock_killable(&queues_mutex))
		return -ERESTARTSYS;
	
	demux_disable_sync();
	demux_change_input(DEMUX_IN_CI0);
	
	for(i=0; i<STBx25xx_MAX_FEED; i++)
		demux_resume_queue(&demux_queues[i]);
	
	demux_enable_sync();
	
	mutex_unlock(&queues_mutex);
	
	return 0;
}

static int demux_feed_clip(struct stbx25xx_demux_data *dmx, const u8 *buf, size_t size)
{
	// TODO: Implement clip mode data feeding (Transport DMA)
	
	return -EINVAL;
}

/*
-------------------------------------------------------------------------------
 */

/**
	Hardware initialzation routines
**/

/* demux_load_config */
static void demux_load_default_config(void)
{
	stbx25xx_demux_val config1, config2, config3, control1, pbuflvl;

/* Read the registers */
	config1 = get_demux_reg(CONFIG1);
	config2 = get_demux_reg(CONFIG2);
	config3 = get_demux_reg(CONFIG3);
	control1 = get_demux_reg(CONTROL1);
	pbuflvl = get_demux_reg(PBUFLVL);

/* CONFIG1 */
	config1.config1.vpu	= config1_def.config1.vpu;
	config1.config1.apu	= config1_def.config1.apu;
	config1.config1.tstoe	= config1_def.config1.tstoe;
	config1.config1.tsclkp	= config1_def.config1.tsclkp;
	config1.config1.tsdp	= config1_def.config1.tsdp;
	config1.config1.tsvp	= config1_def.config1.tsvp;
	config1.config1.syncd	= config1_def.config1.syncd;
	config1.config1.bb	= config1_def.config1.bb;
	config1.config1.syncl	= config1_def.config1.syncl;

/* CONFIG2 */
	config2.config2.ved	= config2_def.config2.ved;
	config2.config2.acpm	= config2_def.config2.acpm;
	config2.config2.vcpm	= config2_def.config2.vcpm;
	config2.config2.mwe	= config2_def.config2.mwe;
	config2.config2.sa	= config2_def.config2.sa;
	config2.config2.atsed	= config2_def.config2.atsed;
	config2.config2.atbd	= config2_def.config2.atbd;
	config2.config2.accd	= config2_def.config2.accd;
	config2.config2.vtsed	= config2_def.config2.vtsed;
	config2.config2.vtbd	= config2_def.config2.vtbd;
	config2.config2.vccd	= config2_def.config2.vccd;

/* CONFIG3 */
	config3.config3.insel	= config3_def.config3.insel;
	
/* CONTROL1 */
	control1.control1.sbe	= control1_def.control1.sbe;
	control1.control1.pbe	= control1_def.control1.pbe;
	control1.control1.sdop	= control1_def.control1.sdop;
	
/* PBUFLVL */
	pbuflvl.pbuflvl.qpt	= pbuflvl_def.pbuflvl.qpt;
	pbuflvl.pbuflvl.apt	= pbuflvl_def.pbuflvl.apt;
	pbuflvl.pbuflvl.vpt	= pbuflvl_def.pbuflvl.vpt;
	
/* Write the registers with Sync Enable bit cleared */
	control1.control1.se	= 0;
	set_demux_reg(CONFIG1, config1);
	set_demux_reg(CONFIG2, config2);
	set_demux_reg(CONFIG3, config3);
	set_demux_reg(CONTROL1, control1);
	set_demux_reg(PBUFLVL, pbuflvl);
	
/* Set Sync Enable bit if wanted to */
	control1.control1.se	= control1_def.control1.se;
	set_demux_reg(CONTROL1, control1);
}

/* demux_reset */
static int demux_reset(void)
{
	stbx25xx_demux_val reg;
	int i;
	
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
	demux_load_default_config();
	
	return 0;
}

/*
-------------------------------------------------------------------------------
 */

/**
	Interrupts management
**/

static u32 demux_irq_mask;
static u32 demux_festat_irq_mask;
static u32 demux_audio_irq_mask;
static u32 demux_video_irq_mask;
static u32 demux_descr_irq_mask;
spinlock_t demux_irq_lock;

static demux_irq_handler_t irq_handler_table[STBx25xx_DEMUX1_IRQ_COUNT];
static u32 irq_stats[STBx25xx_DEMUX1_IRQ_COUNT];

/*
 * NOTE: IRQ HANDLER
 * demux_irq_handler()
 *
 * Main IRQ handler calling all 2nd level handlers if needed
 */
static irqreturn_t demux_irq_handler(int irq, void *dev_id)
{
	int i;
	struct stbx25xx_demux_data *dmx = dev_id;
	u32 first_stat = mfdcr(DEMUX_INT) & demux_irq_mask;	/* AND with the mask to be safe... */
	mtdcr(DEMUX_INT, first_stat);				/* Clear instantly to minimize interrupt loss */
	
	for(i = 0; (i < STBx25xx_DEMUX1_IRQ_COUNT) && first_stat; i++, first_stat <<= 1) {
		if(first_stat & 0x80000000) {
			irq_stats[i]++;
			if(irq_handler_table[i])
				irq_handler_table[i](dmx, i);
		}
//		info("%s: i=%d: first_stat=0x%08x\n", __func__, i, first_stat);
	}
	
	return IRQ_HANDLED;
}

/*
 * NOTE: IRQ HANDLER
 * demux_frontend_irq_handler()
 *
 * Handles (will handle) following interrupts:
 * Sync Locked, Sync Lost, TS Header Error, Missing Sync, Packet Buffer Overflow,
 * Transport Stream Error, Video Packet Overflow, Audio Packet Overflow, Queue Packet Overflow,
 * Multiple PID Filter Matches, First PCR
 */
static void demux_frontend_irq_handler(struct stbx25xx_demux_data *dmx, int irq)
{
	stbx25xx_demux_val reg;
	
	reg = get_demux_reg(FESTAT);
	reg.raw &= demux_festat_irq_mask;
	set_demux_reg(FESTAT, reg);
	
	if(reg.festat.tshe)
		info("Demux IRQ: Frontend - TS Header Error");
	
	if(reg.festat.pbo)
		info("Demux IRQ: Frontend - Packet Buffer Overflow");
	
	if(reg.festat.tse)
		info("Demux IRQ: Frontend - Transport Stream Error");
	
	if(reg.festat.vpo)
		info("Demux IRQ: Frontend - Video Packet Overflow");
	
	if(reg.festat.apo)
		info("Demux IRQ: Frontend - Audio Packet Overflow");
	
	if(reg.festat.qpo)
		info("Demux IRQ: Frontend - Queue Packet Overflow");
	
	if(reg.festat.mpfm)
		info("Demux IRQ: Frontend - Multiple PID Filter Matches");
}

/*
 * NOTE: IRQ HANDLER
 * demux_plb_err_handler()
 *
 * Handles memory errors (hopefully not going to happen)
 */
static void demux_plb_err_handler(struct stbx25xx_demux_data *dmx, int irq)
{
	info("Demux IRQ: PLB Error");
}

/*
 * NOTE: IRQ HANDLER
 * demux_queues_irq_handler()
 *
 * Handles interrupts of all queues
 */
static void demux_queues_irq_handler(struct stbx25xx_demux_data *dmx, int irq)
{
	int i;
	u32 reg;
	u32 mask = get_demux_reg_raw(QINT) & demux_queues_irq_mask;
	set_demux_reg_raw(QINT, mask);
	
	dprintk("%s: demux_queues_irq_mask = 0x%08x\n", __func__, demux_queues_irq_mask);
	
	for(i=STBx25xx_QUEUE_COUNT-1; i>=0 && mask; i--, mask>>=1) {
		if(mask & 1) {
			reg = get_demux_reg_raw(QSTATA(i)) & demux_qstat_irq_mask;
			set_demux_reg_raw(QSTATA(i), reg & ~QUEUE_RPI);
			demux_process_queue_irq(i, reg);
		}
	}
}

static void demux_enable_irq(int demux_irq)
{
	unsigned long flags;
	
	if(demux_irq >= STBx25xx_DEMUX1_IRQ_COUNT || demux_irq < 0)
		return;

	local_irq_save(flags);
	
	if(!irq_handler_table[demux_irq]) {
		err("demux_enable_irq: Tried to enable demux IRQ %d, which does not have a handler", demux_irq);
		goto exit;
	}
	
	demux_irq_mask |= 1 << (31 - demux_irq);
	set_demux_reg_raw(INTMASK, demux_irq_mask);
	
exit:
	local_irq_restore(flags);
}

static void demux_disable_irq(int demux_irq)
{
	unsigned long flags;
	
	if(demux_irq >= STBx25xx_DEMUX1_IRQ_COUNT || demux_irq < 0)
		return;

	local_irq_save(flags);
		
	demux_irq_mask &= ~(1 << (31 - demux_irq));
	set_demux_reg_raw(INTMASK, demux_irq_mask);
	
	local_irq_restore(flags);
}

static void demux_set_handler(int demux_irq, demux_irq_handler_t demux_handler)
{
	unsigned long flags;
	
	if(demux_irq >= STBx25xx_DEMUX1_IRQ_COUNT || demux_irq < 0)
		return;
	
	if(irq_handler_table[demux_irq] && demux_handler)
		warn("demux_set_handler: Replacing old handler of demux IRQ %d", demux_irq);
	
	local_irq_save(flags);
	irq_handler_table[demux_irq] = demux_handler;
	local_irq_restore(flags);
}

static int demux_init_irq(struct stbx25xx_demux_data *dmx)
{
	int ret = 0;
	struct stbx25xx_dvb_data *dvb = container_of(dmx, struct stbx25xx_dvb_data, demux);
	
	memset(irq_handler_table, 0, sizeof(demux_irq_handler_t) * STBx25xx_DEMUX1_IRQ_COUNT);
	memset(irq_stats, 0, sizeof(u32) * STBx25xx_DEMUX1_IRQ_COUNT);
		
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
	
	if((ret = request_irq(dvb->irq_num[STBx25xx_IRQ_DEMUX], demux_irq_handler, IRQF_TRIGGER_HIGH, "demux", dmx)) != 0) {
		err("Failed to request demux irq: error %d", ret);
		return ret;
	}
	
	demux_set_handler(DEMUX_IRQ_FRONTEND, demux_frontend_irq_handler);
	demux_enable_irq(DEMUX_IRQ_FRONTEND);
	demux_festat_irq_mask |= 0x1928;
	set_demux_reg_raw(FEIMASK, demux_festat_irq_mask);
	
	demux_set_handler(DEMUX_IRQ_QUEUES, demux_queues_irq_handler);
	demux_enable_irq(DEMUX_IRQ_QUEUES);
	
	demux_set_handler(DEMUX_IRQ_PLB_ERR, demux_plb_err_handler);
	demux_enable_irq(DEMUX_IRQ_PLB_ERR);
	
	demux_set_handler(DEMUX_IRQ_STC, demux_pcr_stc_irq);
	demux_set_handler(DEMUX_IRQ_PCR, demux_pcr_stc_irq);
	
	return 0;
}

/*
-------------------------------------------------------------------------------
 */

/**
	DVB API interface
**/

int (*dvbdmx_disconnect_frontend)(struct dmx_demux *demux);
int (*dvbdmx_connect_frontend)(struct dmx_demux *demux,
				   struct dmx_frontend *frontend);

/* stbx25xx_demux_connect_frontend */
int stbx25xx_demux_connect_frontend(struct dmx_demux* demux,
				 struct dmx_frontend* frontend)
{
	int err;

	if ((err = dvbdmx_connect_frontend(demux, frontend)) < 0)
		return err;

	if (demux->frontend->source == DMX_MEMORY_FE) {
		if ((err = demux_enable_clip_mode()) < 0)
			return err;
		clipmode = 1;
	}

	return 0;
}

/* stbx25xx_demux_disconnect_frontend */
int stbx25xx_demux_disconnect_frontend(struct dmx_demux* demux)
{
	int err;

	if (demux->frontend->source == DMX_MEMORY_FE) {
		if ((err = demux_disable_clip_mode()) < 0)
			return err;
		clipmode = 0;
	}

	return dvbdmx_disconnect_frontend(demux);
}

/* stbx25xx_demux_get_stc */
int stbx25xx_demux_get_stc(struct dmx_demux* demux, unsigned int num,
			u64 *stc, unsigned int *base)
{
	if(num != 0)
		return -EINVAL;
	
	*stc = demux_get_latched_stc();
	
	return 0;
}

int stbx25xx_demux_write_to_decoder(struct dvb_demux_feed *dvbdmxfeed, const u8 *buf, size_t count)
{
	struct stbx25xx_demux_data *dmx = dvbdmxfeed->demux->priv;
	
	if(dvbdmxfeed->demux->dmx.frontend->source != DMX_MEMORY_FE)
		return 0;
	
	demux_feed_clip(dmx, buf, count);
	
	return 0;
}

/* stbx25xx_demux_start_feed */
int stbx25xx_demux_start_feed(struct dvb_demux_feed *feed)
{
	int queue;
	u16 config;
	struct stbx25xx_demux_data *dmx = feed->demux->priv;
	
	if(feed->type == DMX_TYPE_TS) {
		if(feed->ts_type & TS_DECODER) {				
			if(feed->pes_type == DMX_TS_PES_AUDIO)
				return demux_audio_channel_change(feed->pid);
			
			if(feed->pes_type == DMX_TS_PES_VIDEO)
				return demux_video_channel_change(feed->pid);
			
			if(feed->pes_type == DMX_TS_PES_PCR) {
				dmx->XpClkNpcrs = 0;
				demux_pcr_sync_start(dmx);
				demux_set_pcr_pid(feed->pid);
				return 0;
			}
			
			err("%s: PES type unsupported by decoder", __func__);	
			return -EINVAL;
		}
		
		if(feed->demux->dmx.frontend->source == DMX_MEMORY_FE)
			return 0;
	
		config = 0;
		if(feed->ts_type & TS_PAYLOAD_ONLY) {
			config = QCFG_DT_PAYLOAD;
		}
		
		if((queue = demux_install_user_queue(feed, 15, config)) < 0)
			return queue;
		
		feed->index = queue;
		
		return 0;
	} else if(feed->type == DMX_TYPE_SEC) {
		if(feed->demux->dmx.frontend->source == DMX_MEMORY_FE)
			return 0;
		
		if((queue = demux_install_section_filter(feed)) < 0)
			return queue;
		
		feed->index = queue;
		
		return 0;
	} else if(feed->type == DMX_TYPE_PES) {
		err("%s: Unsupported feed type (DMX_TYPE_PES)", __func__);
		return -EINVAL;
	}
	
	err("%s: Unknown feed type", __func__);
	
	return -EINVAL;
}

/* stbx25xx_demux_stop_feed */
int stbx25xx_demux_stop_feed(struct dvb_demux_feed *feed)
{
	struct stbx25xx_demux_data *dmx = feed->demux->priv;
	
	if(feed->type == DMX_TYPE_TS) {
		if(feed->ts_type & TS_DECODER) {
			if(feed->pes_type == DMX_TS_PES_AUDIO) {
				return demux_audio_channel_change(0x1FFF);
			}
			
			if(feed->pes_type == DMX_TS_PES_VIDEO) {
				return demux_video_channel_change(0x1FFF);
			}
			
			if(feed->pes_type == DMX_TS_PES_PCR) {
				demux_set_pcr_pid(0x1FFF);
				demux_pcr_sync_stop(dmx);
				return 0;
			}
			
			return -EINVAL;
		} else {
			if(feed->demux->dmx.frontend->source == DMX_MEMORY_FE)
				return 0;
			
			return demux_remove_user_queue(feed);
		}
	} else if(feed->type == DMX_TYPE_SEC) {
		if(feed->demux->dmx.frontend->source == DMX_MEMORY_FE)
			return 0;
		
		return demux_remove_section_filter(feed);
	} 
	
	return -EINVAL;
}

/*
-------------------------------------------------------------------------------
 */

/**
	procfs entry
**/

#define DEMUX_PROC_IRQS_NAME	"demux_interrupts"
#define DEMUX_PROC_QUEUES_NAME	"demux_queues"

static struct proc_dir_entry *demux_proc_irqs;
static struct proc_dir_entry *demux_proc_queues;

static char *demux_proc_config_table[] = {
	"PESL",
	"DE",
	"SCPC",
	"APUS",
	"SYSTEM",
	"SECFLT",
	"SWDEMUX",
};

static char *demux_proc_state_table[] = {
	"FREE",
	"STARTING",
	"ACTIVE",
	"STOPPING",
	"SUSPENDED",
	"SYSTEM",
};

static int demux_show_queues(struct seq_file *p, void *v)
{
	int i = *(loff_t *)v, j;
	unsigned long flags;
	u32 pid, data_count;
	phys_addr_t phys;
	void *virt, *ptr;
	size_t size;
	u16 config;
	u8 state;
	struct demux_queue *queue = &demux_queues[i];
	
	spin_lock_irqsave(&queue->lock, flags);
	pid = queue->pid;
	data_count = queue->data_count;
	phys = queue->phys_addr;
	virt = queue->addr;
	ptr = queue->ptr;
	size = queue->size;
	config = queue->config;
	state = queue->state;
	spin_unlock_irqrestore(&queue->lock, flags);
	
	seq_printf(p, "Queue %d:\n", i);
	seq_printf(p, "PID     : 0x%04x\n", pid);
	seq_printf(p, "Memory  : 0x%08x @ 0x%p\n", phys, virt);
	seq_printf(p, "Size    : %d bytes\n", size);
	seq_printf(p, "Config  : ");
	config >>= 4;
	j = 0;
	while(config && j < ARRAY_SIZE(demux_proc_config_table)) {
		if(config & 1) {
			seq_printf(p, demux_proc_config_table[j]);
			seq_printf(p, " ");
		}
		
		j++;
		config >>= 1;
	}
	seq_printf(p, "\n");
	seq_printf(p, "State   : %s\n", demux_proc_state_table[state]);
	seq_printf(p, "Recvd   : %d bytes\n", data_count);
	seq_printf(p, "\n");
	
	return 0;
}

static void *demux_seq_qstart(struct seq_file *f, loff_t *pos)
{
	return (*pos < STBx25xx_QUEUE_COUNT) ? pos : NULL;
}

static void *demux_seq_qnext(struct seq_file *f, void *v, loff_t *pos)
{
	(*pos)++;
	if (*pos >= STBx25xx_QUEUE_COUNT)
		return NULL;
	return pos;
}

static void demux_seq_qstop(struct seq_file *f, void *v)
{
	/* Nothing to do */
}

static const struct seq_operations demux_seq_qops = {
	.start = demux_seq_qstart,
	.next  = demux_seq_qnext,
	.stop  = demux_seq_qstop,
	.show  = demux_show_queues
};

static int demux_qopen(struct inode *inode, struct file *file)
{
	return seq_open(file, &demux_seq_qops);
}

static const struct file_operations proc_demux_qops = {
	.open           = demux_qopen,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = seq_release,
};

static int demux_show_irqs(struct seq_file *p, void *v)
{
	int i = *(loff_t *)v, j;
	unsigned long flags;
	
	local_irq_save(flags);
	j = irq_stats[i];
	local_irq_restore(flags);
	
	seq_printf(p, "%2d: %d\n", i, j);
	
	return 0;
}

static void *demux_seq_istart(struct seq_file *f, loff_t *pos)
{
	return (*pos < STBx25xx_DEMUX1_IRQ_COUNT) ? pos : NULL;
}

static void *demux_seq_inext(struct seq_file *f, void *v, loff_t *pos)
{
	(*pos)++;
	if (*pos >= STBx25xx_DEMUX1_IRQ_COUNT)
		return NULL;
	return pos;
}

static void demux_seq_istop(struct seq_file *f, void *v)
{
	/* Nothing to do */
}

static const struct seq_operations demux_seq_iops = {
	.start = demux_seq_istart,
	.next  = demux_seq_inext,
	.stop  = demux_seq_istop,
	.show  = demux_show_irqs
};

static int demux_iopen(struct inode *inode, struct file *file)
{
	return seq_open(file, &demux_seq_iops);
}

static const struct file_operations proc_demux_iops = {
	.open           = demux_iopen,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = seq_release,
};

static void demux_init_procfs(void)
{
	demux_proc_irqs = proc_create(DEMUX_PROC_IRQS_NAME, 0, stbx25xx_proc_dir, &proc_demux_iops);
	demux_proc_queues = proc_create(DEMUX_PROC_QUEUES_NAME, 0, stbx25xx_proc_dir, &proc_demux_qops);
}

static void demux_deinit_procfs(void)
{
	remove_proc_entry(DEMUX_PROC_QUEUES_NAME, stbx25xx_proc_dir);
	remove_proc_entry(DEMUX_PROC_IRQS_NAME, stbx25xx_proc_dir);
}

/*
-------------------------------------------------------------------------------
 */

/**
	Module-related routines
**/


/* stbx25xx_demux_init */
int stbx25xx_demux_init(struct stbx25xx_dvb_data *dvb)
{
	struct stbx25xx_demux_data *dmx = &dvb->demux;
	int ret;
	
	printk(KERN_INFO "--- STBx25xx MPEG-2 Transport Demultiplexer driver ---\n");
		
	/* Init the filter block allocator */
	demux_init_section_filters();
	
	if((ret = demux_reset()) != 0)
		goto error_reset;
	
	if((queues_base = ioremap_nocache(DEMUX_QUEUES_BASE, DEMUX_QUEUES_SIZE)) == NULL) {
		err("Failed to map demux memory");
		ret = -ENOMEM;
		goto error_ioremap;
	}
	
	info("Mapped %d bytes of demux memory at 0x%p", DEMUX_QUEUES_SIZE, queues_base);

	/* Init interrupts */
	if((ret = demux_init_irq(dmx)) != 0)
		goto error_irq;
	
	/* Init queues and filters */
	if((ret = demux_init_queues(dmx)) != 0) {
		err("Failed to initialize transport queues");
		goto error_queues;
	}

	demux_clk_mgmt_start(dmx);
	
	demux_enable_sync();
	
	demux_init_procfs();
	
	return 0;
	
error_queues:
	free_irq(dvb->irq_num[STBx25xx_IRQ_DEMUX], dmx);
error_irq:
	iounmap(queues_base);
	queues_base = NULL;
error_ioremap:
error_reset:
	return ret;
}

/* stbx25xx_demux_exit */
void stbx25xx_demux_exit(struct stbx25xx_dvb_data *dvb)
{
	struct stbx25xx_demux_data *dmx = &dvb->demux;
	
	demux_deinit_procfs();
	
	demux_disable_sync();
	
	demux_clk_mgmt_stop(dmx);	
	demux_queues_deinit(dmx);
	
	free_irq(dvb->irq_num[STBx25xx_IRQ_DEMUX], dmx);
	
	if(queues_base)
		iounmap(queues_base);
}

/*
-------------------------------------------------------------------------------
 */
