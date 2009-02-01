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

#define DBG_LEVEL 1

#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/semaphore.h>
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

static struct mutex queues_mutex;
static struct list_head queues_list;
static struct demux_queue demux_queues[STBx25xx_QUEUE_COUNT];
static u32 demux_queues_irq_mask = 0;
static u32 demux_qstat_irq_mask = 0;

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
	mask &= 0xFFFF;
	demux_qstat_irq_mask |= mask;
	set_demux_reg_raw(QSTMSK, demux_qstat_irq_mask);
}

/*
 * demux_mask_qstat_irq()
 * Disable causing interrupts by given status bits
 */
static void demux_mask_qstat_irq(u32 mask)
{
	mask &= 0xFFFF;
	demux_qstat_irq_mask &= ~mask;
	set_demux_reg_raw(QSTMSK, demux_qstat_irq_mask);
}

/*
 * demux_enable_queue_irq()
 * Enable queue interrupt
 */
static void demux_enable_queue_irq(int queue)
{
	if(queue > 31)
		return;
	
	demux_queues_irq_mask |= (1 << 31) >> queue;
	set_demux_reg_raw(QINTMSK, demux_queues_irq_mask);
}

/*
 * demux_disable_queue_irq()
 * Disable queue interrupt
 */
static void demux_disable_queue_irq(int queue)
{
	if(queue > 31)
		return;
	
	demux_queues_irq_mask &= ~((1 << 31) >> queue);
	set_demux_reg_raw(QINTMSK, demux_queues_irq_mask);
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
	
	reg = (1 << 31) >> queue;
	set_demux_reg_raw(QSTOPS, reg);
}

/*
 * demux_reset_queue()
 * Reset the chosen queue
 */
static void demux_reset_queue(int queue)
{
	u32 reg;
	
	reg = (1 << 31) >> queue;
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
static void demux_config_queue(int queue)
{
	stbx25xx_demux_val reg;
	struct filter_block *block;
	
	if(!(demux_queues[queue].config & QUEUE_CONFIG_ACTIVE)) {
		reg.raw		= 0;
		reg.qcfga.strta	= (demux_queues[queue].phys_addr & 0xFFFFFF) >> DEMUX_QUEUE_BLOCK_SHFT;
		reg.qcfga.stopa = ((demux_queues[queue].phys_addr + demux_queues[queue].size) & 0xFFFFFF) >> DEMUX_QUEUE_BLOCK_SHFT;
		set_demux_reg(QCFGA(queue), reg);
		dprintk("%s: Set QCFGA(%d) to 0x%08x\n", __func__, queue, reg.raw);
		
		if(queue > 23 && queue < 28)
			set_demux_reg_raw(TSHM(queue), 0);
		
//		set_demux_reg_raw(QSTATB(queue), demux_queues[queue].phys_addr & 0xFFFFFF);
		
		reg.raw		= 0;
		reg.qcfgb.rp	= (demux_queues[queue].size) >> 8;
		reg.qcfgb.apus	= (demux_queues[queue].config & QUEUE_CONFIG_APUS) != 0;
		reg.qcfgb.dt	= demux_queues[queue].config & QUEUE_CONFIG_TYPE_MASK;
		reg.qcfgb.enbl	= (demux_queues[queue].config & QUEUE_CONFIG_EN) != 0;
		reg.qcfgb.scpc	= 1;
		
		if(demux_queues[queue].config & QUEUE_CONFIG_SECFLT) {
			block = list_first_entry(&demux_queues[queue].filters, struct filter_block, list);
			reg.qcfgb.fsf = block->index;
		}
		
		set_demux_reg(QCFGB(queue), reg);
		dprintk("%s: Set QCFGB(%d) to 0x%08x\n", __func__, queue, reg.raw);
		
		set_demux_reg_raw(QSTATA(queue), 0xFFFF);
		
		demux_queues[queue].config |= QUEUE_CONFIG_ACTIVE;
		
		demux_enable_queue_irq(queue);
		
		reg.raw		= 0;
		reg.pid.de	= (demux_queues[queue].config & QUEUE_CONFIG_DE) != 0;
		reg.pid.pesl	= (demux_queues[queue].config & QUEUE_CONFIG_PESL) != 0;
		reg.pid.kid	= demux_queues[queue].key & 7;
		reg.pid.pidv	= demux_queues[queue].pid & 0x1fff;
		set_demux_reg(PIDFLT(queue), reg);
		dprintk("%s: Set PIDFLT(%d) to 0x%08x\n", __func__, queue, reg.raw);
	}
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
	
	dprintk("%s: Audio channel PID = %d\n", __func__, pid);
	
	spin_lock_irqsave(&queue->lock, flags);
	
	queue->pid = pid;
	
	reg = get_demux_reg(PIDFLT(DEMUX_AUDIO_QUEUE));
	reg.avcchng.pidv = pid;
	set_demux_reg(ACCHNG, reg);
	
	spin_unlock_irqrestore(&queue->lock, flags);
	
	return 0;
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
	
	dprintk("%s: Video channel PID = %d\n", __func__, pid);
	
	spin_lock_irqsave(&queue->lock, flags);
	
	queue->pid = pid;
		
	reg = get_demux_reg(PIDFLT(DEMUX_VIDEO_QUEUE));
	reg.avcchng.pidv = pid;
	set_demux_reg(VCCHNG, reg);
	
	spin_unlock_irqrestore(&queue->lock, flags);
	
	return 0;
}

/* demux_process_queue_irq */
static void demux_process_queue_irq(int queue, u32 stat)
{
	if(stat & QUEUE_RPI)
		dprintk("%s: Queue %d Read Pointer Interrupt\n", __func__, queue);
	
	if(stat & QUEUE_PCSC) {
		tasklet_hi_schedule(&demux_queues[queue].tasklet);
	}
	
	set_demux_reg_raw(QSTATA(queue), stat);
}

/*
-------------------------------------------------------------------------------
 */

/**
	Software part of queues code
*/

#define DEBUG_LISTS

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

/*
 * demux_submit_data()
 * Submit data to apropriate callback function
 */

static void demux_section_callback(struct demux_queue *queue, void *buf, size_t size, void *buf2, size_t size2)
{
	if(!size2)
		buf2 = NULL;
	
	queue->feed->cb.sec(buf, size, buf2, size2, &queue->feed->filter->filter, DMX_OK);
}

static void demux_ts_pes_callback(struct demux_queue *queue, void *buf, size_t size, void *buf2, size_t size2)
{
	if(!size2)
		buf2 = NULL;
	
	queue->feed->cb.ts(buf, size, buf2, size2, &queue->feed->feed.ts, DMX_OK);	
}

static void demux_swdemux_callback(struct demux_queue *queue, void *buf, size_t size, void *buf2, size_t size2)
{
	u32 packets;
	
	if(!size2) {
		dvb_dmx_swfilter_packets(queue->demux, buf, size / 188);
	} else {
		packets = size / 188;
		if(packets)
			dvb_dmx_swfilter_packets(queue->demux, buf, packets);
		
		size -= packets * 188;
		buf += packets * 188;
		
		dvb_dmx_swfilter(queue->demux, buf, size);
		dvb_dmx_swfilter(queue->demux, buf2, 188 - size);
		
		size2 -= 188 - size;
		buf2 += 188 - size;
		
		packets = size2 / 188;
		if(packets)
			dvb_dmx_swfilter_packets(queue->demux, buf2, packets);
	}
}

/*
 * demux_unloading_task()
 * Task unloading data from a single queue (so up to 32 tasks)
 * NOTE: Running as a tasklet
 */
static void demux_unloading_task(unsigned long data)
{
	struct demux_queue * queue = (struct demux_queue *)data;
	int error = 0;
	phys_addr_t write_ptr, read_ptr;
	size_t data_avl, data_avl2;

//	dprintk("%s: Queue %d unloader task started\n", __func__, queue->handle);

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
		dprintk("%s: Queue %d fatal error: read_ptr = 0x%08x, write_ptr = 0x%08x\n",
				__func__, queue->handle, read_ptr, write_ptr);
		return;
	}

//	dprintk("%s: Queue %d: read_ptr = 0x%08x, write_ptr = 0x%08x\n", __func__, queue->handle, read_ptr, write_ptr);

	if(read_ptr <= write_ptr) {
		data_avl = write_ptr - read_ptr;
		data_avl2 = 0;
	} else {
		data_avl2 = (write_ptr - queue->phys_addr);
		data_avl = (queue->phys_addr + queue->size - read_ptr);
	}

	/* Check if there is data ready in the queue */
	if(!data_avl) {
		return;
	} else {
		/* Data available, so get it */

//		dprintk("%s: Unloading %d bytes of queue %d data from %p\n",
//			__func__, data_avl, queue->handle, queue->ptr);

		if(queue->cb)
			queue->cb(queue, queue->ptr, data_avl, queue->addr, data_avl2);
		
		queue->ptr += data_avl + data_avl2;
		queue->data_count += data_avl + data_avl2;

		if(queue->ptr >= queue->addr + queue->size)
			queue->ptr -= queue->size;

		read_ptr = queue_to_phys(queue, queue->ptr);			
		demux_set_read_ptr(queue, read_ptr);
	}
	
//	dprintk("%s: Queue %d unloader task finished (transered %d bytes of data)\n", __func__, queue->handle, data_count);
}

/*
 * demux_remove_queue()
 * Disable the queue and related filter, flush all data and free the memory
 */
static int demux_remove_queue(int queue)
{
	BUG_ON(!demux_queues[queue].size);
	
	if(demux_queues[queue].size) {
		dprintk("%s: Removing queue %d\n", __func__, queue);
		
		/* Stop the hardware */
		demux_stop_queue(queue);
		demux_disable_queue_irq(queue);
		
		dprintk("%s: Waiting for queue %d unloader to terminate...\n", __func__, queue);
		
		tasklet_disable(&demux_queues[queue].tasklet);
		reset_queue_config(queue);		
		queues_pool_free(demux_queues[queue].addr, demux_queues[queue].size);
		
		dprintk("%s: Queue %d removed (transfered %d bytes of data)\n",
			 __func__, queue, demux_queues[queue].data_count);
		
		demux_free_queue(&demux_queues[queue]);
	}
	
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
	
	if(!demux_queues[queue].feed && queue < 30)
		ret = demux_remove_queue(queue);
	
	mutex_unlock(&queues_mutex);
	
	return ret;
}

/*
 * demux_install_queue()
 * Utility function to setup a queue
 */
static int demux_install_queue(int queue, u32 pid, u32 blocks, u16 config, u16 key)
{
	int ret = 0;
	
	dprintk("%s: Installing queue %d, pid = %d, size = %d blocks, config = 0x%04x, key = %d\n",
		 __func__, queue, pid, blocks, config, key);
		
	if(demux_queues[queue].size) {
		dprintk("%s: Queue %d already installed\n", __func__, queue);
		ret = -EBUSY;
		goto exit;
	}
	
	if(queues_pool_freemem() < blocks * DEMUX_QUEUE_BLOCK_SIZE) {
		dprintk("%s: Not enough memory for queue\n", __func__);
		ret = -ENOMEM;
		goto exit;
	}
	
	demux_queues[queue].addr = queues_pool_alloc(blocks * DEMUX_QUEUE_BLOCK_SIZE, &demux_queues[queue].phys_addr);
	if(demux_queues[queue].addr == NULL) {
		dprintk("%s: Couldn't allocate memory for queue\n", __func__);
		ret = -ENOMEM;
		goto exit;
	}
	dprintk("%s: Allocated queue %d to 0x%08x mapped at %p\n", __func__, queue, demux_queues[queue].phys_addr, demux_queues[queue].addr);
	
	tasklet_init(&demux_queues[queue].tasklet, demux_unloading_task, (unsigned long)&demux_queues[queue]);
	
	demux_queues[queue].size	= blocks * DEMUX_QUEUE_BLOCK_SIZE;
	demux_queues[queue].config	= config & ~QUEUE_CONFIG_ACTIVE;
	demux_queues[queue].pid		= pid;
	demux_queues[queue].key		= key;
	demux_queues[queue].ptr		= demux_queues[queue].addr;
	demux_queues[queue].data_count	= 0;
	
	if(config & QUEUE_CONFIG_SWDEMUX)
		demux_queues[queue].cb = demux_swdemux_callback;
	else if(config & QUEUE_CONFIG_SECFLT)
		demux_queues[queue].cb = demux_section_callback;
	else
		demux_queues[queue].cb = demux_ts_pes_callback;
	
	demux_stop_queue(queue);
	demux_reset_queue(queue);
	demux_config_queue(queue);
	
	dprintk("%s: Queue installed successfully\n", __func__);
	
exit:
	return ret;
}

/*
 * demux_install_system_queue()
 * Install system filter - for video or audio PIDs
 */
static int demux_install_system_queue(int queue, u32 pid, u32 blocks, u16 config, u16 key)
{
	int ret = 0;
	
	if(queue < 30 || queue > 31)
		return -EINVAL;
	
	if(pid & ~0x1FFF)
		return -EINVAL;
	
	if(key > 7)
		return -EINVAL;
	
	if(mutex_lock_killable(&queues_mutex)) {
			err("demux_install_system_queue: Received a fatal signal, exiting...\n");
			return -ERESTARTSYS;
	}
	
	if(!demux_queues[queue].size)
		ret = demux_install_queue(queue, pid, blocks, config, key);
	else
		err("demux_install_system_queue: Tried to configure an already configured system queue nr %d", queue);
	
	demux_queues[queue].config &= ~QUEUE_CONFIG_ACTIVE;
	
	mutex_unlock(&queues_mutex);
	
	return ret;
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
		ret = demux_install_queue(queue->handle, feed->pid, blocks, config | QUEUE_CONFIG_EN, 0);
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

/* demux_init_queues */
static int demux_init_queues(struct stbx25xx_dvb_dev *dvb)
{
	int i, ret;
		
	memset(demux_queues, 0, sizeof(struct demux_queue) * STBx25xx_QUEUE_COUNT);
	INIT_LIST_HEAD(&queues_list);
	mutex_init(&queues_mutex);
	
	for(i=0; i<STBx25xx_QUEUE_COUNT; i++) {
		spin_lock_init(&demux_queues[i].lock);
		demux_queues[i].handle = i;
		demux_queues[i].demux = &dvb->demux;
		demux_queues[i].pid = 0x1fff;
		demux_queues[i].cb = NULL;
		
		if(i < 30)
			list_add_tail(&demux_queues[i].list, &queues_list);
		
		INIT_LIST_HEAD(&demux_queues[i].filters);
	}
	
	if((ret = queues_pool_init(DEMUX_QUEUES_BASE, queues_base, DEMUX_QUEUES_SIZE)) != 0)
		goto error_pool;
		
	demux_set_queues_base_ptr(DEMUX_QUEUES_BASE);
	demux_unmask_qstat_irq(QUEUE_RPI | QUEUE_PCSC);
	
	demux_install_system_queue(DEMUX_AUDIO_QUEUE, 0x1FFF, 16, 0, 0);
	demux_install_system_queue(DEMUX_VIDEO_QUEUE, 0x1FFF, 16, 0, 0);
	
	return 0;

error_pool:
	return ret;
}

/* demux_queues_deinit */
static void demux_queues_deinit(void)
{
	int i;
	
	demux_mask_qstat_irq(0xFFFFFFFF);
	
	for(i=0; i<STBx25xx_QUEUE_COUNT; i++) {		
		if(demux_queues[i].size)
			demux_remove_queue(i);
	}
	
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
			err("%s: Received a fatal signal, exiting...\n", __func__);
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
			err("%s: Received a fatal signal, exiting...\n", __func__);
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
		dprintk("%s: Duplicating PID values unallowed.\n", __func__);
		ret = -EINVAL;
		goto exit;
	}
	
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
		
		ret = demux_install_user_queue(feed, 16, QCFG_DT_TSPKT | QUEUE_CONFIG_EN | QUEUE_CONFIG_SWDEMUX);
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
			
	block->positive = ((~feed->filter->maskandmode[0]) << 24) |
			((~feed->filter->maskandmode[3]) << 16) | 
			((~feed->filter->maskandmode[4]) << 8) | 
			(~feed->filter->maskandmode[5]);
			
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
				
		block->positive = ((~feed->filter->maskandmode[i]) << 24) |
				((~feed->filter->maskandmode[i+1]) << 16) | 
				((~feed->filter->maskandmode[i+2]) << 8) | 
				(~feed->filter->maskandmode[i+3]);
				
		block->sfid = feed->filter->index;
		i += 4;
	}
	
	feed->priv = NULL;
	queue->feed = feed;
	
	demux_config_section_filters(queue);
	ret = demux_install_queue(queue->handle, feed->pid, 16, 
				   QCFG_DT_TBSEC_FLT | QUEUE_CONFIG_EN | QUEUE_CONFIG_SECFLT, 0);
	
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

/* demux_init_clk_mgmt() */
static void demux_init_clk_mgmt(void)
{
	
}

/* demux_clk_mgmt_deinit() */
static void demux_clk_mgmt_deinit(void)
{
	
}

/*
-------------------------------------------------------------------------------
 */

/**
	Transport DMA (clip mode) support
**/

static int clipmode = 0;

static int demux_enable_clip_mode(void)
{
	demux_change_input(DEMUX_IN_TSDMA);
	return 0;
}

static int demux_disable_clip_mode(void)
{
	demux_change_input(DEMUX_IN_CI0);
	return 0;
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

typedef void (*demux_irq_handler_t)(struct stbx25xx_dvb_dev *dvb);
static demux_irq_handler_t irq_handler_table[STBx25xx_DEMUX1_IRQ_COUNT];

/*
 * NOTE: IRQ HANDLER
 * demux_irq_handler()
 *
 * Main IRQ handler calling all 2nd level handlers if needed
 */
static irqreturn_t demux_irq_handler(int irq, void *dev_id)
{
	int i;
	struct stbx25xx_dvb_dev *dvb = dev_id;
	u32 first_stat = mfdcr(DEMUX_INT) & demux_irq_mask;	/* AND with the mask to be safe... */
	mtdcr(DEMUX_INT, first_stat);				/* Clear instantly to minimize interrupt loss */
	
	for(i = 0; (i < STBx25xx_DEMUX1_IRQ_COUNT) && first_stat; i++, first_stat <<= 1) {
		if((first_stat & (1 << 31)) && irq_handler_table[i])
			irq_handler_table[i](dvb);
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
static void demux_frontend_irq_handler(struct stbx25xx_dvb_dev *dvb)
{
	stbx25xx_demux_val reg;
	
	reg = get_demux_reg(FESTAT);
	reg.raw &= demux_festat_irq_mask;
	set_demux_reg(FESTAT, reg);
	
	dprintk("%s: IRQ = 0x%08x\n", __func__, reg.raw);
}

/*
 * NOTE: IRQ HANDLER
 * demux_plb_err_handler()
 *
 * Handles memory errors (hopefully not going to happen)
 */
static void demux_plb_err_handler(struct stbx25xx_dvb_dev *dvb)
{
	warn("Demux PLB Error!");
}

/*
 * NOTE: IRQ HANDLER
 * demux_queues_irq_handler()
 *
 * Handles interrupts of all queues
 */
static void demux_queues_irq_handler(struct stbx25xx_dvb_dev *dvb)
{
	int i;
	u32 reg;
	u32 mask = get_demux_reg_raw(QINT) & demux_queues_irq_mask;
	set_demux_reg_raw(QINT, mask);
	
	for(i=STBx25xx_QUEUE_COUNT-1; i>=0 && mask; i--, mask>>=1) {
		if(mask & 1) {
			reg = get_demux_reg_raw(QSTATA(i)) & demux_qstat_irq_mask;
			demux_process_queue_irq(i, reg);
		}
	}
}

static void demux_enable_irq(int demux_irq)
{
	unsigned long flags;
	
	if(demux_irq >= STBx25xx_DEMUX1_IRQ_COUNT || demux_irq < 0)
		return;

	spin_lock_irqsave(&demux_irq_lock, flags);
	
	if(!irq_handler_table[demux_irq]) {
		err("demux_enable_irq: Tried to enable a demux IRQ %d, which does not have a handler", demux_irq);
		goto exit;
	}
	
	demux_irq_mask |= (1 << 31) >> demux_irq;
	set_demux_reg_raw(INTMASK, demux_irq_mask);
	
exit:
	spin_unlock_irqrestore(&demux_irq_lock, flags);
}

static void demux_disable_irq(int demux_irq)
{
	unsigned long flags;
	
	if(demux_irq >= STBx25xx_DEMUX1_IRQ_COUNT || demux_irq < 0)
		return;

	spin_lock_irqsave(&demux_irq_lock, flags);
		
	demux_irq_mask &= ~((1 << 31) >> demux_irq);
	set_demux_reg_raw(INTMASK, demux_irq_mask);
	
	spin_unlock_irqrestore(&demux_irq_lock, flags);
}

static void demux_set_handler(int demux_irq, demux_irq_handler_t demux_handler)
{
	if(demux_irq >= STBx25xx_DEMUX1_IRQ_COUNT || demux_irq < 0)
		return;
	
	if(irq_handler_table[demux_irq] && demux_handler)
		warn("demux_set_handler: Replacing old handler of demux IRQ %d", demux_irq);
	
	irq_handler_table[demux_irq] = demux_handler;
}

static int demux_init_irq(struct stbx25xx_dvb_dev *dvb)
{
	int ret = 0;
	
	memset(irq_handler_table, 0, sizeof(demux_irq_handler_t) * STBx25xx_DEMUX1_IRQ_COUNT);
	
	spin_lock_init(&demux_irq_lock);
	
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
	
	if((ret = request_irq(dvb->irq_num[STBx25xx_IRQ_DEMUX], demux_irq_handler, IRQF_SHARED | IRQF_TRIGGER_HIGH, "demux", dvb)) != 0) {
		err("Failed to request demux irq: error %d", ret);
		return ret;
	}
	
	demux_set_handler(DEMUX_IRQ_FRONTEND, demux_frontend_irq_handler);
	demux_enable_irq(DEMUX_IRQ_FRONTEND);
	demux_festat_irq_mask |= 3;
	set_demux_reg_raw(FEIMASK, demux_festat_irq_mask);
	
	demux_set_handler(DEMUX_IRQ_QUEUES, demux_queues_irq_handler);
	demux_enable_irq(DEMUX_IRQ_QUEUES);
	
	demux_set_handler(DEMUX_IRQ_PLB_ERR, demux_plb_err_handler);
	demux_enable_irq(DEMUX_IRQ_PLB_ERR);
	
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

/*
 * demux_is_clipmode_compatbile()
 * Is this feed compatible with clip mode?
 */
static int demux_is_clipmode_compatbile(struct dvb_demux_feed *dvbdmxfeed)
{
	/* don't put anything but audio and video
	   into the demux output queues */
	if (dvbdmxfeed->pes_type > DMX_TS_PES_VIDEO)
		return 0;
	/* packets being processed by neither the mpeg
	   decoder nor the vbi teletext encoder stay in the
	   software demux layer */
	if (!(dvbdmxfeed->ts_type & TS_DECODER))
		return 0;
	/* ... as well as sections - they don't have a
	   valid ts_type field */
	if (dvbdmxfeed->type == DMX_TYPE_SEC)
		return 0;

	return 1;
}

/* stbx25xx_demux_start_feed */
int stbx25xx_demux_start_feed(struct dvb_demux_feed *feed)
{
	int queue;
	u16 config;
	
	if(feed->type == DMX_TYPE_TS) {
		if(feed->ts_type & TS_DECODER) {
			if(feed->demux->dmx.frontend->source == DMX_MEMORY_FE) {
				dprintk("%s: Memory frontend is not supported yet\n", __func__);
				return -EINVAL;
			} else {
				if(feed->pes_type == DMX_TS_PES_AUDIO)
					return demux_audio_channel_change(feed->pid);
				
				if(feed->pes_type == DMX_TS_PES_VIDEO)
					return demux_video_channel_change(feed->pid);
				
				if(feed->pes_type == DMX_TS_PES_PCR) {
					demux_set_pcr_pid(feed->pid);
					return 0;
				}
			}
			
			dprintk("%s: PES type unsupported by decoder\n", __func__);	
			return -EINVAL;
		}
	
		config = 0;
		if(feed->ts_type & TS_PAYLOAD_ONLY) {
			config = QCFG_DT_PAYLOAD;
		}
		
		if((queue = demux_install_user_queue(feed, 16, config)) < 0)
			return queue;
		
		feed->index = queue;
		
		return 0;
	} else if(feed->type == DMX_TYPE_SEC) {
		if((queue = demux_install_section_filter(feed)) < 0)
			return queue;
		
		feed->index = queue;
		
		return 0;
	} else if(feed->type == DMX_TYPE_PES) {
		dprintk("%s: Unsupported feed type (DMX_TYPE_PES)\n", __func__);
		return -EINVAL;
	}
	
	dprintk("%s: Unknown feed type\n", __func__);
	
	return -EINVAL;
}

/* stbx25xx_demux_stop_feed */
int stbx25xx_demux_stop_feed(struct dvb_demux_feed *feed)
{
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
				return 0;
			}
			
			return -EINVAL;
		} else {
			return demux_remove_user_queue(feed);
		}
	} else if(feed->type == DMX_TYPE_SEC) {
		return demux_remove_section_filter(feed);
	} 
	
	return -EINVAL;
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
	if((ret = demux_init_irq(dvb)) != 0)
		goto error_irq;
	
	/* Init queues and filters */
	if((ret = demux_init_queues(dvb)) != 0) {
		err("Failed to initialize transport queues");
		goto error_queues;
	}

	demux_init_clk_mgmt();
	
	demux_enable_sync();
	
	return 0;
	
error_queues:
	free_irq(dvb->irq_num[STBx25xx_IRQ_DEMUX], dvb);
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
	demux_disable_sync();
	
	demux_clk_mgmt_deinit();	
	demux_queues_deinit();
	
	free_irq(dvb->irq_num[STBx25xx_IRQ_DEMUX], dvb);
	
	if(queues_base)
		iounmap(queues_base);
}

/*
-------------------------------------------------------------------------------
 */
