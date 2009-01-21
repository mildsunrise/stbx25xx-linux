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
	
	reg = get_demux_reg(CONTROL1);
	reg.control1.se = 0;
	set_demux_reg(CONTROL1, reg);
}

/*
-------------------------------------------------------------------------------
 */

/**
	Queue and PID filter management
**/

struct demux_queue {
	u32 pid;
	phys_addr_t phys_addr;
	void *addr;
	size_t size; /* 0 = Queue disabled */
	u16 key;
#define QUEUE_CONFIG_TYPE_MASK	((1 << 4) - 1)
#define QUEUE_CONFIG_PESL	(1 << 4)
#define QUEUE_CONFIG_DE		(1 << 5)
#define QUEUE_CONFIG_SCPC	(1 << 6)
#define QUEUE_CONFIG_APUS	(1 << 7)
#define QUEUE_CONFIG_ACTIVE	(1 << 15)
	u16 config;
	struct semaphore sem;
};

struct queue_handle {
	u32 handle;
};

static struct demux_queue demux_queues[STBx25xx_QUEUE_COUNT];
static struct queue_handle queue_handles[STBx25xx_QUEUE_COUNT];

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
}

/*
 * demux_stop_queue()
 * Stop the chosen queue
 */
static void demux_stop_queue(int queue)
{
	u32 reg;
	
	reg = get_demux_reg_raw(QSTOPS);
	reg |= (1 << 31) >> queue;
	set_demux_reg_raw(QSTOPS, reg);
}

/*
 * demux_reset_queue()
 * Reset the chosen queue
 */
static void demux_reset_queue(int queue)
{
	u32 reg;
	
	reg = get_demux_reg_raw(QRESETS);
	reg |= (1 << 31) >> queue;
	set_demux_reg_raw(QRESETS, reg);
}

/*
 * demux_start_queue()
 * Let the Transport Interface write data to the queue
 */
static void demux_start_queue(int queue)
{
	stbx25xx_demux_val reg;
	
	if(down_killable(&demux_queues[queue].sem)) {
		err("demux_start_queue: Received a fatal signal, exiting...\n");
		return;
	}
	
	if(demux_queues[queue].config & QUEUE_CONFIG_ACTIVE) {
		reg = get_demux_reg(QCFGB(queue));
		reg.qcfgb.enbl	= 1;
		set_demux_reg(QCFGB(queue), reg);
	}
	
	up(&demux_queues[queue].sem);
}

/*
 * demux_config_queue()
 * Configure the queue and related PID filter according to stored settings
 */
static void demux_config_queue(int queue)
{
	stbx25xx_demux_val reg;
	
	if(down_killable(&demux_queues[queue].sem)) {
		err("demux_config_queue: Received a fatal signal, exiting...\n");
		return;
	}
	
	if((demux_queues[queue].config & QUEUE_CONFIG_ACTIVE) == 0) {
		reg.raw		= 0;
		reg.pid.de	= (demux_queues[queue].config & QUEUE_CONFIG_DE) != 0;
		reg.pid.pesl	= (demux_queues[queue].config & QUEUE_CONFIG_PESL) != 0;
		reg.pid.kid	= demux_queues[queue].key & 7;
		reg.pid.pidv	= demux_queues[queue].pid & 0x1fff;
		set_demux_reg(PIDFLT(queue), reg);
		
		reg.raw		= 0;
		reg.qcfga.strta	= demux_queues[queue].phys_addr >> DEMUX_QUEUE_BLOCK_SHFT;
		reg.qcfga.stopa = (demux_queues[queue].phys_addr + demux_queues[queue].size) >> DEMUX_QUEUE_BLOCK_SHFT;
		set_demux_reg(QCFGA(queue), reg);
		
		reg.raw		= 0;
		reg.qcfgb.scpc	= (demux_queues[queue].config & QUEUE_CONFIG_SCPC) != 0;
		reg.qcfgb.apus	= (demux_queues[queue].config & QUEUE_CONFIG_APUS) != 0;
		reg.qcfgb.dt	= demux_queues[queue].config & QUEUE_CONFIG_TYPE_MASK;
		set_demux_reg(QCFGB(queue), reg);
		
		set_demux_reg_raw(QSTATA(queue), 0xFFFF);
		
		if(queue > 23 && queue < 28)
			set_demux_reg_raw(TSHM(queue), 0);
		
		demux_queues[queue].config |= QUEUE_CONFIG_ACTIVE;
	}
		
	up(&demux_queues[queue].sem);
}

/*
 * demux_remove_queue()
 * Disable the queue and related filter, flush all data and free the memory
 */
static void demux_remove_queue(int queue)
{
	if(down_killable(&demux_queues[queue].sem)) {
		err("demux_disable_queue: Received a fatal signal, exiting...\n");
		return;
	}
	
	if(demux_queues[queue].size) {
		demux_stop_queue(queue);
		// TODO: Submit rest of data upstream before deallocation
		reset_queue_config(queue);
		queues_pool_free(demux_queues[queue].addr, demux_queues[queue].size);
		demux_queues[queue].size = 0;
	}
	
	up(&demux_queues[queue].sem);
}

/*
 * demux_remove_queue_pid()
 * Disable the queue connected to the given PID
 */
static void demux_remove_queue_pid(u32 pid)
{
	int queue;
	
	for(queue=0; queue<STBx25xx_QUEUE_COUNT; queue++) {
		if(down_killable(&demux_queues[queue].sem)) {
			err("demux_remove_queue: Received a fatal signal, exiting...\n");
			return;
		}
		
		if(demux_queues[queue].pid == pid) {
			up(&demux_queues[queue].sem);
			demux_remove_queue(queue);
			break;
		} else {
			up(&demux_queues[queue].sem);
		}
	}
}

static int demux_install_queue(int queue, u32 pid, u32 blocks, u16 config, u16 key)
{
	int ret;
	
	if(down_killable(&demux_queues[queue].sem)) {
			err("demux_install_queue: Received a fatal signal, exiting...\n");
			return -EAGAIN;
	}
		
	if(demux_queues[queue].size) {
		ret = -EBUSY;
		goto exit;
	}
	
	if(queues_pool_freemem() < blocks * DEMUX_QUEUE_BLOCK_SIZE) {
		ret = -ENOMEM;
		goto exit;
	}
	
	demux_queues[queue].addr = queues_pool_alloc(blocks * DEMUX_QUEUE_BLOCK_SIZE, &demux_queues[queue].phys_addr);
	if(demux_queues[queue].addr == NULL) {
		ret = -ENOMEM;
		goto exit;
	}
	
	demux_queues[queue].size	= blocks * DEMUX_QUEUE_BLOCK_SIZE;
	demux_queues[queue].config	= config & ~QUEUE_CONFIG_ACTIVE;
	demux_queues[queue].pid		= pid;
	demux_queues[queue].key		= key;
	
	up(&demux_queues[queue].sem);
	
	demux_config_queue(queue);
	demux_reset_queue(queue);
	demux_start_queue(queue);
	
	return 0;
	
exit:
	up(&demux_queues[queue].sem);
	
	return ret;
}

static int demux_install_system_queue(int queue, u32 pid, u32 blocks, u16 config, u16 key)
{
	int ret = 0;
	
	if(queue < 30 || queue > 31)
		return -EINVAL;
	
	if(pid & ~0x1FFF)
		return -EINVAL;
	
	if(key > 7)
		return -EINVAL;
	
	if(down_killable(&demux_queues[queue].sem)) {
			err("demux_install_system_queue: Received a fatal signal, exiting...\n");
			return -EAGAIN;
	}
	
	if(!demux_queues[queue].size)
		ret = demux_install_queue(queue, pid, blocks, config, key);
	else
		err("demux_install_system_queue: Tried to configure an already configured system queue nr %d", queue);
	
	up(&demux_queues[queue].sem);
	
	return ret;
}

/*
 * demux_install_queue()
 * Install queue receiving data from selected PID
 */
static int demux_install_user_queue(u32 pid, u32 blocks, u16 config, u16 key)
{
	int queue, ret;
	
	if(pid & ~0x1FFF)
		return -EINVAL;
	
	if(key > 7)
		return -EINVAL;
	
	for(queue=0; queue<STBx25xx_QUEUE_COUNT; queue++) {
		if(down_killable(&demux_queues[queue].sem)) {
			err("demux_install_user_queue: Received a fatal signal, exiting...\n");
			return -EAGAIN;
		}
		
		if(demux_queues[queue].size)
			up(&demux_queues[queue].sem);
		else
			break;
	}
	
	if(queue >= STBx25xx_QUEUE_COUNT)
		return -EBUSY;
	
	up(&demux_queues[queue].sem);
	
	if((ret = demux_install_queue(queue, pid, blocks, config, key)) != 0)
		return ret;
	
	return queue;
}

/*
 * demux_audio_channel_change()
 * Change audio channel (PID)
 */
static void demux_audio_channel_change(u16 pid)
{
	stbx25xx_demux_val reg;
	
	if(down_killable(&demux_queues[DEMUX_AUDIO_QUEUE].sem)) {
			err("demux_audio_channel_change: Received a fatal signal, exiting...\n");
			return;
	}
	
	demux_queues[DEMUX_AUDIO_QUEUE].pid = pid;
	
	up(&demux_queues[DEMUX_AUDIO_QUEUE].sem);
	
	reg = get_demux_reg(PIDFLT(DEMUX_AUDIO_QUEUE));
	reg.avcchng.pidv = pid;
	set_demux_reg(ACCHNG, reg);
}

/*
 * demux_video_channel_change()
 * Change video channel (PID)
 */
static void demux_video_channel_change(u16 pid)
{
	stbx25xx_demux_val reg;
	
	if(down_killable(&demux_queues[DEMUX_VIDEO_QUEUE].sem)) {
			err("demux_audio_channel_change: Received a fatal signal, exiting...\n");
			return;
	}
	
	demux_queues[DEMUX_VIDEO_QUEUE].pid = pid;
	
	up(&demux_queues[DEMUX_VIDEO_QUEUE].sem);
	
	reg = get_demux_reg(PIDFLT(DEMUX_VIDEO_QUEUE));
	reg.avcchng.pidv = pid;
	set_demux_reg(VCCHNG, reg);
}

/* demux_queues_init */
static int demux_queues_init(void)
{
	int i, ret;
	
	memset(demux_queues, 0, sizeof(struct demux_queue) * STBx25xx_QUEUE_COUNT);
	
	for(i=0; i<STBx25xx_QUEUE_COUNT; i++) {
		sema_init(&demux_queues[i].sem, 1);
		queue_handles[i].handle = i;
	}
	
	if((ret = queues_pool_init(DEMUX_QUEUES_BASE, queues_base, DEMUX_QUEUES_SIZE)) != 0)
		goto error_pool;
	
	demux_set_queues_base_ptr(DEMUX_QUEUES_BASE);
	
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
	
	for(i=0; i<STBx25xx_QUEUE_COUNT; i++)
		demux_remove_queue(i);
	
	demux_set_queues_base_ptr(0);
	
	queues_pool_deinit();
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

/* demux_clk_mgmt_init() */
static void demux_clk_mgmt_init(void)
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
static u32 demux_queues_irq_mask;
static u32 demux_festat_irq_mask;
static u32 demux_audio_irq_mask;
static u32 demux_video_irq_mask;
static u32 demux_descr_irq_mask;

typedef void (*demux_irq_handler_t)(struct stbx25xx_dvb_dev *dvb);
static demux_irq_handler_t irq_handler_table[STBx25xx_DEMUX1_IRQ_COUNT];

static irqreturn_t demux_irq_handler(int irq, void *dev_id)
{
	int i;
	struct stbx25xx_dvb_dev *dvb = dev_id;
	u32 first_stat = mfdcr(DEMUX_INT) & demux_irq_mask;	/* AND with mask to be safe... */
	mtdcr(DEMUX_INT, first_stat);				/* Clear instantly to minimize interrupt loss */
	
	for(i = 0; i < STBx25xx_DEMUX1_IRQ_COUNT && first_stat; i++, first_stat <<= 1) {
		if(first_stat & (1 << 31) && irq_handler_table[i])
			irq_handler_table[i](dvb);
	}
	
	return IRQ_HANDLED;
}

static int demux_irq_init(struct stbx25xx_dvb_dev *dvb)
{
	memset(irq_handler_table, 0, sizeof(demux_irq_handler_t) * STBx25xx_DEMUX1_IRQ_COUNT);
	
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
	
	if(feed->ts_type & TS_DECODER) {
		if(feed->pes_type == DMX_TS_PES_AUDIO) {
			demux_audio_channel_change(feed->pid);
			return 0;
		}
		
		if(feed->pes_type == DMX_TS_PES_VIDEO) {
			demux_video_channel_change(feed->pid);
			return 0;
		}
		
		if(feed->pes_type == DMX_TS_PES_PCR) {
			demux_set_pcr_pid(feed->pid);
			return 0;
		}
			
		
		return -EINVAL;
	}
	
	config = 0;
	if(feed->ts_type & TS_PAYLOAD_ONLY) {
		config = QCFG_DT_PAYLOAD | QUEUE_CONFIG_APUS;
	}
	
	if((queue = demux_install_user_queue(feed->pid, 16, config, 0)) < 0)
		return queue;
	
	feed->priv = &queue_handles[queue];
	
	return 0;
}

/* stbx25xx_demux_stop_feed */
int stbx25xx_demux_stop_feed(struct dvb_demux_feed *feed)
{
	if(feed->ts_type & TS_DECODER) {
		if(feed->pes_type == DMX_TS_PES_AUDIO) {
			demux_audio_channel_change(0x1FFF);
			return 0;
		}
		
		if(feed->pes_type == DMX_TS_PES_VIDEO) {
			demux_video_channel_change(0x1FFF);
			return 0;
		}
		
		if(feed->pes_type == DMX_TS_PES_PCR) {
			demux_set_pcr_pid(0x1FFF);
			return 0;
		}
		
		return -EINVAL;
	}
	
	if(feed->priv)
		demux_remove_queue(((struct queue_handle*)(feed->priv))->handle);
	
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
	
	if((ret = demux_queues_init()) != 0)
		goto error_queues;

	demux_clk_mgmt_init();
	
	demux_enable_sync();
	
	return 0;
	
error_queues:
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
	demux_disable_sync();
	
	demux_clk_mgmt_deinit();	
	demux_queues_deinit();
	
	free_irq(DEMUX_IRQ, dvb);
	
	if(queues_base)
		iounmap(queues_base);
}

/*
-------------------------------------------------------------------------------
 */
