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

#include <linux/spinlock.h>
#include <asm/dcr.h>
#include "stbx25xx_common.h"
#include "stbx25xx_demux.h"

/* Static data */

static spinlock_t demux_dcr_lock;

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

static stbx25xx_demux_val pbuflvl_dev = {
	.pbuflvl = {
		.qpt	= 8,
		.apt	= 4,
		.vpt	= 8,
	},
};
static stbx25xx_demux_val pbuflvl_cur;

/* Utility functions */

static inline void set_demux_reg(u16 reg, stbx25xx_demux_val val)
{
	unsigned long flags;
	
	spin_lock_irqsave(&demux_dcr_lock, flags);
	
	mtdcr(DEMUX_ADDR, reg);
	mtdcr(DEMUX_DATA, val.raw);
	
	spin_unlock_irqrestore(&demux_dcr_lock, flags);
}

static inline void set_demux_reg_raw(u16 reg, u32 val)
{
	unsigned long flags;
	
	spin_lock_irqsave(&demux_dcr_lock, flags);
	
	mtdcr(DEMUX_ADDR, reg);
	mtdcr(DEMUX_DATA, val);
	
	spin_unlock_irqrestore(&demux_dcr_lock, flags);
}

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

/* Private functions */
static int demux_reset(void)
{
	stbx25xx_demux_val reg;
	int i;
	
	printk(KERN_INFO STBx25xx_LOG_PREFIX ": resetting demux...");
		
	reg = get_demux_reg(CONTROL1);
	reg.control1.se = 0;
	reg.control1.swr = 1;
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
	return 0;
}

/* Exported functions */

u32 stbx25xx_demux_check_crc32(struct dvb_demux_feed *feed,
			    const u8 *buf, size_t len)
{
	return 0;
}

void stbx25xx_demux_memcopy(struct dvb_demux_feed *feed, u8 *dst,
			 const u8 *src, size_t len)
{
	
}

int stbx25xx_demux_connect_frontend(struct dmx_demux* demux,
				 struct dmx_frontend* frontend)
{
	return 0;
}

int stbx25xx_demux_disconnect_frontend(struct dmx_demux* demux)
{
	return 0;
}

int stbx25xx_demux_get_stc(struct dmx_demux* demux, unsigned int num,
			u64 *stc, unsigned int *base)
{
	return 0;
}

int stbx25xx_demux_start_feed(struct dvb_demux_feed *feed)
{
	return 0;
}

int stbx25xx_demux_stop_feed(struct dvb_demux_feed *feed)
{
	return 0;
}

int stbx25xx_demux_write_to_decoder(struct dvb_demux_feed *feed,
				 const u8 *buf, size_t len)
{
	return 0;
}

int stbx25xx_demux_init(struct stbx25xx_dvb_dev *dvb)
{
	int ret;
	
	spin_lock_init(&demux_dcr_lock);
	if((ret = demux_reset()) != 0)
		goto error;
	
	return 0;

error:
	return ret;
}

void stbx25xx_demux_exit(struct stbx25xx_dvb_dev *dvb)
{
	
}
