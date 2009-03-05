/*
 * stbx25xx_video.c - MPEG2 Video Decoder driver
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

#define DEBUG		10
#define DBG_LEVEL	1

#include "stbx25xx.h"
#include "stbx25xx_video.h"
#include <linux/firmware.h>

static stbx25xx_video_val def_display_mode = {
	.disp_mode = {
		/* Nice blue background ;) */
		.bgY	= (40 >> 2),
		.bgCr	= (109 >> 4),
		.bgCb	= (240 >> 4),
		.sm	= 7,
#ifndef CONFIG_STBX25XX_NTSC
		.pal	= 1,
#endif
	},
};

static stbx25xx_video_val def_osd_mode = {
	.osd_mode = {
		.chsfr	= 256,
		.edaf	= 1,
		.afdt	= 1,
		.ilbm	= 1,
		.cds	= 1,
		.clbm	= 1,
		.glbm	= 1,
	},
};

struct stbx25xx_video_cmd {
	u8 cmd;
	u8 cnt;
	u16 *par;
};

/**
	Interrupts
*/
u32 video_int_mask;
typedef void (*video_irq_handler_t)(struct stbx25xx_dvb_dev *dvb, int irq);
static video_irq_handler_t video_int_handlers[STBx25xx_VIDEO_IRQ_COUNT];

static irqreturn_t video_interrupt(int irq, void *data)
{
	struct stbx25xx_dvb_dev *dvb = data;
	u32 mask = get_video_reg_raw(VIDEO_INT) & video_int_mask;
	int i;
	
	for(i = STBx25xx_VIDEO_IRQ_COUNT - 1; i >= 0 && mask; i--, mask >>= 1) {
		if((mask & 1) && (video_int_handlers[i]))
			video_int_handlers[i](dvb, i);
	}
	
	return IRQ_HANDLED;
}

static void video_install_int_handler(int irq, video_irq_handler_t handler)
{
	if(irq >= STBx25xx_VIDEO_IRQ_COUNT)
		return;
	
	video_int_handlers[irq] = handler;
}

static void video_remove_int_handler(int irq)
{
	if(irq >= STBx25xx_VIDEO_IRQ_COUNT)
		return;
	
	video_int_handlers[irq] = NULL;
}


/**
	Hardware manipulation routines
*/
static inline void video_soft_reset(void)
{
	set_video_reg_raw(CMD_STAT, 0);
	set_video_reg_raw(PROC_IADDR, 0x8200);
}

static int video_issue_cmd(struct stbx25xx_video_cmd *cmd)
{
	int i;
	
	i = 0;
	while(get_video_reg_raw(CMD_STAT) & 1) {
		msleep(1);
		i++;
 		if(i > 10) {
			err("video command timeout");
			video_soft_reset();
			return -1;
		}
	}
	
	set_video_reg_raw(CMD, cmd->cmd);
	for(i=0; i<cmd->cnt; i++) {
		set_video_reg_raw(CMD_ADDR, i);
		set_video_reg_raw(CMD_DATA, cmd->par[i]);
	}
	set_video_reg_raw(CMD_STAT, 1);
	
	i = 0;
	while(get_video_reg_raw(CMD_STAT) & 1) {
		msleep(1);
		i++;
		if(i > 10) {
			err("video command timeout");
			video_soft_reset();
			return -1;
		}
	}
	
	return 0;
}

static int video_issue_cmds(struct stbx25xx_video_cmd *cmds, u8 count)
{
	int i, j;
	
	for(j = 0; j < count; j++) {
		i = 0;
		while(get_video_reg_raw(CMD_STAT) & 1) {
			msleep(1);
			i++;
			if(i > 10) {
				err("video command timeout");
				video_soft_reset();
				return -1;
			}
		}
		
		set_video_reg_raw(CMD, cmds[j].cmd);
		for(i = 0; i < cmds[j].cnt; i++) {
			set_video_reg_raw(CMD_ADDR, i);
			set_video_reg_raw(CMD_DATA, cmds[j].par[i]);
		}
		set_video_reg_raw(CMD_STAT, 1);
	}
	
	i = 0;
	while(get_video_reg_raw(CMD_STAT) & 1) {
		msleep(1);
		i++;
		if(i > 10) {
			err("video command timeout");
			video_soft_reset();
			return -1;
		}
	}
	
	return 0;
}

static int video_update_hw_config(void)
{
	u16 p0 = 0;
	struct stbx25xx_video_cmd cmd = {
		.cmd = CMD_CFG,	.cnt = 1, .par = &p0,
	};
	
	if(video_issue_cmd(&cmd))
		return -1;
	
	return 0;
}


static inline void video_stop_proc(void)
{
	stbx25xx_video_val reg;
	
	reg = get_video_reg(VIDEO_CNTL);
	reg.video_cntl.svp = 0;
	set_video_reg(VIDEO_CNTL, reg);
}

static inline void video_restart_proc(void)
{
	stbx25xx_video_val reg;
	
	reg = get_video_reg(VIDEO_CNTL);
	reg.video_cntl.svp = 0;
	set_video_reg(VIDEO_CNTL, reg);
	
	set_video_reg_raw(PROC_IADDR, 0);
	
	reg.video_cntl.svp = 1;
	set_video_reg(VIDEO_CNTL, reg);
}

static void video_set_anti_flicker(struct stbx25xx_dvb_dev *dvb, int mode)
{
	stbx25xx_video_val reg;
	
	if(mutex_lock_interruptible(&dvb->osd_mode_mutex))
		return;
	
	if(mode < 0)
		mode = 0;
	
	if(mode > 3)
		mode = 3;
	
	reg = get_video_reg(OSD_MODE);
	
	reg.osd_mode.vpafc = mode;
	if(mode)
		reg.osd_mode.afvp = 1;
	else
		reg.osd_mode.afvp = 0;
	
	set_video_reg(OSD_MODE, reg);
	
	mutex_unlock(&dvb->osd_mode_mutex);
}

static int video_pause(struct stbx25xx_dvb_dev *dvb)
{
	struct stbx25xx_video_cmd cmd = { .cmd = CMD_PAUSE, };
	
	video_issue_cmd(&cmd);
	dvb->vid_state.play_state = VIDEO_FREEZED;
	
	return 0;
}

static void video_set_display_border(int top, int left)
{
	stbx25xx_video_val reg;
	
	reg.raw = 0;
	reg.disp_border.tb = top >> 1;
	reg.disp_border.lb = left >> 1;
	
	set_video_reg(DISP_BORDER, reg);
}

static int video_set_display_format(struct stbx25xx_dvb_dev *dvb, video_displayformat_t fmt)
{
	stbx25xx_video_val reg;
	
	reg = get_video_reg(DISP_MODE);
	
	switch(fmt) {
	case VIDEO_PAN_SCAN:
		reg.disp_mode.df = DF_PSCAN;
		break;
	case VIDEO_LETTER_BOX:
		reg.disp_mode.df = DF_LBOX;
		break;
	case VIDEO_CENTER_CUT_OUT:
		reg.disp_mode.df = DF_NORMAL;
		break;
	default:
		return -EINVAL;
	}
	
	set_video_reg(DISP_MODE, reg);
	dvb->vid_state.display_format = fmt;
	
	return 0;
}

static int video_fastforward(struct stbx25xx_dvb_dev *dvb, int n)
{
	return -EINVAL;
}

static int video_frame_switch(int mode)
{
	u16 p0 = mode;
	struct stbx25xx_video_cmd cmd = { .cmd = CMD_FRM_SW, .cnt = 1, .par = &p0 };
	
	return video_issue_cmd(&cmd);
}

static int video_slow(struct stbx25xx_dvb_dev *dvb, int n)
{
	return -EINVAL;
}

static void video_vfb_clear(struct stbx25xx_dvb_dev *dvb)
{
	memset(dvb->vfb_memory + dvb->vfb_data[BUF0_LUM], 0, dvb->vfb_size[BUF0_LUM]);
	memset(dvb->vfb_memory + dvb->vfb_data[BUF1_LUM], 0, dvb->vfb_size[BUF1_LUM]);
	memset(dvb->vfb_memory + dvb->vfb_data[BUF2_LUM], 0, dvb->vfb_size[BUF2_LUM]);
	memset(dvb->vfb_memory + dvb->vfb_data[BUF0_CHR], 128, dvb->vfb_size[BUF0_CHR]);
	memset(dvb->vfb_memory + dvb->vfb_data[BUF1_CHR], 128, dvb->vfb_size[BUF1_CHR]);
	memset(dvb->vfb_memory + dvb->vfb_data[BUF2_CHR], 128, dvb->vfb_size[BUF2_CHR]);
}

static int video_vrb_reset(int mode)
{
	u16 p0 = mode;
	struct stbx25xx_video_cmd cmd = { .cmd = CMD_RB_RST, .cnt = 1, .par = &p0 };
	
	return video_issue_cmd(&cmd);
}

static int video_set_format(struct stbx25xx_dvb_dev *dvb, video_format_t fmt)
{
	stbx25xx_video_val reg;
	
	if(fmt == dvb->vid_state.video_format)
		return 0;
	
	if(fmt != VIDEO_FORMAT_4_3 && fmt != VIDEO_FORMAT_16_9)
		return -EINVAL;
	
	reg = get_video_reg(DISP_MODE);
	reg.disp_mode.mon = (fmt == VIDEO_FORMAT_16_9);
	set_video_reg(DISP_MODE, reg);
	
	if(video_update_hw_config())
		return -EREMOTEIO;
	
	dvb->vid_state.video_format = fmt;
	
	return 0;
}

static void video_init_mem_pointers(struct stbx25xx_dvb_dev *dvb)
{
	u32 fb_base;

	if(dvb->vid_system != VIDEO_SYSTEM_NTSC) {
		fb_base = 0x16900; // use 0x16900 for 4MB PAL
		
		dvb->vfb_data[BUF2_LUM] = 0x0 + fb_base;
		dvb->vfb_data[BUF2_CHR] = 0x65400 + fb_base;
		dvb->vfb_data[BUF0_LUM] = 0x84e00 + fb_base;
		dvb->vfb_data[BUF0_CHR] = 0xea200 + fb_base;
		dvb->vfb_data[BUF1_LUM] = 0x11cc00 + fb_base;
		dvb->vfb_data[BUF1_CHR] = 0x182000 + fb_base;
		dvb->vfb_size[BUF2_LUM] = 0x65400;
		dvb->vfb_size[BUF1_LUM] = 0x65400;
		dvb->vfb_size[BUF0_LUM] = 0x65400;
		dvb->vfb_size[BUF2_CHR] = 0x19e00;
		dvb->vfb_size[BUF1_CHR] = 0x32a00;
		dvb->vfb_size[BUF0_CHR] = 0x32a00;
	} else {
		fb_base = 0x21D00; // use 0x21D00 for NTSC
		
		dvb->vfb_data[BUF2_LUM] = 0x0 + fb_base;
		dvb->vfb_data[BUF2_CHR] = 0x54600 + fb_base;
		dvb->vfb_data[BUF0_LUM] = 0x89d00 + fb_base;
		dvb->vfb_data[BUF0_CHR] = 0xde300 + fb_base;
		dvb->vfb_data[BUF1_LUM] = 0x108600 + fb_base;
		dvb->vfb_data[BUF1_CHR] = 0x15cc00 + fb_base;
		dvb->vfb_size[BUF2_LUM] = 0x54600;
		dvb->vfb_size[BUF1_LUM] = 0x54600;
		dvb->vfb_size[BUF0_LUM] = 0x54600;
		dvb->vfb_size[BUF2_CHR] = 0x2a300;
		dvb->vfb_size[BUF1_CHR] = 0x2a300;
		dvb->vfb_size[BUF0_CHR] = 0x2a300;
	}
}

static int video_set_system(struct stbx25xx_dvb_dev *dvb, video_system_t sys)
{
	stbx25xx_video_val reg;
	
	if(sys == dvb->vid_system)
		return 0;
	
	reg = get_video_reg(DISP_MODE);
	reg.disp_mode.pal = (sys == VIDEO_SYSTEM_PAL);
	set_video_reg(DISP_MODE, reg);
	
	if(video_update_hw_config())
		return -EREMOTEIO;
	
	// TODO:
	// denc_set_system(dvb, sys);
	
	dvb->vid_system = sys;
	video_init_mem_pointers(dvb);
	
	return 0;
}

static void video_show(void)
{
	stbx25xx_video_val reg;
	
	reg = get_video_reg(VIDEO_CNTL);
	reg.video_cntl.ev = 1;
	set_video_reg(VIDEO_CNTL, reg);
}

static void video_hide(void)
{
	stbx25xx_video_val reg;
	
	reg = get_video_reg(VIDEO_CNTL);
	reg.video_cntl.ev = 0;
	set_video_reg(VIDEO_CNTL, reg);
}

static void video_start_decoding(void)
{
	stbx25xx_video_val reg;
	
	reg = get_video_reg(VIDEO_CNTL);
	reg.video_cntl.svd = 1;
	set_video_reg(VIDEO_CNTL, reg);
}

static void video_stop_decoding(void)
{
	stbx25xx_video_val reg;
	
	reg = get_video_reg(VIDEO_CNTL);
	reg.video_cntl.svd = 0;
	set_video_reg(VIDEO_CNTL, reg);
}

static void video_get_size(struct stbx25xx_dvb_dev *dvb, video_size_t *size)
{

}

static int video_clip_busy(void)
{
	stbx25xx_video_val reg;
	
	reg = get_video_reg(CLIP_LEN);
	return (reg.clip_len.bv != 0);
}

static int video_clip_free(struct stbx25xx_dvb_dev *dvb)
{
	unsigned long flags;
	int ret;
	
	local_irq_save(flags);
	ret = !dvb->clip_busy[dvb->clip_wptr];
	local_irq_restore(flags);
	
	return ret;
}

static int video_clip_queue(struct stbx25xx_dvb_dev *dvb, const char *buf, size_t count, int nonblocking)
{
	size_t size, sent;
	unsigned long flags;
	stbx25xx_video_val reg;
	
	if(dvb->vid_state.stream_source != VIDEO_SOURCE_MEMORY)
		return -EINVAL;
	
	sent = 0;
	
	while(count) {
		local_irq_save(flags);
		if(dvb->clip_busy[dvb->clip_wptr]) {
			if(nonblocking) {
				local_irq_restore(flags);
				return sent ?: -EAGAIN;
			}
		} else {
			wait_for_completion(&dvb->clip_done);
		}
		local_irq_restore(flags);
		
		reg.raw = 0;
		if(dvb->clip_size[dvb->clip_wptr] > count) {
			reg.clip_len.eos = 1;
			size = count;
		} else {
			size = dvb->clip_size[dvb->clip_wptr];
		}
		reg.clip_len.bv = 1;
		reg.clip_len.vcblq = size;
		
		memcpy(dvb->clip_data[dvb->clip_wptr], buf, size);
		
		while(video_clip_busy()) {
			msleep(1);
		}
		
		dvb->clip_busy[dvb->clip_wptr] = 1;
		dvb->clip_wptr ^= 1;
		
		set_video_reg_raw(CLIP_ADDR, dvb->clip_phys[dvb->clip_wptr]);
		set_video_reg(CLIP_LEN, reg);
				
		count -= size;
		sent += size;
	}
	
	return sent;
}

static void video_still_interrupt(struct stbx25xx_dvb_dev *dvb, int irq)
{
	complete(&dvb->still_done);
}

static void video_clip_interrupt(struct stbx25xx_dvb_dev *dvb, int irq)
{
	dvb->clip_busy[dvb->clip_rptr] = 0;
	dvb->clip_rptr ^= 1;
	complete(&dvb->clip_done);
}

static void dummy_int_handler(struct stbx25xx_dvb_dev *dvb, int irq)
{
	u16 width = *(u16 *)(&dvb->user_data[0x1f4]);
	u16 height = *(u16 *)(&dvb->user_data[0x1f6]);
	
	info("video interrupt %d", irq);
	
	if(irq == VIDEO_PRC) {
		info("video resolution changed: width=0x%04x, height=0x%04x", width, height);
	}
}

static int video_set_source(struct stbx25xx_dvb_dev *dvb, video_stream_source_t src)
{
	stbx25xx_video_val reg;
	
	if(dvb->vid_state.play_state != VIDEO_STOPPED)
		return -EINVAL;
	
	reg = get_video_reg(VIDEO_CNTL);
	reg.video_cntl.vcm = (src == VIDEO_SOURCE_MEMORY);
	set_video_reg(VIDEO_CNTL, reg);
	
	if(src == VIDEO_SOURCE_MEMORY)
		video_install_int_handler(VIDEO_BRC, video_clip_interrupt);
	else
		video_install_int_handler(VIDEO_BRC, dummy_int_handler);
	
	dvb->vid_state.stream_source = src;
	
	return 0;
}

static int video_play(struct stbx25xx_dvb_dev *dvb)
{
	u16 p0 = 0;
	struct stbx25xx_video_cmd cmd = { .cmd = CMD_PLAY, };
	struct stbx25xx_video_cmd exit_still[2] = {
		{ .cmd = CMD_RB_RST | CMD_CHAIN, .cnt = 1, .par = &p0 },
		{ .cmd = CMD_FREEZE, },
	};
	
	if(dvb->still_mode) {
		video_issue_cmds(exit_still, 2);
		if(dvb->old_source != VIDEO_SOURCE_MEMORY) {
			video_stop_decoding();
			video_set_source(dvb, dvb->old_source);
			video_start_decoding();
		}
		dvb->still_mode = 0;
	}
	
	video_issue_cmd(&cmd);
	dvb->vid_state.play_state = VIDEO_PLAYING;
	
	return 0;
}

static int video_stillpicture(struct stbx25xx_dvb_dev *dvb, struct video_still_picture *pic)
{
	u16 p0 = 0;
	struct stbx25xx_video_cmd cmd = { .cmd = CMD_STILL, .cnt = 0, .par = &p0 };
	u8 *clip_data;
	
	clip_data = kmalloc(pic->size + 128, GFP_KERNEL);
	if(clip_data == NULL)
		return -ENOMEM;
	
	dvb->old_source = dvb->vid_state.stream_source;
	if(dvb->vid_state.stream_source != VIDEO_SOURCE_MEMORY) {
		video_stop_decoding();
		video_set_source(dvb, VIDEO_SOURCE_MEMORY);
		video_start_decoding();
	}
	video_vrb_reset(0);
	
	if(video_issue_cmd(&cmd)) {
		video_stop_decoding();
		video_set_source(dvb, dvb->old_source);
		if(dvb->vid_state.play_state == VIDEO_PLAYING)
			video_start_decoding();
		return -EREMOTEIO;
	}
	
	dvb->still_mode = 1;

	copy_from_user(clip_data, (void __user *)pic->iFrame, pic->size);
	memset(&clip_data[pic->size], 0, 128);
	
	video_install_int_handler(VIDEO_SEND, video_still_interrupt);
	
	video_clip_queue(dvb, clip_data, pic->size + 128, 0);
	wait_for_completion(&dvb->still_done);
	
	kfree(clip_data);
	video_remove_int_handler(VIDEO_SEND);
	
	return 0;
}

/**
	Hardware setup
*/
static void video_init_hw(struct stbx25xx_dvb_dev *dvb)
{
	stbx25xx_video_val reg;
	
	reg = get_video_reg(CICVCR);
	reg.cicvcr.viddis = 0;
	set_video_reg(CICVCR, reg);
	
	set_video_reg_raw(MASK, video_int_mask);
	set_video_reg_raw(VIDEO_MODE, 0);
	set_video_reg(DISP_MODE, def_display_mode);
	set_video_reg(OSD_MODE, def_osd_mode);
	set_video_reg_raw(VBI_CNTL, 0);
}

static int video_init_memory(struct stbx25xx_dvb_dev *dvb)
{
	stbx25xx_video_val reg;
	u32 clip, size;
	
	reg.raw = 0;
	reg.mem_seg.seg0a = SEG0_BASE >> 20;
	reg.mem_seg.seg0s = SEG0_SIZE;
	set_video_reg(MEM_SEG0, reg);
	
	reg.mem_seg.seg0a = SEG1_BASE >> 20;
	reg.mem_seg.seg0s = SEG1_SIZE;
	set_video_reg(MEM_SEG1, reg);
	
	reg.mem_seg.seg0a = SEG2_BASE >> 20;
	reg.mem_seg.seg0s = SEG2_SIZE;
	set_video_reg(MEM_SEG2, reg);
	
	set_video_reg_raw(USERDATA_BASE, (SEG1_ADDR + USER_OFFSET) >> 7);
	
	reg.raw = 0;
	reg.vbi_base.vbi0ba = (SEG1_ADDR + VBI0_OFFSET) >> 7;
	reg.vbi_base.vbi1ba = (SEG1_ADDR + VBI1_OFFSET) >> 7;
	set_video_reg(VBI_BASE, reg);
	
	set_video_reg_raw(VID0_GPBASE, (SEG2_ADDR + FB0_OFFSET) >> 7);
	set_video_reg_raw(VID0_IPBASE, (SEG2_ADDR + FB1_OFFSET) >> 7);
	set_video_reg_raw(VID0_CPBASE, (SEG2_ADDR + CUR_OFFSET) >> 7);
	set_video_reg_raw(RATEBUF_BASE, (SEG1_ADDR + RB_OFFSET) >> 7);
	set_video_reg_raw(FB_BASE, (SEG0_ADDR + VFB_OFFSET) >> 7);
	
	dvb->vfb_memory = ioremap_nocache(VIDEO_FB_BASE, VIDEO_FB_SIZE);
	if(dvb->vfb_memory == NULL) {
		dprintk("%s: Could not remap VFB memory\n", __func__);
		goto err_vfb;
	}
	dprintk("%s: VFB memory - 0x%08x @ %p (%d bytes)\n", __func__, VIDEO_FB_BASE, dvb->vfb_memory, VIDEO_FB_SIZE);
	
	dvb->mpeg_memory = ioremap_nocache(VIDEO_MPEG_BASE, VIDEO_MPEG_SIZE);
	if(dvb->mpeg_memory == NULL) {
		dprintk("%s: Could not remap MPEG memory\n", __func__);
		goto err_mpeg;
	}
	dprintk("%s: MPEG memory - 0x%08x @ %p (%d bytes)\n", __func__, VIDEO_MPEG_BASE, dvb->mpeg_memory, VIDEO_MPEG_SIZE);
	
	dvb->user_data = dvb->mpeg_memory + USER_OFFSET;
	dvb->vbi0_data = dvb->mpeg_memory + VBI0_OFFSET;
	dvb->vbi1_data = dvb->mpeg_memory + VBI1_OFFSET;
	dvb->rb_data = dvb->mpeg_memory + RB_OFFSET;
	
	/* Clip mode buffer must be 4K aligned */
	clip = (u32)dvb->rb_data;
	clip += 4096 - 1;
	clip &= ~(4096 - 1);
	
	size = RB_SIZE;
	size -= (clip - (u32)dvb->rb_data);
	size &= ~(8192 - 1);
	size /= 2;
	
	dvb->clip_data[0] = (void *)clip;
	dvb->clip_data[1] = (void *)(clip + size);	
	dvb->clip_phys[0] = (dvb->clip_data[0] - dvb->rb_data);
	dvb->clip_phys[1] = dvb->clip_phys[0] + size;
	dvb->clip_size[0] = size;
	dvb->clip_size[1] = size;
	
	dvb->osd_memory = ioremap_nocache(VIDEO_OSD_BASE, VIDEO_OSD_SIZE);
	if(dvb->osd_memory == NULL) {
		dprintk("%s: Could not remap OSD memory\n", __func__);
		goto err_osd;
	}
	dprintk("%s: OSD memory - 0x%08x @ %p (%d bytes)\n", __func__, VIDEO_OSD_BASE, dvb->osd_memory, VIDEO_OSD_SIZE);
	
	dvb->osdg_data = dvb->osd_memory + FB0_OFFSET;
	dvb->osdi_data = dvb->osd_memory + FB1_OFFSET;
	dvb->osdc_data = dvb->osd_memory + CUR_OFFSET;
	
	return 0;
	
err_osd:
	iounmap(dvb->mpeg_memory);
err_mpeg:
	iounmap(dvb->vfb_memory);
err_vfb:
	return -ENOMEM;
}

static void video_deinit_memory(struct stbx25xx_dvb_dev *dvb)
{
	iounmap(dvb->vfb_memory);
	iounmap(dvb->mpeg_memory);
	iounmap(dvb->osd_memory);	
}

static int video_init_firmware(struct stbx25xx_dvb_dev *dvb)
{
	int ret = 0, i;
	const struct firmware *fw;
	u16 word;

	/* request the firmware, this will block until someone uploads it */
	ret = request_firmware(&fw, "dvb-stbx25xx-vid.fw", dvb->dev);
	if (ret) {
		if (ret == -ENOENT) {
			err("could not load firmware,"
			       " file not found: dvb-stbx25xx-vid.fw");
			err("usually this should be in "
			       "/usr/lib/hotplug/firmware or /lib/firmware");
		} else {
			err("(cannot request firmware"
			       " (error %i)", ret);
		}
		return -EINVAL;
	}

	if (fw->size != 8192) {
		err("incorrect firmware size (got %d bytes, should be 8192 bytes)", fw->size);
		release_firmware(fw);
		return -EINVAL;
	}
	
	info("video firmware release %d version %d", fw->data[0], fw->data[1]);

	/* upload the firmware */
	set_video_reg_raw(WRT_PROT, 1);
	set_video_reg_raw(PROC_IADDR, 0x8000);
	
	for(i=0; i<8192; i+=2) {
		word = (fw->data[i] << 8) | fw->data[i+1];
		set_video_reg_raw(PROC_IDATA, word);
	}
	
	set_video_reg_raw(PROC_IADDR, 0);
	set_video_reg_raw(WRT_PROT, 0);
	
	/* verify the firmware */
	set_video_reg_raw(WRT_PROT, 1);
	set_video_reg_raw(PROC_IADDR, 0);
	
	for(i=0; i<8192; i+=2) {
		set_video_reg_raw(PROC_IADDR, i >> 1);
		word = (fw->data[i] << 8) | fw->data[i+1];
		if(get_video_reg_raw(PROC_IDATA) != word) {
			err("firmware verification failure at address %d!", i);
			ret = -EAGAIN;
			break;
		}
	}
	
	set_video_reg_raw(PROC_IADDR, 0);
	set_video_reg_raw(WRT_PROT, 0);
	
	/* release the firmware after uploading */
	release_firmware(fw);
	
	return ret;
}

/**
	Exported API calls
*/
static void video_add_event(struct stbx25xx_dvb_dev *dvb, u16 width, u16 height, u16 aspect_ratio)
{
	struct video_event *event;
	int wp;

	spin_lock_irq(&dvb->vid_events.lock);

	wp = (dvb->vid_events.eventw + 1) % MAX_VIDEO_EVENT;

	if (wp == dvb->vid_events.eventr) {
		dvb->vid_events.overflow = 1;
		dvb->vid_events.eventr = (dvb->vid_events.eventr + 1) % MAX_VIDEO_EVENT;
	}

	event = &dvb->vid_events.events[dvb->vid_events.eventw];
	event->type = VIDEO_EVENT_SIZE_CHANGED;
	event->u.size.w = width;
	event->u.size.h = height;

	switch (aspect_ratio) {
	case 2:
		event->u.size.aspect_ratio = VIDEO_FORMAT_4_3;
		break;
	case 3:
		event->u.size.aspect_ratio = VIDEO_FORMAT_16_9;
		break;
	case 4:
		event->u.size.aspect_ratio = VIDEO_FORMAT_221_1;
		break;
	default:
		event->u.size.aspect_ratio = VIDEO_FORMAT_4_3;
		break;
	}

	dvb->vid_events.eventw = wp;

	spin_unlock_irq(&dvb->vid_events.lock);

	wake_up_interruptible(&dvb->vid_events.wait_queue);
}

static int video_get_event(struct stbx25xx_dvb_dev *dvb, struct video_event *event, int flags)
{
	int ret;

	if (dvb->vid_events.overflow) {
		dvb->vid_events.overflow = 0;
		return -EOVERFLOW;
	}

	if (dvb->vid_events.eventw == dvb->vid_events.eventr) {
		if (flags & O_NONBLOCK)
			return -EWOULDBLOCK;

		ret = wait_event_interruptible(dvb->vid_events.wait_queue,
				dvb->vid_events.eventw != dvb->vid_events.eventr);

		if (ret < 0)
			return ret;
	}

	spin_lock_irq(&dvb->vid_events.lock);

	memcpy(event, &dvb->vid_events.events[dvb->vid_events.eventr],
			sizeof(struct video_event));

	dvb->vid_events.eventr = (dvb->vid_events.eventr + 1) % MAX_VIDEO_EVENT;

	spin_unlock_irq(&dvb->vid_events.lock);

	return 0;
}

ssize_t stbx25xx_video_write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
	struct dvb_device *dvbdev;
	struct stbx25xx_dvb_dev *dvb;
	
	if(file == NULL)
		return -EINVAL;
	
	dvbdev = file->private_data;
	
	if(dvbdev == NULL)
		return -EINVAL;
	
	dvb = dvbdev->priv;
	
	if(dvb == NULL)
		return -EINVAL;
	
	if (((file->f_flags & O_ACCMODE) == O_RDONLY) ||
		(dvb->vid_state.stream_source != VIDEO_SOURCE_MEMORY))
			return -EPERM;

	return video_clip_queue(dvb, buf, count, file->f_flags & O_NONBLOCK);
}

int stbx25xx_video_open(struct inode *inode, struct file *file)
{
	int err;
	struct dvb_device *dvbdev;
	struct stbx25xx_dvb_dev *dvb;
	
	if(file == NULL)
		return -EINVAL;
	
	dvbdev = file->private_data;
	
	if(dvbdev == NULL)
		return -EINVAL;
	
	dvb = dvbdev->priv;
	
	if(dvb == NULL)
		return -EINVAL;
	
	if ((err = dvb_generic_open(inode, file)) < 0)
		return err;
	
	video_restart_proc();
	
	if(video_update_hw_config())
		return -EREMOTEIO;
	
	if((err = video_set_format(dvb, VIDEO_FORMAT_4_3)) != 0)
		return err;
		
	video_set_display_format(dvb, VIDEO_CENTER_CUT_OUT);
	video_set_display_border(0, 0);
	
	if(video_vrb_reset(0))
		return -EREMOTEIO;
	
	video_vfb_clear(dvb);
	
	video_start_decoding();
	
	err = video_frame_switch(1);
	if(err)
		warn("frame switch error");
	
	err = video_frame_switch(0);
	if(err)
		warn("frame switch error");
		
	video_stop_decoding();
	
	video_show();

	if ((file->f_flags & O_ACCMODE) != O_RDONLY) {
		/* empty event queue */
		dvb->vid_events.eventr = 0;
		dvb->vid_events.eventw = 0;
	}

	return 0;
}

int stbx25xx_video_release(struct inode *inode, struct file *file)
{
	struct dvb_device *dvbdev;
	struct stbx25xx_dvb_dev *dvb;
	
	if(file == NULL)
		return -EINVAL;
	
	dvbdev = file->private_data;
	
	if(dvbdev == NULL)
		return -EINVAL;
	
	dvb = dvbdev->priv;
	
	if(dvb == NULL)
		return -EINVAL;
	
	if ((file->f_flags & O_ACCMODE) != O_RDONLY) {
		video_hide();
		video_stop_decoding();
		dvb->vid_state.play_state = VIDEO_STOPPED;
		video_set_source(dvb, VIDEO_SOURCE_DEMUX);
		video_vrb_reset(0);
		video_stop_proc();
	}

	return dvb_generic_release(inode, file);
}

unsigned int stbx25xx_video_poll(struct file *file, poll_table *wait)
{
	unsigned int mask = 0;
	struct dvb_device *dvbdev;
	struct stbx25xx_dvb_dev *dvb;
	
	if(file == NULL)
		return -EINVAL;
	
	dvbdev = file->private_data;
	
	if(dvbdev == NULL)
		return -EINVAL;
	
	dvb = dvbdev->priv;
	
	if(dvb == NULL)
		return -EINVAL;

	if ((file->f_flags & O_ACCMODE) != O_RDONLY)
		poll_wait(file, &dvb->vid_write_wq, wait);

	poll_wait(file, &dvb->vid_events.wait_queue, wait);

	if (dvb->vid_events.eventw != dvb->vid_events.eventr)
		mask |= POLLPRI;

	if ((file->f_flags & O_ACCMODE) != O_RDONLY) {
		if (dvb->vid_state.play_state == VIDEO_PLAYING) {
			if (video_clip_free(dvb))
				mask |= (POLLOUT | POLLWRNORM);
		} else {
			mask |= (POLLOUT | POLLWRNORM);
		}
	}

	return mask;
}

int stbx25xx_video_ioctl(struct inode *inode, struct file *file, unsigned int cmd, void *parg)
{
	unsigned long arg = (unsigned long) parg;
	struct dvb_device *dvbdev;
	struct stbx25xx_dvb_dev *dvb;
	
	if(file == NULL)
		return -EINVAL;
	
	dvbdev = file->private_data;
	
	if(dvbdev == NULL)
		return -EINVAL;
	
	dvb = dvbdev->priv;
	
	if(dvb == NULL)
		return -EINVAL;

	if (((file->f_flags & O_ACCMODE) == O_RDONLY) &&
		(cmd != VIDEO_GET_STATUS) &&
		(cmd != VIDEO_GET_EVENT) &&
		(cmd != VIDEO_GET_SIZE))
		return -EPERM;

	switch (cmd) {
	case VIDEO_STOP:
		video_stop_decoding();
		dvb->vid_state.play_state = VIDEO_STOPPED;
		break;
		
	case VIDEO_FREEZE:
		return video_pause(dvb);

	case VIDEO_PLAY:
		video_start_decoding();
		dvb->vid_state.play_state = VIDEO_PLAYING;
		break;
		
	case VIDEO_CONTINUE:
		return video_play(dvb);

	case VIDEO_SELECT_SOURCE:
		return video_set_source(dvb, (video_stream_source_t) arg);

	case VIDEO_SET_BLANK:
		dvb->vid_state.video_blank = !!arg;
		break;

	case VIDEO_GET_STATUS:
		memcpy(parg, &dvb->vid_state, sizeof(struct video_status));
		break;

	case VIDEO_GET_EVENT:
		return video_get_event(dvb, parg, file->f_flags);

	case VIDEO_SET_DISPLAY_FORMAT:
		return video_set_display_format(dvb, (video_displayformat_t) arg);

	case VIDEO_STILLPICTURE:
		return video_stillpicture(dvb, parg);

	case VIDEO_FAST_FORWARD:
		return video_fastforward(dvb, arg);

	case VIDEO_SLOWMOTION:
		return video_slow(dvb, arg);

	case VIDEO_GET_CAPABILITIES:
		*((unsigned int *)parg) = STBx25xx_VIDEO_CAPS;
		break;

	case VIDEO_CLEAR_BUFFER:
		video_vfb_clear(dvb);
		video_vrb_reset(0);
		break;

	case VIDEO_SET_ID:
		return -EOPNOTSUPP;

	case VIDEO_SET_STREAMTYPE:
		return -EOPNOTSUPP;

	case VIDEO_SET_FORMAT:
		return video_set_format(dvb, (video_format_t) arg);

	case VIDEO_SET_SYSTEM:
		return video_set_system(dvb, (video_system_t) arg);

	case VIDEO_SET_HIGHLIGHT:
		return -EOPNOTSUPP;

	case VIDEO_SET_SPU:
		return -EOPNOTSUPP;

	case VIDEO_SET_SPU_PALETTE:
		return -EOPNOTSUPP;

	case VIDEO_GET_NAVI:
		return -EOPNOTSUPP;

	case VIDEO_SET_ATTRIBUTES:
		return -EOPNOTSUPP;

	case VIDEO_GET_SIZE:
		video_get_size(dvb, (video_size_t *)parg);
		break;
		
	case STB_VID_SETAF:
		video_set_anti_flicker(dvb, arg);
		break;

	default:
		return -ENOIOCTLCMD;
	}

	return 0;
}

/**
	Module init/exit
*/
int stbx25xx_video_init(struct stbx25xx_dvb_dev *dvb)
{
	int ret, i;
	
	printk(KERN_INFO "--- STBx25xx MPEG-2 Video Decoder driver ---\n");
	
	mutex_init(&dvb->osd_mode_mutex);
	init_completion(&dvb->clip_done);
	dvb->clip_rptr = 0;
	dvb->clip_wptr = 0;
	init_completion(&dvb->still_done);
	dvb->still_mode = 0;
	
	video_int_mask = VIDEO_BRC | VIDEO_ACCC | VIDEO_SERR | VIDEO_PRC;
	
	for(i = 0; i < STBx25xx_VIDEO_IRQ_COUNT; i++)
		video_install_int_handler(i, dummy_int_handler);
	
	video_init_hw(dvb);
	
	if((ret = request_irq(dvb->irq_num[STBx25xx_IRQ_VIDEO], video_interrupt, IRQF_SHARED | IRQF_TRIGGER_HIGH, "video", dvb)) != 0) {
		err("failed to request video irq: error %d", ret);
		goto err_irq;
	}
	
	if((ret = video_init_memory(dvb)) != 0) {
		err("memory initialization failed.");
		goto err_mem;
	}
	
	if((ret = video_init_firmware(dvb)) != 0) {
		err("firmware initialization failed.");
		goto err_fw;
	}
	
#ifndef CONFIG_STBx25xx_NTSC
	dvb->vid_system = VIDEO_SYSTEM_PAL;
#else
	dvb->vid_system = VIDEO_SYSTEM_NTSC;
#endif

	video_init_mem_pointers(dvb);
		
#if defined(CONFIG_FB) || defined(CONFIG_FB_MODULE)
	stbx25xx_osd_init(dvb);
#endif
	
	return 0;

err_fw:
	video_deinit_memory(dvb);
err_mem:
	free_irq(dvb->irq_num[STBx25xx_IRQ_VIDEO], dvb);
err_irq:
	return ret;
}

void stbx25xx_video_exit(struct stbx25xx_dvb_dev *dvb)
{
	/* TODO: Disable the hardware (?) */
	
	/* TODO: Tidy up */

#if defined(CONFIG_FB) || defined(CONFIG_FB_MODULE)
	stbx25xx_osd_exit(dvb);
#endif
	
	set_video_reg_raw(VIDEO_CNTL, 0);
	video_deinit_memory(dvb);
}
