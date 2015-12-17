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
#include <linux/proc_fs.h>

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
	u32 *par;
};

/**
	Interrupts
*/
u32 video_int_mask;
typedef void (*video_irq_handler_t)(struct stbx25xx_video_data *vid, int irq);
static video_irq_handler_t video_int_handlers[STBx25xx_VIDEO_IRQ_COUNT];
static u32 irq_stats[STBx25xx_VIDEO_IRQ_COUNT];

static char *irq_name[STBx25xx_VIDEO_IRQ_COUNT] = {
	"End of service picture",
	"Freeze frame status",
	"Reserved 2",
	"Reserved 3",
	"Saved PTS",
	"STC ready",
	"Zoom offset overflow",
	"Automatic channel change complete",
	"PLB error",
	"Block read complete",
	"Reserved 10",
	"Reserved 11",
	"Reserved 12",
	"Reserved 13",
	"Reserved 14",
	"Reserved 15",
	"Sequence start",
	"Sequence error",
	"Sequence end",
	"GOP/SMPTE",
	"Picture skipped",
	"Picture start",
	"Picture resolution changes",
	"User data",
	"VBI start",
	"Video start",
	"First field video start",
	"Block move complete",
	"Time base change",
	"Video rate buffer threshold",
	"Video rate buffer overflow",
	"OSD data",
};

static void video_enable_irq(unsigned int irq)
{
	unsigned long flags;
	
	local_irq_save(flags);
	video_int_mask |= 1 << (31 - irq);
	set_video_reg_raw(MASK, video_int_mask);
	local_irq_restore(flags);
}

static void video_disable_irq(unsigned int irq)
{
	unsigned long flags;
	
	local_irq_save(flags);
	video_int_mask &= ~(1 << (31 - irq));
	set_video_reg_raw(MASK, video_int_mask);
	local_irq_restore(flags);
}

static irqreturn_t video_interrupt(int irq, void *data)
{
	struct stbx25xx_video_data *vid = data;
	u32 mask = get_video_reg_raw(VIDEO_INT) & video_int_mask;
	int i;
	
	for(i = STBx25xx_VIDEO_IRQ_COUNT - 1; i >= 0 && mask; i--, mask >>= 1) {
		if(mask & 1) {
			irq_stats[i]++;
			if(video_int_handlers[i])
				video_int_handlers[i](vid, i);
		}
	}
	
	return IRQ_HANDLED;
}

static void video_install_int_handler(unsigned int irq, video_irq_handler_t handler)
{
	unsigned long flags;
	
	if(irq >= STBx25xx_VIDEO_IRQ_COUNT)
		return;
	
	if(video_int_handlers[irq])
		dprintk("%s: Replacing interrupt handler (IRQ %d)\n", __func__, irq);
	
	local_irq_save(flags);
	video_int_handlers[irq] = handler;
	local_irq_restore(flags);
}

static void video_remove_int_handler(int irq)
{
	unsigned long flags;
	
	if(irq >= STBx25xx_VIDEO_IRQ_COUNT)
		return;
	
	if(!video_int_handlers[irq])
		dprintk("%s: Interrupt handler already free (IRQ %d)\n", __func__, irq);
	
	local_irq_save(flags);
	video_int_handlers[irq] = NULL;
	local_irq_restore(flags);
}


/**
	Hardware manipulation routines
*/
static inline void video_soft_reset(void)
{
	set_video_reg_raw(CMD_STAT, 0);
	set_video_reg_raw(PROC_IADDR, 0x8200);	
}

static inline int video_force_cmd(void)
{
	int i;
	
	video_soft_reset();
	
	set_video_reg_raw(CMD, CMD_RB_RST | CMD_CHAIN);
	set_video_reg_raw(CMD_ADDR, 0);
	set_video_reg_raw(CMD_DATA, 0);
	set_video_reg_raw(CMD_STAT, 1);
	
	i = 0;
	while(get_video_reg_raw(CMD_STAT) & 1) {
		msleep(1);
		i++;
 		if(i > VIDEO_CMD_TIMEOUT_MS) {
			return -1;
		}
	}
	
	return 0;
}

static int video_issue_cmd(struct stbx25xx_video_cmd *cmd)
{
	int i;
	int force = 1;
	
	i = 0;
	while(get_video_reg_raw(CMD_STAT) & 1) {
		msleep(1);
		i++;
 		if(i > VIDEO_CMD_TIMEOUT_MS) {
			if(video_force_cmd()) {
				err("Host command interface stuck!");
				video_soft_reset();
				return -1;
			}
			break;
		}
	}
	
retry:
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
 		if(i > VIDEO_CMD_TIMEOUT_MS) {
			if(!force) {
				err("Video command timeout!");
				video_soft_reset();
				return -1;
			}
			
			if(video_force_cmd()) {
				err("Host command interface stuck!");
				video_soft_reset();
				return -1;
			}
			
			dprintk("%s: Forcing command to be executed\n", __func__);
			force = 0;
			goto retry;
		}
	}
	
	return 0;
}

static int video_issue_cmds(struct stbx25xx_video_cmd *cmds, u8 count)
{
	int i;
	
	for(i=0; i<count; i++)
		if(video_issue_cmd(&cmds[i]))
			return -1;
	
	return 0;
}

static int video_update_hw_config(struct stbx25xx_video_data *vid)
{
	u32 p0 = 0;
	struct stbx25xx_video_cmd cmd = {
		.cmd = CMD_CFG,	.cnt = 1, .par = &p0,
	};
	
	if(vid->system == VIDEO_SYSTEM_NTSC)
		set_video_reg_raw(FB_BASE, (SEG0_ADDR + VFB_OFFSET + NTSC_OFFSET) >> 7);
	else
		set_video_reg_raw(FB_BASE, (SEG0_ADDR + VFB_OFFSET + PAL_OFFSET) >> 7);
	
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

static void video_set_anti_flicker(struct stbx25xx_video_data *vid, int mode)
{
	stbx25xx_video_val reg;
	
	if(mutex_lock_interruptible(&vid->osd_mode_mutex))
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
	
	mutex_unlock(&vid->osd_mode_mutex);
}

static int video_pause(struct stbx25xx_video_data *vid)
{
	struct stbx25xx_video_cmd cmd = { .cmd = CMD_PAUSE, };
	
	video_issue_cmd(&cmd);
	vid->state.play_state = VIDEO_FREEZED;
	
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

static int video_set_display_format(struct stbx25xx_video_data *vid, video_displayformat_t fmt)
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
	vid->state.display_format = fmt;
	
	return 0;
}

static int video_fastforward(struct stbx25xx_video_data *vid, int n)
{
	return -EINVAL;
}

static int video_frame_switch(int mode)
{
	u32 p0 = mode;
	struct stbx25xx_video_cmd cmd = { .cmd = CMD_FRM_SW, .cnt = 1, .par = &p0 };
	
	return video_issue_cmd(&cmd);
}

static int video_slow(struct stbx25xx_video_data *vid, int n)
{
	return -EINVAL;
}

static void video_vfb_clear(struct stbx25xx_video_data *vid)
{
	memset(vid->vfb_memory + vid->vfb_data[BUF0_LUM], 0, vid->vfb_size[BUF0_LUM]);
	memset(vid->vfb_memory + vid->vfb_data[BUF1_LUM], 0, vid->vfb_size[BUF1_LUM]);
	memset(vid->vfb_memory + vid->vfb_data[BUF2_LUM], 0, vid->vfb_size[BUF2_LUM]);
	memset(vid->vfb_memory + vid->vfb_data[BUF0_CHR], 128, vid->vfb_size[BUF0_CHR]);
	memset(vid->vfb_memory + vid->vfb_data[BUF1_CHR], 128, vid->vfb_size[BUF1_CHR]);
	memset(vid->vfb_memory + vid->vfb_data[BUF2_CHR], 128, vid->vfb_size[BUF2_CHR]);
}

static int video_vrb_reset(int mode)
{
	u32 p0 = mode;
	struct stbx25xx_video_cmd cmd = { .cmd = CMD_RB_RST, .cnt = 1, .par = &p0 };
	
	video_soft_reset();
	
	return video_issue_cmd(&cmd);
}

static int video_vrb_reset_freeze(int mode)
{
	u32 p0 = mode;
	struct stbx25xx_video_cmd cmds[2] = { 
		{.cmd = CMD_RB_RST | CMD_CHAIN, .cnt = 1, .par = &p0, },
		{.cmd = CMD_FREEZE, .cnt = 0, },
	};
	
	video_soft_reset();
	
	return video_issue_cmds(cmds, 2);
}

static int video_set_format(struct stbx25xx_video_data *vid, video_format_t fmt)
{
	stbx25xx_video_val reg;
	
	if(fmt == vid->state.video_format)
		return 0;
	
	if(fmt != VIDEO_FORMAT_4_3 && fmt != VIDEO_FORMAT_16_9)
		return -EINVAL;
	
	reg = get_video_reg(DISP_MODE);
	reg.disp_mode.mon = (fmt == VIDEO_FORMAT_16_9);
	set_video_reg(DISP_MODE, reg);
	
	if(video_update_hw_config(vid))
		return -EREMOTEIO;
	
	vid->state.video_format = fmt;
	
	return 0;
}

static void video_set_sfm(struct stbx25xx_video_data *vid, int sfm)
{
	stbx25xx_video_val reg;
	
	if(vid->sfm != SFM_NORMAL)
		return;
	
	reg = get_video_reg(DISP_MODE);
	reg.disp_mode.sfm = sfm;
	set_video_reg(DISP_MODE, reg);
	
	vid->sfm = sfm;
}

static void video_init_mem_pointers(struct stbx25xx_video_data *vid)
{
	u32 fb_base;

	if(vid->system != VIDEO_SYSTEM_NTSC) {
		fb_base = PAL_OFFSET; // use 0x16900 for 4MB PAL
		
		vid->vfb_data[BUF2_LUM] = 0x0 + fb_base;
		vid->vfb_data[BUF2_CHR] = 0x65400 + fb_base;
		vid->vfb_data[BUF0_LUM] = 0x84e00 + fb_base;
		vid->vfb_data[BUF0_CHR] = 0xea200 + fb_base;
		vid->vfb_data[BUF1_LUM] = 0x11cc00 + fb_base;
		vid->vfb_data[BUF1_CHR] = 0x182000 + fb_base;
		vid->vfb_size[BUF2_LUM] = 0x65400;
		vid->vfb_size[BUF1_LUM] = 0x65400;
		vid->vfb_size[BUF0_LUM] = 0x65400;
		vid->vfb_size[BUF2_CHR] = 0x19e00;
		vid->vfb_size[BUF1_CHR] = 0x32a00;
		vid->vfb_size[BUF0_CHR] = 0x32a00;
	} else {
		fb_base = NTSC_OFFSET; // use 0x21D00 for NTSC
		
		vid->vfb_data[BUF2_LUM] = 0x0 + fb_base;
		vid->vfb_data[BUF2_CHR] = 0x54600 + fb_base;
		vid->vfb_data[BUF0_LUM] = 0x89d00 + fb_base;
		vid->vfb_data[BUF0_CHR] = 0xde300 + fb_base;
		vid->vfb_data[BUF1_LUM] = 0x108600 + fb_base;
		vid->vfb_data[BUF1_CHR] = 0x15cc00 + fb_base;
		vid->vfb_size[BUF2_LUM] = 0x54600;
		vid->vfb_size[BUF1_LUM] = 0x54600;
		vid->vfb_size[BUF0_LUM] = 0x54600;
		vid->vfb_size[BUF2_CHR] = 0x2a300;
		vid->vfb_size[BUF1_CHR] = 0x2a300;
		vid->vfb_size[BUF0_CHR] = 0x2a300;
	}
}

static int video_set_system(struct stbx25xx_video_data *vid, video_system_t sys)
{
	stbx25xx_video_val reg;
	
	if(sys == vid->system)
		return 0;
	
	reg = get_video_reg(DISP_MODE);
	reg.disp_mode.pal = (sys == VIDEO_SYSTEM_PAL);
	set_video_reg(DISP_MODE, reg);
	
	if(video_update_hw_config(vid))
		return -EREMOTEIO;
	
	// TODO:
	// denc_set_system(vid, sys);
	
	vid->system = sys;
	video_init_mem_pointers(vid);
	
	return 0;
}

static void video_show(void)
{
	unsigned long flags;
	stbx25xx_video_val reg;
	
	local_irq_save(flags);
	
	reg = get_video_reg(VIDEO_CNTL);
	reg.video_cntl.ev = 1;
	set_video_reg(VIDEO_CNTL, reg);
	
	local_irq_restore(flags);
}

static void video_hide(void)
{
	unsigned long flags;
	stbx25xx_video_val reg;
	
	local_irq_save(flags);
	
	reg = get_video_reg(VIDEO_CNTL);
	reg.video_cntl.ev = 0;
	set_video_reg(VIDEO_CNTL, reg);
	
	local_irq_restore(flags);
}

static void video_start_decoding(void)
{
	unsigned long flags;
	stbx25xx_video_val reg;
	
	local_irq_save(flags);
	
	reg = get_video_reg(VIDEO_CNTL);
	reg.video_cntl.svd = 1;
	set_video_reg(VIDEO_CNTL, reg);
	
	local_irq_restore(flags);
}

static void video_stop_decoding(void)
{
	unsigned long flags;
	stbx25xx_video_val reg;
	
	local_irq_save(flags);
	
	reg = get_video_reg(VIDEO_CNTL);
	reg.video_cntl.svd = 0;
	set_video_reg(VIDEO_CNTL, reg);
	
	local_irq_restore(flags);
}

#define VIDEO_SYNC_AMASTER	0
#define VIDEO_SYNC_VMASTER	1
static void video_enable_sync(int master)
{
	unsigned long flags;
	stbx25xx_video_val reg;
	
	local_irq_save(flags);
	
	reg = get_video_reg(VIDEO_CNTL);
	reg.video_cntl.vmsm = (master == VIDEO_SYNC_VMASTER);
	reg.video_cntl.amsm = (master == VIDEO_SYNC_AMASTER);
	reg.video_cntl.ds = 0;
	set_video_reg(VIDEO_CNTL, reg);
	
	local_irq_restore(flags);
	
	if(master == VIDEO_SYNC_VMASTER)
		video_enable_irq(VIDEO_STC_IRQ);
}

static void video_disable_sync(void)
{
	unsigned long flags;
	stbx25xx_video_val reg;
	
	video_disable_irq(VIDEO_STC_IRQ);
	
	local_irq_save(flags);
	
	reg = get_video_reg(VIDEO_CNTL);
	reg.video_cntl.ds = 1;
	set_video_reg(VIDEO_CNTL, reg);
	
	local_irq_restore(flags);
}

static void video_stc_ready_handler(struct stbx25xx_video_data *vid, int irq)
{
	unsigned int stchi, stclo;
	
	stchi = get_video_reg_raw(SYNC_STC0);
	stclo = get_video_reg_raw(SYNC_STC1);
	
	stbx25xx_audio_sync_stc(stclo, stchi);
}

static int video_play(struct stbx25xx_video_data *vid)
{
	struct stbx25xx_video_cmd cmd = { .cmd = CMD_PLAY, };
	
	video_start_decoding();
//	video_enable_sync(VIDEO_SYNC_VMASTER);

	if(video_issue_cmd(&cmd))
		return -EREMOTEIO;
	
	vid->state.play_state = VIDEO_PLAYING;
	
	return 0;
}

static int video_stop(struct stbx25xx_video_data *vid)
{
	struct stbx25xx_video_cmd freeze = { .cmd = CMD_FREEZE, .cnt = 0, .par = NULL };
	
	if(video_issue_cmd(&freeze))
		return -EREMOTEIO;
	
//	video_disable_sync();
	video_stop_decoding();
	
	video_vrb_reset(0x8200);
	video_vfb_clear(vid);
	
	vid->state.play_state = VIDEO_STOPPED;
	
	return 0;
}

static void video_get_size(struct stbx25xx_video_data *vid, video_size_t *size)
{

}

static int video_block_valid(void)
{
	stbx25xx_video_val reg;
	
	reg = get_video_reg(CLIP_LEN);
	return (reg.clip_len.bv != 0);
}

static int video_clip_free(struct stbx25xx_video_data *vid)
{
	unsigned long flags;
	int ret;
	
	local_irq_save(flags);
	ret = !vid->clip_busy[vid->clip_wptr];
	local_irq_restore(flags);
	
	return ret;
}

static int video_clip_queue(struct stbx25xx_video_data *vid, const char *buf, size_t count, int nonblocking, int user)
{
	size_t size, sent;
	stbx25xx_video_val reg;
	unsigned long flags;
	
	BUG_ON(in_irq());
	
	if(vid->state.stream_source != VIDEO_SOURCE_MEMORY)
		return -EINVAL;
	
	sent = 0;
	
	while(count) {
		if(!video_clip_free(vid)) {
			if(nonblocking) {
				info("Video clip not free in non-blocking mode - exiting.");
				return sent ?: -EAGAIN;
			} else {
				if(wait_event_killable(vid->clip_avail, video_clip_free(vid)) < 0)
					return -ERESTARTSYS;
			}
		}
		
		reg.raw = 0;
#if 0
		if(vid->clip_size[vid->clip_wptr] >= count) {
			reg.clip_len.eos = 1;
			size = count;
		} else {
			size = vid->clip_size[vid->clip_wptr];
		}
#endif
		size = vid->clip_size[vid->clip_wptr];
		size = (size >= count) ? count : size;
		reg.clip_len.bv = 1;
		reg.clip_len.vcblq = size;
	
#if 0
		if(video_block_valid()) {
			if(nonblocking) {
				info("Video block still queued in non-blocking mode - exiting.");
				return sent ?: -EAGAIN;
			} else {
				memcpy(vid->clip_data[vid->clip_wptr], buf, size);
				wait_for_completion(&vid->clip_done);
			}
		} 
		
		if(nonblocking)
#endif
		if(user)
			copy_from_user(vid->clip_data[vid->clip_wptr], &buf[sent], size);
		else
			memcpy(vid->clip_data[vid->clip_wptr], &buf[sent], size);
		
		local_irq_save(flags);
		vid->clip_busy[vid->clip_wptr] = 1;
		local_irq_restore(flags);
		
		set_video_reg_raw(CLIP_ADDR, vid->clip_phys[vid->clip_wptr]);
		set_video_reg(CLIP_LEN, reg);
		
		vid->clip_wptr ^= 1;
		
		if(vid->state.play_state != VIDEO_PLAYING)
			video_play(vid);
				
		count -= size;
		sent += size;
	}
	
	return sent;
}

static int video_clip_buffer_empty(struct stbx25xx_video_data *vid)
{
	stbx25xx_video_val reg;
	
	reg = get_video_reg(CLIP_LEN);

	return !(reg.clip_len.bv || reg.clip_len.sb);
}

static void video_empty_buffers(struct stbx25xx_video_data *vid)
{
	if(vid->state.stream_source == VIDEO_SOURCE_MEMORY) {
		info("Waiting for clip buffer to get empty...");
		wait_event_killable(vid->clip_avail, video_clip_buffer_empty(vid));
	}
}

static void video_still_interrupt(struct stbx25xx_video_data *vid, int irq)
{
	complete(&vid->still_done);
}

static void video_clip_interrupt(struct stbx25xx_video_data *vid, int irq)
{
	unsigned long flags;
	
	local_irq_save(flags);
	vid->clip_busy[vid->clip_rptr] = 0;
	local_irq_restore(flags);
	
	vid->clip_rptr ^= 1;
	wake_up(&vid->clip_avail);
}

static void dummy_int_handler(struct stbx25xx_video_data *vid, int irq)
{
	u16 width, height;
	
	info("Video IRQ: %s interrupt", irq_name[irq]);
	
	if(irq == VIDEO_PRC_IRQ) {
		width = *(u16 *)(&vid->user_data[0x1f4]);
		height = *(u16 *)(&vid->user_data[0x1f6]);
		info("new width: 0x%04x, new height: 0x%04x", width, height);
	}
}

static int video_set_source(struct stbx25xx_video_data *vid, video_stream_source_t src)
{
	unsigned long flags;
	stbx25xx_video_val reg;
	
	if(vid->state.play_state != VIDEO_STOPPED)
		return -EINVAL;
	
	reg = get_video_reg(VIDEO_CNTL);
	reg.video_cntl.vcm = (src == VIDEO_SOURCE_MEMORY);
	set_video_reg(VIDEO_CNTL, reg);
	
	if(src == VIDEO_SOURCE_MEMORY) {
		local_irq_save(flags);
		vid->clip_busy[0] = 0;
		vid->clip_busy[1] = 0;
		local_irq_restore(flags);
		
		video_install_int_handler(VIDEO_BRC_IRQ, video_clip_interrupt);
	} else {
		video_install_int_handler(VIDEO_BRC_IRQ, dummy_int_handler);
	}
	
	vid->state.stream_source = src;
	
	return 0;
}

static int video_resume(struct stbx25xx_video_data *vid)
{
	u32 p0 = 0;
	struct stbx25xx_video_cmd resume = { .cmd = CMD_PLAY, .cnt = 0, .par = NULL };
	struct stbx25xx_video_cmd exit_still[2] = {
		{ .cmd = CMD_RB_RST | CMD_CHAIN, .cnt = 1, .par = &p0 },
		{ .cmd = CMD_FREEZE, },
	};
	
	if(vid->still_mode) {
		video_issue_cmds(exit_still, 2);
		if(vid->old_source != VIDEO_SOURCE_MEMORY) {
			video_stop_decoding();
			video_set_source(vid, vid->old_source);
			video_start_decoding();
		}
		vid->still_mode = 0;
		video_issue_cmd(&resume);
	} else {
		video_vrb_reset(0x8000);
	}
	
	vid->state.play_state = VIDEO_PLAYING;
	
	return 0;
}

static int video_stillpicture(struct stbx25xx_video_data *vid, struct video_still_picture *pic)
{
	u32 p0 = 0;
	struct stbx25xx_video_cmd cmd = { .cmd = CMD_STILL, .cnt = 0, .par = &p0 };
	u8 *clip_data;
	
	clip_data = kmalloc(pic->size + 128, GFP_KERNEL);
	if(clip_data == NULL)
		return -ENOMEM;
	
	vid->old_source = vid->state.stream_source;
	if(vid->state.stream_source != VIDEO_SOURCE_MEMORY) {
		video_stop_decoding();
		video_set_source(vid, VIDEO_SOURCE_MEMORY);
		video_start_decoding();
	}
	video_vrb_reset(0);
	
	if(video_issue_cmd(&cmd)) {
		video_stop_decoding();
		video_set_source(vid, vid->old_source);
		if(vid->state.play_state == VIDEO_PLAYING)
			video_start_decoding();
		return -EREMOTEIO;
	}
	
	vid->still_mode = 1;

	copy_from_user(clip_data, (void __user *)pic->iFrame, pic->size);
	memset(&clip_data[pic->size], 0, 128);
	
	video_install_int_handler(VIDEO_SEND_IRQ, video_still_interrupt);
	
	video_clip_queue(vid, clip_data, pic->size + 128, 0, 0);
	wait_for_completion(&vid->still_done);
	
	kfree(clip_data);
	video_install_int_handler(VIDEO_SEND_IRQ, dummy_int_handler);
	
	return 0;
}

/**
	Hardware setup
*/
static void video_init_hw(struct stbx25xx_video_data *vid)
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

static int video_init_memory(struct stbx25xx_video_data *vid)
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
	set_video_reg_raw(RATEBUF_SIZE, RB_SIZE >> 5);
	
	if(vid->system == VIDEO_SYSTEM_NTSC)
		set_video_reg_raw(FB_BASE, (SEG0_ADDR + VFB_OFFSET + NTSC_OFFSET) >> 7);
	else
		set_video_reg_raw(FB_BASE, (SEG0_ADDR + VFB_OFFSET + PAL_OFFSET) >> 7);
	
	vid->vfb_memory = ioremap_nocache(VIDEO_FB_BASE, VIDEO_FB_SIZE);
	if(vid->vfb_memory == NULL) {
		dprintk("%s: Could not remap VFB memory\n", __func__);
		goto err_vfb;
	}
	dprintk("%s: VFB memory - 0x%08x @ %p (%d bytes)\n", __func__, VIDEO_FB_BASE, vid->vfb_memory, VIDEO_FB_SIZE);
	
	vid->mpeg_memory = ioremap_nocache(VIDEO_MPEG_BASE, VIDEO_MPEG_SIZE);
	if(vid->mpeg_memory == NULL) {
		dprintk("%s: Could not remap MPEG memory\n", __func__);
		goto err_mpeg;
	}
	dprintk("%s: MPEG memory - 0x%08x @ %p (%d bytes)\n", __func__, VIDEO_MPEG_BASE, vid->mpeg_memory, VIDEO_MPEG_SIZE);
	
	vid->user_data = vid->mpeg_memory + USER_OFFSET;
	vid->vbi0_data = vid->mpeg_memory + VBI0_OFFSET;
	vid->vbi1_data = vid->mpeg_memory + VBI1_OFFSET;
	vid->rb_data = vid->mpeg_memory + RB_OFFSET;
	
	/* Clip mode buffer must be 4K aligned */
	clip = (u32)vid->rb_data;
	clip += 4096 - 1;
	clip &= ~(4096 - 1);
	
	size = RB_SIZE;
	size -= (clip - (u32)vid->rb_data);
	size &= ~(8192 - 1);
	size /= 2;
	
	vid->clip_data[0] = (void *)clip;
	vid->clip_data[1] = (void *)(clip + size);	
	vid->clip_phys[0] = (vid->clip_data[0] - vid->rb_data);
	vid->clip_phys[1] = vid->clip_phys[0] + size;
	vid->clip_size[0] = size;
	vid->clip_size[1] = size;
	
	vid->osd_memory = ioremap_nocache(VIDEO_OSD_BASE, VIDEO_OSD_SIZE);
	if(vid->osd_memory == NULL) {
		dprintk("%s: Could not remap OSD memory\n", __func__);
		goto err_osd;
	}
	dprintk("%s: OSD memory - 0x%08x @ %p (%d bytes)\n", __func__, VIDEO_OSD_BASE, vid->osd_memory, VIDEO_OSD_SIZE);
	
	vid->osdg_data = vid->osd_memory + FB0_OFFSET;
	vid->osdi_data = vid->osd_memory + FB1_OFFSET;
	vid->osdc_data = vid->osd_memory + CUR_OFFSET;
	
	return 0;
	
err_osd:
	iounmap(vid->mpeg_memory);
err_mpeg:
	iounmap(vid->vfb_memory);
err_vfb:
	return -ENOMEM;
}

static void video_deinit_memory(struct stbx25xx_video_data *vid)
{
	iounmap(vid->vfb_memory);
	iounmap(vid->mpeg_memory);
	iounmap(vid->osd_memory);	
}

static int video_init_firmware(struct stbx25xx_video_data *vid)
{
	int ret = 0, i;
	const struct firmware *fw;
	u16 word;
	struct stbx25xx_dvb_data *dvb = container_of(vid, struct stbx25xx_dvb_data, video);

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
	procfs
*/

static int video_show_irqs(struct seq_file *p, void *v)
{
	int i = *(loff_t *)v, j;
	unsigned long flags;
	
	local_irq_save(flags);
	j = irq_stats[i];
	local_irq_restore(flags);
	
	seq_printf(p, "%2d : %8d : %s\n", i, j, irq_name[i]);
	
	return 0;
}

static void *video_seq_istart(struct seq_file *f, loff_t *pos)
{
	return (*pos < STBx25xx_VIDEO_IRQ_COUNT) ? pos : NULL;
}

static void *video_seq_inext(struct seq_file *f, void *v, loff_t *pos)
{
	(*pos)++;
	if (*pos >= STBx25xx_VIDEO_IRQ_COUNT)
		return NULL;
	return pos;
}

static void video_seq_istop(struct seq_file *f, void *v)
{
	/* Nothing to do */
}

static const struct seq_operations video_seq_iops = {
	.start = video_seq_istart,
	.next  = video_seq_inext,
	.stop  = video_seq_istop,
	.show  = video_show_irqs
};

static int video_iopen(struct inode *inode, struct file *file)
{
	return seq_open(file, &video_seq_iops);
}

static const struct file_operations proc_video_iops = {
	.open           = video_iopen,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = seq_release,
};

static struct proc_dir_entry *video_proc_irqs;

#define VIDEO_PROC_IRQS_NAME	"video_interrupts"
static void video_init_procfs(void)
{
	video_proc_irqs = proc_create(VIDEO_PROC_IRQS_NAME, 0, stbx25xx_proc_dir, &proc_video_iops);
}

static void video_deinit_procfs(void)
{
	remove_proc_entry(VIDEO_PROC_IRQS_NAME, stbx25xx_proc_dir);
}

/**
	Exported API calls
*/
void stbx25xx_video_sync_stc(u32 stcl, u32 stch)
{
	set_video_reg_raw(SYNC_STC1, (stcl & 0x1) << 9);
	set_video_reg_raw(SYNC_STC0, stch);
}

void stbx25xx_video_enable_sync(void)
{
	video_enable_sync(VIDEO_SYNC_VMASTER);
}

void stbx25xx_video_disable_sync(void)
{
	video_disable_sync();
}

static void video_add_event(struct stbx25xx_video_data *vid, u16 width, u16 height, u16 aspect_ratio)
{
	struct video_event *event;
	int wp;

	spin_lock_irq(&vid->events.lock);

	wp = (vid->events.eventw + 1) % MAX_VIDEO_EVENT;

	if (wp == vid->events.eventr) {
		vid->events.overflow = 1;
		vid->events.eventr = (vid->events.eventr + 1) % MAX_VIDEO_EVENT;
	}

	event = &vid->events.events[vid->events.eventw];
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

	vid->events.eventw = wp;

	spin_unlock_irq(&vid->events.lock);

	wake_up_interruptible(&vid->events.wait_queue);
}

static int video_get_event(struct stbx25xx_video_data *vid, struct video_event *event, int flags)
{
	int ret;

	if (vid->events.overflow) {
		vid->events.overflow = 0;
		return -EOVERFLOW;
	}

	if (vid->events.eventw == vid->events.eventr) {
		if (flags & O_NONBLOCK)
			return -EWOULDBLOCK;

		ret = wait_event_interruptible(vid->events.wait_queue,
				vid->events.eventw != vid->events.eventr);

		if (ret < 0)
			return ret;
	}

	spin_lock_irq(&vid->events.lock);

	memcpy(event, &vid->events.events[vid->events.eventr],
			sizeof(struct video_event));

	vid->events.eventr = (vid->events.eventr + 1) % MAX_VIDEO_EVENT;

	spin_unlock_irq(&vid->events.lock);

	return 0;
}

ssize_t stbx25xx_video_write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
	struct dvb_device *dvbdev;
	struct stbx25xx_dvb_data *dvb;
	struct stbx25xx_video_data *vid;
	
	if(file == NULL)
		return -EINVAL;
	
	dvbdev = file->private_data;
	
	if(dvbdev == NULL)
		return -EINVAL;
	
	dvb = dvbdev->priv;
	
	if(dvb == NULL)
		return -EINVAL;
	
	vid = &dvb->video;
	
	if (((file->f_flags & O_ACCMODE) == O_RDONLY) ||
		(vid->state.stream_source != VIDEO_SOURCE_MEMORY))
			return -EPERM;

	return video_clip_queue(vid, buf, count, file->f_flags & O_NONBLOCK, 1);
}

int stbx25xx_video_open(struct inode *inode, struct file *file)
{
	int err = 0;
	struct dvb_device *dvbdev;
	struct stbx25xx_dvb_data *dvb;
	struct stbx25xx_video_data *vid;
	
	if(file == NULL)
		return -EINVAL;
	
	dvbdev = file->private_data;
	
	if(dvbdev == NULL)
		return -EINVAL;
	
	dvb = dvbdev->priv;
	
	if(dvb == NULL)
		return -EINVAL;
	
	vid = &dvb->video;
	
	if ((err = dvb_generic_open(inode, file)) < 0)
		return err;
	
	if ((file->f_flags & O_ACCMODE) != O_RDONLY) {
		video_restart_proc();
		
		video_disable_sync();
		
		if((err = video_update_hw_config(vid)) != 0) {
			dprintk("%s: Failed to reconfigure the decoder\n", __func__);
			goto error;
		}
		
		if((err = video_set_format(vid, VIDEO_FORMAT_4_3)) != 0) {
			dprintk("%s: Failed to set video aspect ratio\n", __func__);
			goto error;
		}
		
		video_set_display_format(vid, VIDEO_CENTER_CUT_OUT);
		video_set_display_border(0, 0);
		
		if(video_vrb_reset(0)) {
			dprintk("%s: Failed to reset rate buffer\n", __func__);
			err = -EREMOTEIO;
			goto error;
		}

		video_vfb_clear(vid);

		video_start_decoding();

		err = video_frame_switch(1);
		if(err)
			warn("frame switch error");

		err = video_frame_switch(0);
		if(err)
			warn("frame switch error");

		video_stop_decoding();
		
		if(video_vrb_reset_freeze(0)) {
			dprintk("%s: Failed to reset rate buffer\n", __func__);
			err = -EREMOTEIO;
			goto error;
		}
		
		video_show();

		/* empty event queue */
		vid->events.eventr = 0;
		vid->events.eventw = 0;
	}
	
	return 0;
	
error:
	dvb_generic_release(inode, file);
	
	return err;
}

int stbx25xx_video_release(struct inode *inode, struct file *file)
{
	struct dvb_device *dvbdev;
	struct stbx25xx_dvb_data *dvb;
	struct stbx25xx_video_data *vid;
	
	if(file == NULL)
		return -EINVAL;
	
	dvbdev = file->private_data;
	
	if(dvbdev == NULL)
		return -EINVAL;
	
	dvb = dvbdev->priv;
	
	if(dvb == NULL)
		return -EINVAL;
	
	vid = &dvb->video;
	
	if ((file->f_flags & O_ACCMODE) != O_RDONLY) {
		video_empty_buffers(vid);
		video_hide();
		video_stop(vid);
		video_set_source(vid, VIDEO_SOURCE_DEMUX);
//		video_vrb_reset(0);
		video_stop_proc();
	}

	return dvb_generic_release(inode, file);
}

unsigned int stbx25xx_video_poll(struct file *file, poll_table *wait)
{
	unsigned int mask = 0;
	struct dvb_device *dvbdev;
	struct stbx25xx_dvb_data *dvb;
	struct stbx25xx_video_data *vid;
	
	if(file == NULL)
		return -EINVAL;
	
	dvbdev = file->private_data;
	
	if(dvbdev == NULL)
		return -EINVAL;
	
	dvb = dvbdev->priv;
	
	if(dvb == NULL)
		return -EINVAL;
	
	vid = &dvb->video;

	if ((file->f_flags & O_ACCMODE) != O_RDONLY)
		poll_wait(file, &vid->write_wq, wait);

	poll_wait(file, &vid->events.wait_queue, wait);

	if (vid->events.eventw != vid->events.eventr)
		mask |= POLLPRI;

	if ((file->f_flags & O_ACCMODE) != O_RDONLY) {
		if (vid->state.play_state == VIDEO_PLAYING) {
			if (video_clip_free(vid))
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
	struct stbx25xx_dvb_data *dvb;
	struct stbx25xx_video_data *vid;
	
	if(file == NULL)
		return -EINVAL;
	
	dvbdev = file->private_data;
	
	if(dvbdev == NULL)
		return -EINVAL;
	
	dvb = dvbdev->priv;
	
	if(dvb == NULL)
		return -EINVAL;
	
	vid = &dvb->video;

	if (((file->f_flags & O_ACCMODE) == O_RDONLY) &&
		(cmd != VIDEO_GET_STATUS) &&
		(cmd != VIDEO_GET_EVENT) &&
		(cmd != VIDEO_GET_SIZE))
		return -EPERM;

	switch (cmd) {
	case VIDEO_STOP:
		return video_stop(vid);
		
	case VIDEO_FREEZE:
		return video_pause(vid);

	case VIDEO_PLAY:
		return video_play(vid);
		
	case VIDEO_CONTINUE:
		return video_resume(vid);

	case VIDEO_SELECT_SOURCE:
		return video_set_source(vid, (video_stream_source_t) arg);

	case VIDEO_SET_BLANK:
		vid->state.video_blank = !!arg;
		break;

	case VIDEO_GET_STATUS:
		memcpy(parg, &vid->state, sizeof(struct video_status));
		break;

	case VIDEO_GET_EVENT:
		return video_get_event(vid, parg, file->f_flags);

	case VIDEO_SET_DISPLAY_FORMAT:
		return video_set_display_format(vid, (video_displayformat_t) arg);

	case VIDEO_STILLPICTURE:
		return video_stillpicture(vid, parg);

	case VIDEO_FAST_FORWARD:
		return video_fastforward(vid, arg);

	case VIDEO_SLOWMOTION:
		return video_slow(vid, arg);

	case VIDEO_GET_CAPABILITIES:
		*((unsigned int *)parg) = STBx25xx_VIDEO_CAPS;
		break;

	case VIDEO_CLEAR_BUFFER:
		video_vfb_clear(vid);
		video_vrb_reset(0);
		break;

	case VIDEO_SET_ID:
		return -EOPNOTSUPP;

	case VIDEO_SET_STREAMTYPE:
		return -EOPNOTSUPP;

	case VIDEO_SET_FORMAT:
		return video_set_format(vid, (video_format_t) arg);

	case VIDEO_SET_SYSTEM:
		return video_set_system(vid, (video_system_t) arg);

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
		video_get_size(vid, (video_size_t *)parg);
		break;
		
	case STB_VID_SETAF:
		video_set_anti_flicker(vid, arg);
		break;

	default:
		return -ENOIOCTLCMD;
	}

	return 0;
}

/**
	Module init/exit
*/
int stbx25xx_video_init(struct stbx25xx_dvb_data *dvb)
{
	int ret, i;
	struct stbx25xx_video_data *vid = &dvb->video;
	
	printk(KERN_INFO "--- STBx25xx MPEG-2 Video Decoder driver ---\n");
	
	vid->state.video_blank = 0;
	vid->state.play_state = VIDEO_STOPPED;
	vid->state.stream_source = VIDEO_SOURCE_DEMUX;
	vid->state.video_format = VIDEO_FORMAT_4_3;
	vid->state.display_format = VIDEO_CENTER_CUT_OUT;
	init_waitqueue_head(&vid->write_wq);
	init_waitqueue_head(&vid->events.wait_queue);
	spin_lock_init(&vid->events.lock);
	vid->events.eventw = 0;
	vid->events.eventr = 0;
	vid->events.overflow = 0;
	mutex_init(&vid->osd_mode_mutex);
	init_waitqueue_head(&vid->clip_avail);
	vid->clip_rptr = 0;
	vid->clip_wptr = 0;
	init_completion(&vid->still_done);
	vid->still_mode = 0;
#ifndef CONFIG_STBx25xx_NTSC
	vid->system = VIDEO_SYSTEM_PAL;
#else
	vid->system = VIDEO_SYSTEM_NTSC;
#endif
	
	memset(irq_stats, 0, sizeof(u32) * STBx25xx_VIDEO_IRQ_COUNT);
	video_int_mask = IRQ_BIT(VIDEO_BRC_IRQ) | IRQ_BIT(VIDEO_ACCC_IRQ) | IRQ_BIT(VIDEO_SERR_IRQ) | 
			IRQ_BIT(VIDEO_PRC_IRQ) | IRQ_BIT(VIDEO_PLBME_IRQ) | IRQ_BIT(VIDEO_PSKIP_IRQ) |
			IRQ_BIT(VIDEO_BMC_IRQ) | IRQ_BIT(VIDEO_TBC_IRQ) | IRQ_BIT(VIDEO_VRBO_IRQ);
	
	for(i = 0; i < STBx25xx_VIDEO_IRQ_COUNT; i++)
		video_install_int_handler(i, dummy_int_handler);
	
	video_install_int_handler(VIDEO_STC_IRQ, video_stc_ready_handler);
	
	video_init_hw(vid);
	
	if((ret = request_irq(dvb->irq_num[STBx25xx_IRQ_VIDEO], video_interrupt, IRQF_TRIGGER_HIGH, "video", vid)) != 0) {
		err("failed to request video irq: error %d", ret);
		goto err_irq;
	}
	
	if((ret = video_init_memory(vid)) != 0) {
		err("memory initialization failed.");
		goto err_mem;
	}
	
	if((ret = video_init_firmware(vid)) != 0) {
		err("firmware initialization failed.");
		goto err_fw;
	}

	video_init_mem_pointers(vid);
		
#if defined(CONFIG_DVB_STBx25xx_OSD)
	stbx25xx_osd_init(dvb);
#endif

	video_init_procfs();
	video_set_source(vid, vid->state.stream_source);
	
	video_restart_proc();
	video_update_hw_config(vid);
	video_stop(vid);
	video_stop_proc();
	
	return 0;

err_fw:
	video_deinit_memory(vid);
err_mem:
	free_irq(dvb->irq_num[STBx25xx_IRQ_VIDEO], vid);
err_irq:
	return ret;
}

void stbx25xx_video_exit(struct stbx25xx_dvb_data *dvb)
{
	struct stbx25xx_video_data *vid = &dvb->video;

	video_deinit_procfs();

#if defined(CONFIG_DVB_STBx25xx_OSD)
	stbx25xx_osd_exit(dvb);
#endif
	
	video_hide();
	video_stop_decoding();
	video_stop_proc();
	free_irq(dvb->irq_num[STBx25xx_IRQ_VIDEO], vid);
	set_video_reg_raw(VIDEO_CNTL, 0);
	video_deinit_memory(vid);
}
