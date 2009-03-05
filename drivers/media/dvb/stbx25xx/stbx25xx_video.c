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
 *
 * The driver relies on following assumptions:
 * - fb0 is a generic framebuffer running in 8-bit indexed mode
 * - fb1 is an extended hardware-specific framebuffer running in 16-bit direct color mode
 * - more to come...
 */

#define DEBUG		10
#define DBG_LEVEL	1

#include "stbx25xx.h"
#include "stbx25xx_video.h"
#include <linux/firmware.h>

static stbx25xx_video_val def_display_mode = {
	.disp_mode = {
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

static stbx25xx_video_val display_mode; /* Running parameters */

struct stbx25xx_video_cmd {
	u8 cmd;
	u8 cnt;
	u16 *par;
};

/**
	Hardware manipulation routines
*/
static void video_issue_cmd(struct stbx25xx_video_cmd *cmd)
{
	unsigned long flags;
	int i;
	
	local_irq_save(flags);
	
	while(get_video_reg_raw(CMD_STAT) & 1);
	
	set_video_reg_raw(CMD, cmd->cmd << 1);
	for(i=0; i<cmd->cnt; i++) {
		set_video_reg_raw(CMD_ADDR, i);
		set_video_reg_raw(CMD_DATA, cmd->par[i]);
	}
	set_video_reg_raw(CMD_STAT, 1);
	
	while(get_video_reg_raw(CMD_STAT) & 1);
	
	local_irq_restore(flags);
}

static void video_issue_cmds(struct stbx25xx_video_cmd *cmds, u8 count)
{
	unsigned long flags;
	int i, j;
	
	local_irq_save(flags);
	
	for(j = 0; j < count; j++) {
		while(get_video_reg_raw(CMD_STAT) & 1);
		
		set_video_reg_raw(CMD, cmds[j].cmd << 1);
		for(i = 0; i < cmds[j].cnt; i++) {
			set_video_reg_raw(CMD_ADDR, i);
			set_video_reg_raw(CMD_DATA, cmds[j].par[i]);
		}
		set_video_reg_raw(CMD_STAT, 1);
	}
	
	while(get_video_reg_raw(CMD_STAT) & 1);
	
	local_irq_restore(flags);
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

static void video_pause(struct stbx25xx_dvb_dev *dvb)
{
	
}

static void video_play(struct stbx25xx_dvb_dev *dvb)
{
	
}

static int video_set_source(struct stbx25xx_dvb_dev *dvb, video_stream_source_t src)
{
	return -EINVAL;
}

static int video_set_display_format(struct stbx25xx_dvb_dev *dvb, video_displayformat_t fmt)
{
	return -EINVAL;
}

static int video_stillpicture(struct stbx25xx_dvb_dev *dvb, struct video_still_picture *pic)
{
	return -EINVAL;
}

static int video_fastforward(struct stbx25xx_dvb_dev *dvb, int n)
{
	return -EINVAL;
}

static int video_slow(struct stbx25xx_dvb_dev *dvb, int n)
{
	return -EINVAL;
}

static void video_vrb_reset(struct stbx25xx_dvb_dev *dvb)
{

}

static int video_set_stream_type(struct stbx25xx_dvb_dev *dvb, int type)
{
	return -EINVAL;
}

static int video_set_format(struct stbx25xx_dvb_dev *dvb, video_format_t fmt)
{
	return -EINVAL;
}

static int video_set_system(struct stbx25xx_dvb_dev *dvb, video_system_t sys)
{
	return -EINVAL;
}

static void video_get_size(struct stbx25xx_dvb_dev *dvb, video_size_t *size)
{

}

static int video_clip_queue(struct stbx25xx_dvb_dev *dvb, const char *buf, size_t count, int nonblocking)
{
	return -EINVAL;
}

static int video_clip_free(struct stbx25xx_dvb_dev *dvb)
{
	return 0;
}

/**
	Hardware setup
*/
static void video_update_display_mode(void)
{
	set_video_reg(DISP_MODE, display_mode);
}

static void video_init_hw(struct stbx25xx_dvb_dev *dvb)
{
	stbx25xx_video_val reg;
	
	reg = get_video_reg(CICVCR);
	reg.cicvcr.viddis = 0;
	set_video_reg(CICVCR, reg);
	
	set_video_reg_raw(VIDEO_MODE, 0);
	
	display_mode = def_display_mode;
	video_update_display_mode();
	
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
	dvb->clip_phys[0] = (SEG1_ADDR + RB_OFFSET) + (dvb->clip_data[0] - dvb->rb_data);
	dvb->clip_phys[1] = dvb->clip_phys[0] + size;
	dvb->clip_size[0] = size;
	dvb->clip_size[1] = size;
	
	dprintk("%s: Clip mode ping-pong buffer - 0x%08x @ %p (Total size: %d bytes)\n", __func__, dvb->clip_phys[0], dvb->clip_data[0], dvb->clip_size[0] + dvb->clip_size[1]);
	
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
	set_video_reg_raw(PROC_IADDR, 0);
	
	for(i=0; i<8192; i+=2) {
		word = (fw->data[i] << 8) | fw->data[i+1];
		set_video_reg_raw(PROC_IDATA, word);
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
		video_pause(dvb);
		video_set_source(dvb, VIDEO_SOURCE_DEMUX);
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
		}
		else {
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
		video_pause(dvb);
		dvb->vid_state.play_state = VIDEO_STOPPED;
		break;
		
	case VIDEO_FREEZE:
		video_pause(dvb);
		dvb->vid_state.play_state = VIDEO_FREEZED;
		break;

	case VIDEO_PLAY:
		video_play(dvb);
		dvb->vid_state.play_state = VIDEO_PLAYING;
		break;
		
	case VIDEO_CONTINUE:
		video_play(dvb);
		dvb->vid_state.play_state = VIDEO_PLAYING;
		break;

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
		video_vrb_reset(dvb);
		break;

	case VIDEO_SET_ID:
		return -EOPNOTSUPP;

	case VIDEO_SET_STREAMTYPE:
		return video_set_stream_type(dvb, arg);

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
	int ret;
	u16 params = 0;
	struct stbx25xx_video_cmd cmd = {
		.cmd = CMD_CFG,	.cnt = 1, .par = &params,
	};
	
	printk(KERN_INFO "--- STBx25xx MPEG-2 Video Decoder driver ---\n");
	
	mutex_init(&dvb->osd_mode_mutex);
	
	video_init_hw(dvb);
	
	if((ret = video_init_memory(dvb)) != 0) {
		err("Memory initialization failed.");
		goto err_mem;
	}
	
	if((ret = video_init_firmware(dvb)) != 0) {
		err("Firmware initialization failed.");
		goto err_fw;
	}
	
	set_video_reg_raw(VIDEO_CNTL, 2);
	
	video_issue_cmd(&cmd);
	
#if defined(CONFIG_FB) || defined(CONFIG_FB_MODULE)
	stbx25xx_osd_init(dvb);
#endif
	
	return 0;
	
err_fw:
	video_deinit_memory(dvb);
err_mem:
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
