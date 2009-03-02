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
		.vpafc	= 1,
		.ilbm	= 1,
		.cds	= 1,
		.clbm	= 1,
		.afvp	= 1,
		.glbm	= 1,
	},
};

static stbx25xx_video_val display_mode; /* Running parameters */

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
	set_video_reg(MEM_SEG1, reg);
	
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
	dvb->osdc_data = dvb->osd_memory + FB1_OFFSET;
	dvb->osdi_data = dvb->osd_memory + CUR_OFFSET;
	
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
ssize_t stbx25xx_video_write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
	return 0;
}

int stbx25xx_video_open(struct inode *inode, struct file *file)
{
	return 0;
}

int stbx25xx_video_release(struct inode *inode, struct file *file)
{
	return 0;
}

unsigned int stbx25xx_video_poll(struct file *file, poll_table *wait)
{
	return 0;
}

int stbx25xx_video_ioctl(struct inode *inode, struct file *file, unsigned int cmd, void *parg)
{
	return 0;
}

/**
	Module init/exit
*/

struct stbx25xx_video_cmd {
	u8 cmd;
	u8 cnt;
	u16 *par;
};

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

int stbx25xx_video_init(struct stbx25xx_dvb_dev *dvb)
{
	int ret;
	u16 params = 0;
	struct stbx25xx_video_cmd cmd = {
		.cmd = CMD_CFG,	.cnt = 1, .par = &params,
	};
	
	printk(KERN_INFO "--- STBx25xx MPEG-2 Video Decoder driver ---\n");
	
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
	
#ifdef CONFIG_FB
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

#ifdef CONFIG_FB
	stbx25xx_osd_exit(dvb);
#endif
	
	set_video_reg_raw(VIDEO_CNTL, 0);
	video_deinit_memory(dvb);
}
