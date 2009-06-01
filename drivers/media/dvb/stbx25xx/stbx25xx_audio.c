/*
 * stbx25xx_audio.c - MPEG2 Audio Decoder driver
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

#include "stbx25xx.h"
#include "stbx25xx_audio.h"
#include <linux/firmware.h>
#include <linux/proc_fs.h>

/**
	Interrupts
*/
u32 audio_int_mask;
typedef void (*audio_irq_handler_t)(struct stbx25xx_audio_data *audio, int irq);
static audio_irq_handler_t audio_int_handlers[STBx25xx_AUDIO_IRQ_COUNT];
static u32 irq_stats[STBx25xx_AUDIO_IRQ_COUNT];

static char *irq_name[STBx25xx_AUDIO_IRQ_COUNT] = {
	"Reserved 0",
	"Reserved 1",
	"Reserved 2",
	"Reserved 3",
	"Reserved 4",
	"Reserved 5",
	"Reserved 6",
	"Reserved 7",
	"Reserved 8",
	"Reserved 9",
	"Reserved 10"
	"Reserved 11",
	"Stream synchronized",
	"Reserved 13",
	"Reserved 14",
	"Reserved 15",
	"Channel change complete",
	"Timebase change required",
	"Buffer transitioned to full",
	"Buffer transitioned to empty",
	"Audio master synchronization",
	"Program end",
	"Buffer empty",
	"Buffer full",
	"PLB slave error",
	"PLB timeout",
	"Ancillary data full",
	"Ancillary data",
	"Reserved 28"
	"Reserved 29"
	"Audio clip mode 2",
	"Audio clip mode",
};

static irqreturn_t audio_interrupt(int irq, void *data)
{
	struct stbx25xx_audio_data *vid = data;
	u32 mask = get_audio_reg_raw(AUD_ISR) & audio_int_mask;
	int i;

	set_audio_reg_raw(AUD_ISR, mask);

	for(i = STBx25xx_AUDIO_IRQ_COUNT - 1; i >= 0 && mask; i--, mask >>= 1) {
		if(mask & 1) {
			irq_stats[i]++;
			if(audio_int_handlers[i])
				audio_int_handlers[i](vid, i);
		}
	}

	return IRQ_HANDLED;
}

static void audio_install_int_handler(unsigned int irq, audio_irq_handler_t handler)
{
	unsigned long flags;

	if(irq >= STBx25xx_AUDIO_IRQ_COUNT)
		return;

	if(audio_int_handlers[irq])
		dprintk("%s: Replacing interrupt handler (IRQ %d)\n", __func__, irq);

	local_irq_save(flags);
	audio_int_handlers[irq] = handler;
	local_irq_restore(flags);
}

static void audio_remove_int_handler(int irq)
{
	unsigned long flags;

	if(irq >= STBx25xx_AUDIO_IRQ_COUNT)
		return;

	if(!audio_int_handlers[irq])
		dprintk("%s: Interrupt handler already free (IRQ %d)\n", __func__, irq);

	local_irq_save(flags);
	audio_int_handlers[irq] = NULL;
	local_irq_restore(flags);
}

static void dummy_int_handler(struct stbx25xx_audio_data *aud, int irq)
{
	info("Audio IRQ: %s interrupt", irq_name[irq]);
}

/**
	Utility functions
*/

static void audio_rb_reset(void)
{
	set_audio_reg_raw(AUD_CMD, 1);

	udelay(1);
}

static int audio_stop(struct stbx25xx_audio_data *aud)
{
	return 0;
}

static int audio_play(struct stbx25xx_audio_data *aud)
{
	return 0;
}

static int audio_pause(struct stbx25xx_audio_data *aud)
{
	return 0;
}

static int audio_resume(struct stbx25xx_audio_data *aud)
{
	return 0;
}

static int audio_set_source(struct stbx25xx_audio_data *aud, audio_stream_source_t source)
{
	return 0;
}

static int audio_set_mute(struct stbx25xx_audio_data *aud, int mute)
{
	return 0;
}

static int audio_set_sync(struct stbx25xx_audio_data *aud, int sync)
{
	return 0;
}

static int audio_set_bypass(struct stbx25xx_audio_data *aud, int bypass)
{
	return 0;
}

static int audio_channel_sel(struct stbx25xx_audio_data *aud, audio_channel_select_t channel)
{
	return 0;
}

static int audio_clear_buffer(struct stbx25xx_audio_data *aud)
{
	return 0;
}

static int audio_set_stream(struct stbx25xx_audio_data *aud, unsigned int stream_id)
{
	return 0;
}

static int audio_set_volume(struct stbx25xx_audio_data *aud, audio_mixer_t *mixer)
{
	return 0;
}

static int audio_set_stream_type(struct stbx25xx_audio_data *aud, int type)
{
	return 0;
}

/**
	procfs
*/

static int audio_show_irqs(struct seq_file *p, void *v)
{
	int i = *(loff_t *)v, j;
	unsigned long flags;
	
	local_irq_save(flags);
	j = irq_stats[i];
	local_irq_restore(flags);
	
	seq_printf(p, "%2d : %8d : %s\n", i, j, irq_name[i]);
	
	return 0;
}

static void *audio_seq_istart(struct seq_file *f, loff_t *pos)
{
	return (*pos < STBx25xx_AUDIO_IRQ_COUNT) ? pos : NULL;
}

static void *audio_seq_inext(struct seq_file *f, void *v, loff_t *pos)
{
	(*pos)++;
	if (*pos >= STBx25xx_AUDIO_IRQ_COUNT)
		return NULL;
	return pos;
}

static void audio_seq_istop(struct seq_file *f, void *v)
{
	/* Nothing to do */
}

static const struct seq_operations audio_seq_iops = {
	.start = audio_seq_istart,
	.next  = audio_seq_inext,
	.stop  = audio_seq_istop,
	.show  = audio_show_irqs
};

static int audio_iopen(struct inode *inode, struct file *file)
{
	return seq_open(file, &audio_seq_iops);
}

static const struct file_operations proc_audio_iops = {
	.open           = audio_iopen,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = seq_release,
};

static struct proc_dir_entry *audio_proc_irqs;

static void audio_init_procfs(void)
{
	audio_proc_irqs = proc_create(AUDIO_PROC_IRQS_NAME, 0, stbx25xx_proc_dir, &proc_audio_iops);
}

static void audio_deinit_procfs(void)
{
	remove_proc_entry(AUDIO_PROC_IRQS_NAME, stbx25xx_proc_dir);
}

/**
	Hardware setup
*/
static int audio_memory_init(struct stbx25xx_audio_data *aud)
{
	stbx25xx_audio_val reg;

	aud->memory = ioremap_nocache(AUDIO_DATA_BASE, AUDIO_DATA_SIZE);
	if(aud->memory == NULL) {
		dprintk("%s: Could not remap audio memory\n", __func__);
		return -ENOMEM;
	}

	set_audio_reg_raw(AUD_SEG1, SEG1_BASE >> AUDIO_SEGMENT_SHIFT);
	set_audio_reg_raw(AUD_SEG2, SEG2_BASE >> AUDIO_SEGMENT_SHIFT);
	set_audio_reg_raw(AUD_SEG3, (SEG3_BASE >> AUDIO_SEGMENT_SHIFT) | 0xF0000000);

	reg.raw = 0;
	reg.offsets.dab2 = AUDIO_DAB2_OFFSET / 4096;
	reg.offsets.dab1 = AUDIO_DAB1_OFFSET / 4096;
	reg.offsets.awa = AUDIO_AWA_OFFSET / 4096;
	set_audio_reg(AUD_OFFS, reg);

	aud->clip = aud->memory + AUDIO_CLIP_OFFSET;
	aud->clip_size = AUDIO_CLIP_SIZE / 2;
	aud->mixer = aud->memory + AUDIO_CLIP_OFFSET + AUDIO_CLIP_SIZE / 2;
	aud->mixer_size = AUDIO_CLIP_SIZE / 2;

	return 0;
}

static void audio_memory_deinit(struct stbx25xx_audio_data *aud)
{
	aud->clip = NULL;
	aud->clip_size = 0;
	aud->mixer = NULL;
	aud->mixer_size = 0;
	
	iounmap(aud->memory);
	
	aud->memory = NULL;
}

static int audio_firmware_init(struct stbx25xx_audio_data *aud)
{
	struct stbx25xx_dvb_data *dvb = container_of(aud, struct stbx25xx_dvb_data, audio);
	const struct firmware *code;
	const struct firmware *data;
	stbx25xx_audio_val reg;
	int ret = 0, i;
	u16 word;

	/* request the firmware, this will block until someone uploads it */
	ret = request_firmware(&code, "dvb-stbx25xx-aud-c.fw", dvb->dev);
	if (ret) {
		if (ret == -ENOENT) {
			err("could not load firmware,"
			       " file not found: dvb-stbx25xx-aud-c.fw");
			err("usually this should be in "
			       "/usr/lib/hotplug/firmware or /lib/firmware");
		} else {
			err("(cannot request firmware"
			       " (error %i)", ret);
		}
		return -EINVAL;
	}

	ret = request_firmware(&data, "dvb-stbx25xx-aud-d.fw", dvb->dev);
	if (ret) {
		if (ret == -ENOENT) {
			err("could not load firmware,"
			       " file not found: dvb-stbx25xx-aud-d.fw");
			err("usually this should be in "
			       "/usr/lib/hotplug/firmware or /lib/firmware");
		} else {
			err("(cannot request firmware"
			       " (error %i)", ret);
		}
		return -EINVAL;
	}

	/* upload the firmware */
	get_audio_reg_raw(AUD_CTRL0);
	
	reg = get_audio_reg(AUD_CTRL0);
	reg.ctrl0.vmd = 0;
	reg.ctrl0.md = 0;
	set_audio_reg(AUD_CTRL0, reg);
	
	reg.ctrl0.md = 1;
	set_audio_reg(AUD_CTRL0, reg);

	for(i = 0; i < code->size; i += 2) {
		word = (code->data[i] << 8) | code->data[i+1];
		set_audio_reg_raw(AUD_MDR, word);
	}

	reg.ctrl0.md = 0;
	set_audio_reg(AUD_CTRL0, reg);

	/* verify the firmware */
	reg.ctrl0.md = 1;
	set_audio_reg(AUD_CTRL0, reg);

	for(i = 0; i < code->size; i += 2) {
		word = (code->data[i] << 8) | code->data[i+1];
		if(get_audio_reg_raw(AUD_MDR) != word) {
			err("firmware verification failure at address %d!", i);
			ret = -EAGAIN;
			break;
		}
	}

	reg.ctrl0.md = 0;
	set_audio_reg(AUD_CTRL0, reg);

	info("audio firmware loaded, loading utility data");

	memcpy(aud->memory + AUDIO_AWA_OFFSET + 64, data->data, data->size);

	info("audio utility data loaded");

	/* release the firmware after uploading */
	release_firmware(code);
	release_firmware(data);

	return ret;
}

/**
	Exported API calls
*/
void stbx25xx_audio_sync_stc(u32 stcl, u32 stch)
{
	
}

ssize_t stbx25xx_audio_write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
	return 0;
}

int stbx25xx_audio_open(struct inode *inode, struct file *file)
{
	return 0;
}

int stbx25xx_audio_release(struct inode *inode, struct file *file)
{
	return 0;
}

unsigned int stbx25xx_audio_poll(struct file *file, poll_table *wait)
{
	return 0;
}

int stbx25xx_audio_ioctl(struct inode *inode, struct file *file, unsigned int cmd, void *parg)
{
	unsigned long arg = (unsigned long) parg;
	struct dvb_device *dvbdev;
	struct stbx25xx_dvb_data *dvb;
	struct stbx25xx_audio_data *aud;
	
	if(file == NULL)
		return -EINVAL;
	
	dvbdev = file->private_data;
	
	if(dvbdev == NULL)
		return -EINVAL;
	
	dvb = dvbdev->priv;
	
	if(dvb == NULL)
		return -EINVAL;
	
	aud = &dvb->audio;

	if (((file->f_flags & O_ACCMODE) == O_RDONLY) &&
		(cmd != AUDIO_GET_STATUS))
		return -EPERM;

	switch (cmd) {
	case AUDIO_STOP:
		return audio_stop(aud);

	case AUDIO_PLAY:
		return audio_play(aud);

	case AUDIO_PAUSE:
		return audio_pause(aud);

	case AUDIO_CONTINUE:
		return audio_resume(aud);

	case AUDIO_SELECT_SOURCE:
		return audio_set_source(aud, (audio_stream_source_t) arg);

	case AUDIO_SET_MUTE:
		return audio_set_mute(aud, arg);

	case AUDIO_SET_AV_SYNC:
		return audio_set_sync(aud, arg);

	case AUDIO_SET_BYPASS_MODE:
		return audio_set_bypass(aud, arg);

	case AUDIO_CHANNEL_SELECT:
		return audio_channel_sel(aud, (audio_channel_select_t) arg);

	case AUDIO_GET_STATUS:
		memcpy(parg, &aud->state, sizeof(struct audio_status));
		break;

	case AUDIO_GET_CAPABILITIES:
		*((unsigned int *)parg) = STBx25xx_AUDIO_CAPS;
		break;

	case AUDIO_CLEAR_BUFFER:
		return audio_clear_buffer(aud);

	case AUDIO_SET_ID:
		return audio_set_stream(aud, arg);

	case AUDIO_SET_MIXER:
		return audio_set_volume(aud, (audio_mixer_t *) parg);

	case AUDIO_SET_STREAMTYPE:
		return audio_set_stream_type(aud, arg);

	case AUDIO_SET_EXT_ID:
		return -EOPNOTSUPP;

	case AUDIO_SET_ATTRIBUTES:
		return -EOPNOTSUPP;

	case AUDIO_SET_KARAOKE:
		return -EOPNOTSUPP;

	default:
		return -ENOIOCTLCMD;
	}

	return 0;
}

/**
	Module init/exit
*/
int stbx25xx_audio_init(struct stbx25xx_dvb_data *dvb)
{
	struct stbx25xx_audio_data *aud = &dvb->audio;
	int i, ret;

	printk(KERN_INFO "--- STBx25xx MPEG-2 Audio Decoder driver ---\n");

	aud->state.AV_sync_state = 0;
	aud->state.mute_state = 0;
	aud->state.play_state = AUDIO_STOPPED;
	aud->state.stream_source = AUDIO_SOURCE_DEMUX;
	aud->state.channel_select = AUDIO_STEREO;
	aud->state.bypass_mode = 1;
	aud->state.mixer_state.volume_left = 0;
	aud->state.mixer_state.volume_right = 0;
	init_waitqueue_head(&aud->write_wq);

	memset(irq_stats, 0, sizeof(u32) * STBx25xx_AUDIO_IRQ_COUNT);
	audio_int_mask = 0;

	for(i = 0; i < STBx25xx_AUDIO_IRQ_COUNT; i++)
		audio_install_int_handler(i, dummy_int_handler);

	audio_rb_reset();

	if((ret = request_irq(dvb->irq_num[STBx25xx_IRQ_AUDIO], audio_interrupt, IRQF_TRIGGER_HIGH, "audio", aud)) != 0) {
		err("failed to request audio irq: error %d", ret);
		goto err_irq;
	}

	if((ret = audio_memory_init(aud)) != 0) {
		err("memory initialization failed.");
		goto err_mem;
	}

	if((ret = audio_firmware_init(aud)) != 0) {
		err("firmware initialization failed.");
		goto err_fw;
	}

	return 0;

err_fw:
	audio_memory_deinit(aud);
err_mem:
	free_irq(dvb->irq_num[STBx25xx_IRQ_AUDIO], aud);
err_irq:
	return ret;
}

void stbx25xx_audio_exit(struct stbx25xx_dvb_data *dvb)
{
	struct stbx25xx_audio_data *aud = &dvb->audio;
	
	audio_memory_deinit(aud);
	free_irq(dvb->irq_num[STBx25xx_IRQ_AUDIO], aud);
}
