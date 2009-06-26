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

#define DEBUG 10
#define DBG_LEVEL 10

#include "stbx25xx.h"
#include "stbx25xx_audio.h"
#include <linux/firmware.h>
#include <linux/proc_fs.h>
#include <linux/kthread.h>

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
	"Reserved 10",
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
	"Reserved 28",
	"Reserved 29",
	"Audio clip mode 2",
	"Audio clip mode",
};

static irqreturn_t audio_interrupt(int irq, void *data)
{
	struct stbx25xx_audio_data *aud = data;
	u32 mask = get_audio_reg_raw(AUD_ISR) & audio_int_mask;
	int i;

	set_audio_reg_raw(AUD_ISR, mask);

	for(i = STBx25xx_AUDIO_IRQ_COUNT - 1; i >= 0 && mask; i--, mask >>= 1) {
		if(mask & 1) {
			irq_stats[i]++;
			if(audio_int_handlers[i])
				audio_int_handlers[i](aud, i);
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
	Clip mode
*/

static int audio_clip_buffers_free(struct stbx25xx_clip_dev *clip)
{
	unsigned long flags;
	int ret;

	local_irq_save(flags);

	if(clip->buf_r > clip->buf_w)
		ret = clip->buf_num - clip->buf_r + clip->buf_w;
	else
		ret = clip->buf_w - clip->buf_r;

	if(!ret && clip->buf_full)
		ret = clip->buf_num;

	local_irq_restore(flags);

	return ret;
}

static unsigned int audio_clip_get_buf_wait(struct stbx25xx_clip_dev *clip)
{
	unsigned long flags;
	unsigned int ret = 0;

	if(wait_event_killable(clip->buf_wait, audio_clip_buffers_free(clip)))
		return 0;

	local_irq_save(flags);

	if(audio_clip_buffers_free(clip) > 0) {
		ret = clip->buf_queue[clip->buf_r++];

		if(clip->buf_queue[clip->buf_r] == 0xffffffff)
			clip->buf_r = 0;

		if(clip->buf_r == clip->buf_w)
			clip->buf_full = 0;
	}

	local_irq_restore(flags);

	return ret;
}

static unsigned int audio_clip_get_buf_nowait(struct stbx25xx_clip_dev *clip)
{
	unsigned long flags;
	unsigned int ret = 0;

	local_irq_save(flags);

	if(audio_clip_buffers_free(clip) > 0) {
		ret = clip->buf_queue[clip->buf_r++];

		if(clip->buf_queue[clip->buf_r] == 0xffffffff)
			clip->buf_r = 0;

		if(clip->buf_r == clip->buf_w)
			clip->buf_full = 0;
	}

	local_irq_restore(flags);

	return ret;
}

static void audio_clip_put_buf(struct stbx25xx_clip_dev *clip, unsigned int buf)
{
	unsigned long flags;

	local_irq_save(flags);

	clip->buf_queue[clip->buf_w++] = buf;

	if(clip->buf_queue[clip->buf_w] == 0xffffffff)
		clip->buf_w = 0;

	if(clip->buf_r == clip->buf_w)
		clip->buf_full = 1;

	local_irq_restore(flags);

	wake_up(&clip->buf_wait);
}

static int audio_clip_clips_available(struct stbx25xx_clip_dev *clip)
{
	unsigned long flags;
	int ret;
	
//	dprintk("%s: clip_r = %u, clip_w = %u\n", __func__, clip->clip_r, clip->clip_w);

	local_irq_save(flags);

	if(clip->clip_r > clip->clip_w)
		ret = clip->clip_num - clip->clip_r + clip->clip_w;
	else
		ret = clip->clip_w - clip->clip_r;

	if(!ret && clip->clip_full)
		ret = clip->clip_num;

	local_irq_restore(flags);

	return ret;
}

static unsigned int audio_clip_get_clip_nowait(struct stbx25xx_clip_dev *clip)
{
	unsigned long flags;
	unsigned int ret = 0;

	local_irq_save(flags);

	if(audio_clip_clips_available(clip) > 0) {
		ret = clip->clip_queue[clip->clip_r++];

		if(clip->clip_r >= clip->clip_num)
			clip->clip_r = 0;

		if(clip->clip_r == clip->clip_w)
			clip->clip_full = 0;
	}

	local_irq_restore(flags);
	
//	dprintk("%s: clip_r = %u, clip_w = %u\n", __func__, clip->clip_r, clip->clip_w);

	return ret;
}

static unsigned int audio_clip_get_clip_wait(struct stbx25xx_clip_dev *clip)
{
	unsigned long flags;
	unsigned int ret = 0;

	if(wait_event_interruptible(clip->clip_wait, audio_clip_clips_available(clip) || kthread_should_stop()))
		return 0;

	local_irq_save(flags);

	if(audio_clip_clips_available(clip) > 0) {
		ret = clip->clip_queue[clip->clip_r++];

		if(clip->clip_r >= clip->clip_num)
			clip->clip_r = 0;

		if(clip->clip_r == clip->clip_w)
			clip->clip_full = 0;
	}

	local_irq_restore(flags);
	
//	dprintk("%s: clip_r = %u, clip_w = %u\n", __func__, clip->clip_r, clip->clip_w);

	return ret;
}

static void audio_clip_put_clip(struct stbx25xx_clip_dev *clip, unsigned int data)
{
	unsigned long flags;

	local_irq_save(flags);

	clip->clip_queue[clip->clip_w++] = data;

	if(clip->clip_w >= clip->clip_num)
		clip->clip_w = 0;

	if(clip->clip_r == clip->clip_w)
		clip->clip_full = 1;

	local_irq_restore(flags);
	
//	dprintk("%s: clip_r = %u, clip_w = %u\n", __func__, clip->clip_r, clip->clip_w);

	wake_up(&clip->clip_wait);
}

static int audio_block_valid(struct stbx25xx_clip_dev *clip)
{
	stbx25xx_audio_val reg;

	reg = get_audio_reg(clip->qlr);
	return (reg.qlr.bv != 0);
}

static int audio_stream_busy(struct stbx25xx_clip_dev *clip)
{
	stbx25xx_audio_val reg;

	reg = get_audio_reg(clip->qlr);
	return (reg.qlr.sb != 0);
}

static int audio_clip_queue(struct stbx25xx_clip_dev *clip, const char *buf, size_t count, int nonblocking, int user)
{
	size_t size, sent;
	unsigned int clip_buf;

	BUG_ON(in_irq());

	sent = 0;

	while(count) {
		if(nonblocking) {
			clip_buf = audio_clip_get_buf_nowait(clip);
			if(!clip_buf)
				return (sent) ? sent : -EAGAIN;
		} else {
			clip_buf = audio_clip_get_buf_wait(clip);
			if(!clip_buf)
				return -ENOMEM;
		}

		size = min(count, (size_t)AUDIO_CLIP_BLOCK_SIZE);

		if(user)
			copy_from_user((void *)clip_buf, &buf[sent], size);
		else
			memcpy((void *)clip_buf, &buf[sent], size);

		clip_buf |= size - 1;

		audio_clip_put_clip(clip, clip_buf);
		
		dprintk("%s: Queued clip %08x\n", __func__, clip_buf);

		sent += size;
		count -= size;
	}

	return sent;
}

static void audio_empty_clip_buffers(struct stbx25xx_clip_dev *clip)
{
	unsigned long flags;
	unsigned int clip_data;
	
	local_irq_save(flags);
	
	while((clip_data = audio_clip_get_clip_nowait(clip)) != 0) {
		clip_data &= ~(AUDIO_CLIP_BLOCK_SIZE - 1);
		audio_clip_put_buf(clip, clip_data);
	}
	
	local_irq_restore(flags);
	
	wait_event_interruptible(clip->done, !audio_block_valid(clip) && !audio_stream_busy(clip));
}

static int audio_clip_write(struct stbx25xx_clip_dev *clip, unsigned int addr, unsigned int len)
{
	stbx25xx_audio_val reg;
	unsigned long flags;
	unsigned int phys;

	if(wait_event_interruptible(clip->done, !audio_block_valid(clip) || kthread_should_stop()) || audio_block_valid(clip))
		return -1;

	local_irq_save(flags);

	clip->cur_clips[clip->cur_w] = addr;
	clip->cur_w ^= 1;

	local_irq_restore(flags);
	
	phys = addr - (u32)clip->memory + clip->phys;

	set_audio_reg_raw(clip->qar, phys);

	reg.raw = 0;
	reg.qlr.bv = 0;
	reg.qlr.len = len;
	set_audio_reg(clip->qlr, reg);
	reg.qlr.bv = 1;
	set_audio_reg(clip->qlr, reg);
	
//	dprintk("%s: Added clip %u@%08x to hardware queue\n", __func__, len, phys);
	dprintk("%s: DSP Status = 0x%08x\n", __func__, get_audio_reg_raw(AUD_DSP_STAT));
	dprintk("%s: Decoder Status = 0x%08x\n", __func__, get_audio_reg_raw(AUD_DSR));

	return 0;
}

static int audio_clip_thread(void *data)
{
	struct stbx25xx_clip_dev *clip = data;
	unsigned int clip_data;

	while(!kthread_should_stop()) {
		clip_data = audio_clip_get_clip_wait(clip);
		
		dprintk("%s: Received clip %08x\n", __func__, clip_data);

		if(!clip_data)
			break;

		if(audio_block_valid(clip) && kthread_should_stop())
			break;

		audio_clip_write(clip, clip_data & ~(AUDIO_CLIP_BLOCK_SIZE - 1), (clip_data & (AUDIO_CLIP_BLOCK_SIZE - 1)) + 1);
	}

	return 0;
}

static void audio_clip_flushed(struct stbx25xx_audio_data *aud, int irq)
{
	struct stbx25xx_clip_dev *clip = &aud->clip;
	
	wake_up(&aud->clip.flushed);
}

static void audio_clip_interrupt(struct stbx25xx_audio_data *aud, int irq)
{
	struct stbx25xx_clip_dev *clip = &aud->clip;
	unsigned long flags;

	local_irq_save(flags);
	
	audio_clip_put_buf(clip, clip->cur_clips[clip->cur_r]);
	clip->cur_r ^= 1;

	local_irq_restore(flags);
	
	wake_up(&aud->clip.done);
}

static void audio_init_clip_queues(struct stbx25xx_clip_dev *clip)
{
	unsigned int addr;
	int i;
	
	clip->buf_r = 0;
	clip->buf_w = 0;
	clip->buf_queue[clip->buf_num] = 0xffffffff;
	clip->buf_full = 1;

	addr = (unsigned int) clip->memory;
	for(i = 0; i < clip->buf_num; i++, addr += AUDIO_CLIP_BLOCK_SIZE)
		clip->buf_queue[i] = addr;

	clip->clip_r = 0;
	clip->clip_w = 0;
	clip->clip_queue[clip->clip_num] = 0xffffffff;
	clip->clip_full = 0;

	memset(clip->clip_queue, 0, clip->clip_num * sizeof(unsigned int));
}

/**
	Utility functions
*/

static int audio_generate_tone(struct stbx25xx_audio_data *aud, unsigned int tone_ctrl)
{
	set_audio_reg_raw(AUD_TONE_GEN, tone_ctrl);
	
	return 0;
}

static void audio_soft_reset(void)
{
	set_audio_reg_raw(AUD_CMD, 0);
	
	udelay(1);
}

static void audio_rb_reset(void)
{
	set_audio_reg_raw(AUD_CMD, 1);

	udelay(1);
}

static int audio_stop(struct stbx25xx_audio_data *aud)
{
	stbx25xx_audio_val reg;

	reg = get_audio_reg(AUD_CTRL2);
	reg.ctrl2.hd = 1;
	set_audio_reg(AUD_CTRL2, reg);

	return 0;
}

static int audio_play(struct stbx25xx_audio_data *aud)
{
	stbx25xx_audio_val reg;

	reg = get_audio_reg(AUD_CTRL2);
	reg.ctrl2.hd = 0;
	set_audio_reg(AUD_CTRL2, reg);
	
	return 0;
}

static int audio_set_source(struct stbx25xx_audio_data *aud, audio_stream_source_t source)
{
	stbx25xx_audio_val reg;

	reg = get_audio_reg(AUD_CTRL0);
	reg.ctrl0.cm = (source == AUDIO_SOURCE_MEMORY);
	set_audio_reg(AUD_CTRL0, reg);

	aud->state.stream_source = source;
	
	return 0;
}

static int audio_set_mute(struct stbx25xx_audio_data *aud, int mute)
{
	stbx25xx_audio_val reg;

	reg = get_audio_reg(AUD_CTRL1);
	reg.ctrl1.sm = !!mute;
	set_audio_reg(AUD_CTRL1, reg);
	
	reg = get_audio_reg(AUD_CTRL2);
	reg.ctrl2.mute = !!mute;
	set_audio_reg(AUD_CTRL2, reg);

	aud->state.mute_state = !!mute;
	
	return 0;
}

static int audio_set_sync(struct stbx25xx_audio_data *aud, int sync)
{
	stbx25xx_audio_val reg;

	reg = get_audio_reg(AUD_CTRL0);
	reg.ctrl0.as = !!sync;
	set_audio_reg(AUD_CTRL0, reg);

	aud->state.AV_sync_state = !!sync;

	return 0;
}

static int audio_set_bypass(struct stbx25xx_audio_data *aud, int bypass)
{
	stbx25xx_audio_val reg;

	if(!(aud->stream_type & (AUDIO_CAP_MP1 | AUDIO_CAP_MP2 | AUDIO_CAP_MP3 | AUDIO_CAP_AC3)))
		return -EINVAL;

	reg = get_audio_reg(AUD_CTRL2);
	reg.ctrl2.id = !!bypass;
	set_audio_reg(AUD_CTRL2, reg);

	aud->state.bypass_mode = !!bypass;
	
	return 0;
}

static int audio_channel_sel(struct stbx25xx_audio_data *aud, audio_channel_select_t channel)
{
	stbx25xx_audio_val reg;

	reg = get_audio_reg(AUD_CTRL1);

	switch(channel) {
	case AUDIO_STEREO:
		reg.ctrl1.drm = 0;
		break;
	case AUDIO_MONO_LEFT:
		reg.ctrl1.drm = 1;
		break;
	case AUDIO_MONO_RIGHT:
		reg.ctrl1.drm = 2;
		break;
	case AUDIO_MONO:
		reg.ctrl1.drm = 3;
		break;
	default:
		return -EINVAL;
	}

	set_audio_reg(AUD_CTRL1, reg);
	aud->state.channel_select = channel;

	return 0;
}

static int audio_set_stream(struct stbx25xx_audio_data *aud, unsigned int stream_id)
{
	stbx25xx_audio_val reg;

	reg.raw = 0;
	reg.stream_id.stmie = 0xff;
	reg.stream_id.stmm = stream_id & 0xff;
	set_audio_reg(AUD_STREAM_ID, reg);

	return 0;
}

static int audio_set_volume(struct stbx25xx_audio_data *aud, audio_mixer_t *mixer)
{
	stbx25xx_audio_val reg;

	memcpy(&aud->state.mixer_state, mixer, sizeof(audio_mixer_t));

	if(aud->state.mixer_state.volume_left > 255)
		aud->state.mixer_state.volume_left = 255;

	if(aud->state.mixer_state.volume_right > 255)
		aud->state.mixer_state.volume_right = 255;

	reg = get_audio_reg(AUD_ATT_FT);
	reg.att.left = 63 - (aud->state.mixer_state.volume_left >> 2);
	reg.att.right = 63 - (aud->state.mixer_state.volume_right >> 2);
	set_audio_reg(AUD_ATT_FT, reg);
	set_audio_reg(AUD_ATT_RR, reg);

	reg.att.left = (reg.att.left + reg.att.right) / 2;
	reg.att.right = min(63 - (aud->state.mixer_state.volume_left >> 2), 63 - (aud->state.mixer_state.volume_right >> 2));
	set_audio_reg(AUD_ATT_CR, reg);
	
	return 0;
}

static int audio_set_stream_type(struct stbx25xx_audio_data *aud, unsigned int type)
{
	stbx25xx_audio_val reg;
	unsigned int id;
	unsigned int container;
	
	if(aud->state.play_state != AUDIO_STOPPED)
		return -EBUSY;

	reg = get_audio_reg(AUD_CTRL2);
	reg.ctrl2.prog3.raw = 0;

	switch(type & ~0xc0000000) {
	case AUDIO_CAP_MP1:
	case AUDIO_CAP_MP2:
	case AUDIO_CAP_MP3:
		reg.ctrl2.id = 0;
		reg.ctrl2.strmtp = AUD_STRMTP_MPEG;
		id = 0xe0c0;
		break;
	case AUDIO_CAP_AC3:
		reg.ctrl2.id = 1;
		reg.ctrl2.strmtp = AUD_STRMTP_AC3;
		id = 0xffbd;
		break;
	default:
		return -EINVAL;
	}

	set_audio_reg(AUD_CTRL2, reg);
	
	audio_rb_reset();
	
	container = (type & 0xc0000000) ? type >> 30 : AUD_TYPE_PES;
	
	reg = get_audio_reg(AUD_CTRL0);
	reg.ctrl0.type = container;
	set_audio_reg(AUD_CTRL0, reg);
	
	if(container == AUD_TYPE_PES)
		set_audio_reg_raw(AUD_STREAM_ID, id);
	else
		set_audio_reg_raw(AUD_STREAM_ID, 0);
	
	audio_rb_reset();
	
	aud->stream_type = type;
	
	return 0;
}

static void audio_dac_enable(void)
{
	stbx25xx_audio_val reg;

	reg = get_audio_reg(AUD_CTRL1);
	reg.ctrl1.dce = 1;
	set_audio_reg(AUD_CTRL1, reg);
}

static void audio_dac_disable(void)
{
	stbx25xx_audio_val reg;

	reg = get_audio_reg(AUD_CTRL1);
	reg.ctrl1.dce = 0;
	set_audio_reg(AUD_CTRL1, reg);
}

static void audio_dsp_start(void)
{
	stbx25xx_audio_val reg;

	reg = get_audio_reg(AUD_CTRL0);
	reg.ctrl0.adiso = 1;
	reg.ctrl0.ep = 1;
	reg.ctrl0.vmd = 1;
	reg.ctrl0.ei = 1;
	set_audio_reg(AUD_CTRL0, reg);
}

static void audio_dsp_stop(void)
{
	stbx25xx_audio_val reg;

	reg = get_audio_reg(AUD_CTRL0);
	reg.ctrl0.adiso = 0;
	reg.ctrl0.ep = 0;
	reg.ctrl0.vmd = 0;
	reg.ctrl0.ei = 0;
	set_audio_reg(AUD_CTRL0, reg);
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
	memset(irq_stats, 0, sizeof(u32) * STBx25xx_AUDIO_IRQ_COUNT);
	
	audio_proc_irqs = proc_create(AUDIO_PROC_IRQS_NAME, 0, stbx25xx_proc_dir, &proc_audio_iops);
}

static void audio_deinit_procfs(void)
{
	remove_proc_entry(AUDIO_PROC_IRQS_NAME, stbx25xx_proc_dir);
}

/**
	Hardware setup
*/
static int audio_clip_init(struct stbx25xx_clip_dev *clip, int mixer)
{
	int ret;
	
	if(mixer) {
		clip->qar = AUD_QAR2;
		clip->qlr = AUD_QLR2;
	} else {
		clip->qar = AUD_QAR;
		clip->qlr = AUD_QLR;
	}

	init_waitqueue_head(&clip->done);
	init_waitqueue_head(&clip->flushed);

	/* Item count */
	clip->buf_num = clip->size / AUDIO_CLIP_BLOCK_SIZE;
	clip->clip_num = clip->buf_num;
	dprintk("%s: Available %u %s clip buffers\n", __func__, clip->buf_num, (mixer) ? "mixer" : "main");

	/* Buffer queue */
	clip->buf_queue = kmalloc((clip->buf_num + 1) * sizeof(unsigned int), GFP_KERNEL);
	if(clip->buf_queue == NULL) {
		err("could not allocate memory for clip mode buffer queue");
		ret = -ENOMEM;
		goto err_bqueue;
	}

	init_waitqueue_head(&clip->buf_wait);

	/* Clip queue */
	clip->clip_queue = kmalloc((clip->clip_num + 1) * sizeof(unsigned int), GFP_KERNEL);
	if(clip->buf_queue == NULL) {
		err("could not allocate memory for clip mode clip queue");
		ret = -ENOMEM;
		goto err_cqueue;
	}

	audio_init_clip_queues(clip);

	init_waitqueue_head(&clip->clip_wait);

	clip->cur_r = 0;
	clip->cur_w = 0;

	audio_install_int_handler((mixer) ? AUDIO_CM2_IRQ : AUDIO_CM_IRQ, audio_clip_interrupt);
	
	clip->thread = kthread_run(audio_clip_thread, clip, "stbaudioclip%d", mixer);

	return 0;
	
err_cqueue:
	kfree(clip->buf_queue);
err_bqueue:
	return ret;
}

static void audio_clip_deinit(struct stbx25xx_clip_dev *clip)
{
	kthread_stop(clip->thread);
	kfree(clip->clip_queue);
	kfree(clip->buf_queue);
}

static int audio_memory_init(struct stbx25xx_audio_data *aud)
{
	stbx25xx_audio_val reg;

	aud->memory = ioremap_nocache(AUDIO_DATA_BASE, AUDIO_DATA_SIZE);
	if(aud->memory == NULL) {
		dprintk("%s: Could not remap audio memory\n", __func__);
		return -ENOMEM;
	}
	dprintk("%s: Audio memory - 0x%08x @ %p (%u bytes)\n", __func__, (u32)AUDIO_DATA_BASE, aud->memory, (u32)AUDIO_DATA_SIZE);

	set_audio_reg_raw(AUD_SEG1, SEG1_BASE >> AUDIO_SEGMENT_SHIFT);
	set_audio_reg_raw(AUD_SEG2, SEG2_BASE >> AUDIO_SEGMENT_SHIFT);
	set_audio_reg_raw(AUD_SEG3, (SEG3_BASE >> AUDIO_SEGMENT_SHIFT) | (((AUDIO_RB_SIZE / 4096 - 1) & 0xF) << 28));
	dprintk("%s: Audio segments: 0x%08x, 0x%08x, 0x%08x\n", __func__, 
		 (u32)get_audio_reg_raw(AUD_SEG1), (u32)get_audio_reg_raw(AUD_SEG2), (u32)get_audio_reg_raw(AUD_SEG3));

	reg.raw = 0;
	reg.offsets.dab2 = AUDIO_DAB2_OFFSET / 4096;
	reg.offsets.dab1 = AUDIO_DAB1_OFFSET / 4096;
	reg.offsets.awa = AUDIO_AWA_OFFSET / 4096;
	set_audio_reg(AUD_OFFS, reg);
	dprintk("%s: Audio segment 1 offsets: 0x%08x\n", __func__,
		 (u32)get_audio_reg_raw(AUD_OFFS));

	aud->clip.phys = AUDIO_DATA_BASE + AUDIO_CLIP_OFFSET;
	aud->clip.memory = aud->memory + AUDIO_CLIP_OFFSET;
	aud->clip.size = AUDIO_CLIP_SIZE / 2;
	aud->mixer.phys = AUDIO_DATA_BASE + AUDIO_CLIP_OFFSET + AUDIO_CLIP_SIZE / 2;
	aud->mixer.memory = aud->memory + AUDIO_CLIP_OFFSET + AUDIO_CLIP_SIZE / 2;
	aud->mixer.size = AUDIO_CLIP_SIZE / 2;
	dprintk("%s: Clip memory - 0x%08x @ %p (%u bytes)\n", __func__, aud->clip.phys, aud->clip.memory, aud->clip.size);
	dprintk("%s: Mixer memory - 0x%08x @ %p (%u bytes)\n", __func__, aud->mixer.phys, aud->mixer.memory, aud->mixer.size);

	return 0;
}

static void audio_memory_deinit(struct stbx25xx_audio_data *aud)
{
	aud->clip.memory = NULL;
	aud->clip.size = 0;
	aud->mixer.memory = NULL;
	aud->mixer.size = 0;
	
	iounmap(aud->memory);
	
	aud->memory = NULL;
}

static int audio_firmware_init(struct stbx25xx_audio_data *aud)
{
	struct stbx25xx_dvb_data *dvb = container_of(aud, struct stbx25xx_dvb_data, audio);
	const struct firmware *code;
	const struct firmware *data;
	stbx25xx_audio_val reg;
	int ret = 0, i, *ver;
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
	
	if ((code->data[248] != 0) || (code->data[249] != 0)) {
		ver = (int *)(&code->data[248]);
		info("audio firmware **test version** built on %d/%d/%d", 
			(*ver%10000)/100, *ver%100, *ver/10000);
	} else {
		info("audio firmware release %d version %d",
			code->data[250], code->data[251]);
	}

	/* upload the firmware */
	get_audio_reg_raw(AUD_CTRL0);
	
	reg = get_audio_reg(AUD_CTRL0);
	reg.ctrl0.vmd = 0;
	reg.ctrl0.md = 0;
	set_audio_reg(AUD_CTRL0, reg);
	
	reg.ctrl0.md = 1;
	set_audio_reg(AUD_CTRL0, reg);

	for(i = 0; i < code->size / 4; i++)
		set_audio_reg_raw(AUD_MDR, ((unsigned long *)code->data)[i]);

	reg.ctrl0.md = 0;
	set_audio_reg(AUD_CTRL0, reg);
	
	reg.ctrl0.vmd = 1;
	set_audio_reg(AUD_CTRL0, reg);

#if 0
	/* verify the firmware */
	reg.ctrl0.md = 1;
	set_audio_reg(AUD_CTRL0, reg);

	for(i = 0; i < code->size; i += 4) {
		word = (code->data[i] << 8) | code->data[i+1];
		if(get_audio_reg_raw(AUD_MDR) != word) {
			err("firmware verification failure at address %d!", i);
			ret = -EAGAIN;
			break;
		}
	}

	reg.ctrl0.md = 0;
	set_audio_reg(AUD_CTRL0, reg);
#endif

	info("audio firmware loaded, loading utility data");

	memcpy(aud->memory + AUDIO_AWA_OFFSET + 64, data->data, data->size);

	info("audio utility data loaded");

	/* release the firmware after uploading */
	release_firmware(code);
	release_firmware(data);

	return ret;
}

static void audio_hw_init(void)
{
	audio_soft_reset();
	
	set_audio_reg_raw(AUD_CTRL0, 0);
	set_audio_reg_raw(AUD_IMR, 0);
	
	audio_rb_reset();
}

static void audio_hw_configure(struct stbx25xx_audio_data *aud)
{
	set_audio_reg_raw(AUD_STREAM_ID, STREAM_ID_PES);
	set_audio_reg_raw(AUD_IMR, audio_int_mask);
	
	audio_dsp_start();

	set_audio_reg_raw(AUD_CTRL1, 0);
	audio_channel_sel(aud, aud->state.channel_select);
	
	set_audio_reg_raw(AUD_CTRL2, AUD_STRMTP_MPEG);
	audio_rb_reset();
	
	audio_set_stream_type(aud, aud->stream_type);
	audio_set_bypass(aud, aud->state.bypass_mode);
	audio_set_mute(aud, aud->state.mute_state);
	
	audio_rb_reset();

	audio_set_sync(aud, aud->state.AV_sync_state);
	audio_set_source(aud, aud->state.stream_source);

	audio_set_volume(aud, &aud->state.mixer_state);
	
	audio_rb_reset();
}

/**
	Exported API calls
*/
void stbx25xx_audio_sync_stc(u32 stcl, u32 stch)
{
	stbx25xx_audio_val reg;
	
	do {
		reg = get_audio_reg(AUD_DSR);
		if(reg.dsr.hv)
			get_audio_reg_raw(AUD_STC);
	} while(reg.dsr.hv);
	
	set_audio_reg_raw(AUD_STC, stch & 0xffff);
	set_audio_reg_raw(AUD_STC, stch >> 16);
	set_audio_reg_raw(AUD_STC, (stcl >> 9) & 0x1);
}

ssize_t stbx25xx_audio_write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
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

	if (((file->f_flags & O_ACCMODE) == O_RDONLY) ||
		(aud->state.stream_source != AUDIO_SOURCE_MEMORY))
			return -EPERM;

	return audio_clip_queue(&aud->clip, buf, count, file->f_flags & O_NONBLOCK, 1);
}

unsigned int stbx25xx_audio_poll(struct file *file, poll_table *wait)
{
	struct dvb_device *dvbdev;
	struct stbx25xx_dvb_data *dvb;
	struct stbx25xx_audio_data *aud;
	unsigned int mask = 0;

	if(file == NULL)
		return -EINVAL;

	dvbdev = file->private_data;

	if(dvbdev == NULL)
		return -EINVAL;

	dvb = dvbdev->priv;

	if(dvb == NULL)
		return -EINVAL;

	aud = &dvb->audio;

	poll_wait(file, &aud->write_wq, wait);

	if (aud->state.play_state == AUDIO_PLAYING) {
		if (audio_clip_buffers_free(&aud->clip) > 0)
			mask |= (POLLOUT | POLLWRNORM);
	} else {
		mask |= (POLLOUT | POLLWRNORM);
	}

	return mask;
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
		if(aud->state.stream_source == AUDIO_SOURCE_MEMORY)
			audio_empty_clip_buffers(&aud->clip);
		audio_stop(aud);
		aud->state.play_state = AUDIO_STOPPED;
		break;

	case AUDIO_PLAY:
		audio_rb_reset();
		audio_play(aud);
		aud->state.play_state = AUDIO_PLAYING;
		break;

	case AUDIO_PAUSE:
		audio_stop(aud);
		aud->state.play_state = AUDIO_PAUSED;
		break;

	case AUDIO_CONTINUE:
		audio_play(aud);
		aud->state.play_state = AUDIO_PLAYING;
		break;

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
		audio_rb_reset();
		break;

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
		
	case STB_AUD_TONEGEN:
		return audio_generate_tone(aud, arg);
		
	case STB_AUD_GET_STATUS:
		memcpy(parg, aud->memory + AUDIO_AWA_OFFSET, 64);
		break;

	default:
		return -ENOIOCTLCMD;
	}

	return 0;
}


int stbx25xx_audio_open(struct inode *inode, struct file *file)
{
	struct dvb_device *dvbdev;
	struct stbx25xx_dvb_data *dvb;
	struct stbx25xx_audio_data *aud;
	int err;

	if(file == NULL)
		return -EINVAL;

	dvbdev = file->private_data;

	if(dvbdev == NULL)
		return -EINVAL;

	dvb = dvbdev->priv;

	if(dvb == NULL)
		return -EINVAL;

	aud = &dvb->audio;

	if ((err = dvb_generic_open(inode, file)) < 0)
		return err;

	if((file->f_flags & O_ACCMODE) != O_RDONLY) {
		audio_dsp_start();
		audio_dac_enable();
		audio_rb_reset();
	}

	return 0;
}

int stbx25xx_audio_release(struct inode *inode, struct file *file)
{
	struct dvb_device *dvbdev;
	struct stbx25xx_dvb_data *dvb;
	struct stbx25xx_audio_data *aud;
	int err;

	if(file == NULL)
		return -EINVAL;

	dvbdev = file->private_data;

	if(dvbdev == NULL)
		return -EINVAL;

	dvb = dvbdev->priv;

	if(dvb == NULL)
		return -EINVAL;

	aud = &dvb->audio;
	
	if((file->f_flags & O_ACCMODE) != O_RDONLY) {
		if(aud->state.play_state == AUDIO_PLAYING)
			wait_event_interruptible(aud->clip.flushed, !audio_block_valid(&aud->clip) && !audio_stream_busy(&aud->clip));
		
		stbx25xx_audio_ioctl(inode, file, AUDIO_STOP, NULL);
		stbx25xx_audio_ioctl(inode, file, AUDIO_SELECT_SOURCE, AUDIO_SOURCE_DEMUX);
		audio_rb_reset();
		audio_dac_disable();
		audio_dsp_stop();
	}

	return dvb_generic_release(inode, file);
}

/**
	Module init/exit
*/
int stbx25xx_audio_init(struct stbx25xx_dvb_data *dvb)
{
	struct stbx25xx_audio_data *aud = &dvb->audio;
	int i, ret;

	printk(KERN_INFO "--- STBx25xx MPEG-2 Audio Decoder driver ---\n");

	aud->stream_type = AUDIO_CAP_MP2;
	aud->state.AV_sync_state = 0;
	aud->state.mute_state = 0;
	aud->state.play_state = AUDIO_STOPPED;
	aud->state.stream_source = AUDIO_SOURCE_DEMUX;
	aud->state.channel_select = AUDIO_STEREO;
	aud->state.bypass_mode = 1;
	aud->state.mixer_state.volume_left = 0;
	aud->state.mixer_state.volume_right = 0;
	init_waitqueue_head(&aud->write_wq);

	audio_init_procfs();

	audio_int_mask = IRQ_BIT(AUDIO_CCC_IRQ) | IRQ_BIT(AUDIO_RTBC_IRQ) |
			IRQ_BIT(AUDIO_BTE_IRQ) | IRQ_BIT(AUDIO_CM2_IRQ) |
			IRQ_BIT(AUDIO_CM_IRQ);

	for(i = 0; i < STBx25xx_AUDIO_IRQ_COUNT; i++)
		audio_install_int_handler(i, dummy_int_handler);
	
	audio_install_int_handler(AUDIO_BTE_IRQ, audio_clip_flushed);

	audio_hw_init();

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

	audio_hw_configure(aud);

	if((ret = audio_clip_init(&aud->clip, 0)) != 0) {
		err("main clip device initialization failed.");
		goto err_clip_main;
	}
	
	if((ret = audio_clip_init(&aud->mixer, 1)) != 0) {
		err("mixer clip device initialization failed.");
		goto err_clip_mixer;
	}
	
	audio_stop(aud);
	audio_dsp_stop();

	return 0;
	
err_clip_mixer:
	audio_clip_deinit(&aud->clip);
err_clip_main:
	audio_dsp_stop();
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

	audio_dac_disable();
	audio_dsp_stop();
	audio_clip_deinit(&aud->clip);
	audio_clip_deinit(&aud->mixer);
	audio_deinit_procfs();
	audio_memory_deinit(aud);
	free_irq(dvb->irq_num[STBx25xx_IRQ_AUDIO], aud);
}
