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
	return 0;
}

/**
	Module init/exit
*/
int stbx25xx_audio_init(struct stbx25xx_dvb_data *dvb)
{
	struct stbx25xx_audio_data *aud = &dvb->audio;
	
	aud->state.AV_sync_state = 0;
	aud->state.mute_state = 0;
	aud->state.play_state = AUDIO_STOPPED;
	aud->state.stream_source = AUDIO_SOURCE_DEMUX;
	aud->state.channel_select = AUDIO_STEREO;
	aud->state.bypass_mode = 1;
	aud->state.mixer_state.volume_left = 0;
	aud->state.mixer_state.volume_right = 0;
	init_waitqueue_head(&aud->write_wq);
	
	return 0;
}

void stbx25xx_audio_exit(struct stbx25xx_dvb_data *dvb)
{

}
