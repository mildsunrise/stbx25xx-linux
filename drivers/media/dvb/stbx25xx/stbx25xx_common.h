/*
 * This file is part of linux driver the digital TV devices equipped with IBM STBx25xx SoC
 *
 * stbx25xx-common.h - common header file for device-specific source files.
 *
 * see stbx25xx.c for copyright information.
 */

#ifndef __STBx25xx_COMMON_H__
#define __STBx25xx_COMMON_H__

#define DRIVER_NAME "IBM STBx25xx digital TV hardware driver"
#define DRIVER_AUTHOR "Tomasz Figa <tomasz.figa@gmail.com>"

#include <linux/list.h>
#include <linux/dvb/video.h>
#include <linux/dvb/audio.h>
#include "dmxdev.h"
#include "dvb_demux.h"
#include "dvb_filter.h"
#include "dvb_net.h"
#include "dvb_frontend.h"

#define STBx25xx_MAX_FEED		30
#define STBx25xx_QUEUE_COUNT		32
#define STBx25xx_FILTER_BLOCK_COUNT	64
#define STBx25xx_DEMUX1_IRQ_COUNT	15
#define STBx25xx_FB_COUNT		2
#define STBx25xx_LOG_PREFIX	"dvb-stbx25xx"

/* Stolen from usb.h */
#undef err
#define err(format,  arg...) printk(KERN_ERR     STBx25xx_LOG_PREFIX ": " format "\n" , ## arg)
#undef info
#define info(format, arg...) printk(KERN_INFO    STBx25xx_LOG_PREFIX ": " format "\n" , ## arg)
#undef warn
#define warn(format, arg...) printk(KERN_WARNING STBx25xx_LOG_PREFIX ": " format "\n" , ## arg)

/* video MPEG decoder events: */
/* (code copied from dvb_frontend.c, should maybe be factored out...) */
#define MAX_VIDEO_EVENT 8
struct dvb_video_events {
	struct video_event	  events[MAX_VIDEO_EVENT];
	int			  eventw;
	int			  eventr;
	int			  overflow;
	wait_queue_head_t	  wait_queue;
	spinlock_t		  lock;
};

/* STBx25xx DVB device helper structure */
struct stbx25xx_dvb_dev {
	/* general */
	struct device *dev; /* for firmware_class */
	
	/* interrupts */
#define STBx25xx_IRQ_DEMUX	0
#define STBx25xx_IRQ_AUDIO	1
#define STBx25xx_IRQ_VIDEO	2
#define STBx25xx_IRQ_TSDMA	3
	int irq_num[4];

#define FC_STATE_DVB_INIT 0x01
#define FC_STATE_I2C_INIT 0x02
#define FC_STATE_FE_INIT  0x04
	int init_state;

	/* dvb stuff */
	struct i2c_adapter *i2c;
	struct dvb_adapter dvb_adapter;
	struct dvb_frontend *fe;
	struct dvb_net dvbnet;
	struct dvb_demux demux;
	struct dmxdev dmxdev;
	struct dmx_frontend hw_frontend;
	struct dmx_frontend mem_frontend;
//	int (*fe_sleep) (struct dvb_frontend *);

	struct module *owner;

//	/* options and status */
//	int extra_feedcount;
//	int feedcount;
	
	/* AV stuff */
	struct dvb_device *audio;
	struct dvb_device *video;
	struct audio_status aud_state;
	struct video_status vid_state;
	wait_queue_head_t aud_write_wq;
	wait_queue_head_t vid_write_wq;
	struct dvb_video_events vid_events;
	video_system_t vid_system;
	
	/* Video decoder */
	void *vfb_memory;
	void *mpeg_memory;
	void *user_data;
	void *vbi0_data;
	void *vbi1_data;
	void *rb_data;
	phys_addr_t clip_phys[2];
	void *clip_data[2];
	ssize_t clip_size[2];
	u32 vfb_data[6];
	u32 vfb_size[6];
	
	/* Audio decoder */

	/* Framebuffer */
	struct fb_info *fb_info[STBx25xx_FB_COUNT];
	void *osd_memory;
	void *osdg_data;
	void *osdi_data;
	void *osdc_data;
	struct mutex osd_mode_mutex;
};

/* Function prototypes */
extern u32 stbx25xx_demux_check_crc32(struct dvb_demux_feed *feed,
			    const u8 *buf, size_t len);
extern void stbx25xx_demux_memcopy(struct dvb_demux_feed *feed, u8 *dst,
			 const u8 *src, size_t len);
extern int stbx25xx_demux_connect_frontend(struct dmx_demux* demux,
				 struct dmx_frontend* frontend);
extern int stbx25xx_demux_disconnect_frontend(struct dmx_demux* demux);
extern int stbx25xx_demux_get_stc(struct dmx_demux* demux, unsigned int num,
			u64 *stc, unsigned int *base);
extern void stbx25xx_demux_before_after_tune(fe_status_t fe_status, void *data);
extern int stbx25xx_demux_start_feed(struct dvb_demux_feed *feed);
extern int stbx25xx_demux_stop_feed(struct dvb_demux_feed *feed);
extern int stbx25xx_demux_write_to_decoder(struct dvb_demux_feed *feed,
				 const u8 *buf, size_t len);
				 
extern ssize_t stbx25xx_video_write(struct file *file, const char *buf, size_t count, loff_t *ppos);
extern int stbx25xx_video_open(struct inode *inode, struct file *file);
extern int stbx25xx_video_release(struct inode *inode, struct file *file);
extern unsigned int stbx25xx_video_poll(struct file *file, poll_table *wait);
extern int stbx25xx_video_ioctl(struct inode *inode, struct file *file, unsigned int cmd, void *parg);

extern ssize_t stbx25xx_audio_write(struct file *file, const char *buf, size_t count, loff_t *ppos);
extern int stbx25xx_audio_open(struct inode *inode, struct file *file);
extern int stbx25xx_audio_release(struct inode *inode, struct file *file);
extern unsigned int stbx25xx_audio_poll(struct file *file, poll_table *wait);
extern int stbx25xx_audio_ioctl(struct inode *inode, struct file *file, unsigned int cmd, void *parg);
 
extern int stbx25xx_video_init(struct stbx25xx_dvb_dev *);
extern int stbx25xx_audio_init(struct stbx25xx_dvb_dev *);
extern int stbx25xx_demux_init(struct stbx25xx_dvb_dev *);
extern int stbx25xx_frontend_init(struct stbx25xx_dvb_dev *);
extern int stbx25xx_osd_init(struct stbx25xx_dvb_dev *);

extern void stbx25xx_video_exit(struct stbx25xx_dvb_dev *);
extern void stbx25xx_audio_exit(struct stbx25xx_dvb_dev *);
extern void stbx25xx_demux_exit(struct stbx25xx_dvb_dev *);
extern void stbx25xx_frontend_exit(struct stbx25xx_dvb_dev *);
extern void stbx25xx_osd_exit(struct stbx25xx_dvb_dev *);

#endif
