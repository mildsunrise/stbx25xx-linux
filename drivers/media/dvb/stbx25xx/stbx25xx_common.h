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
#include <linux/wait.h>
#include <linux/dvb/video.h>
#include <linux/dvb/audio.h>
#include "dmxdev.h"
#include "dvb_demux.h"
#include "dvb_filter.h"
#include "dvb_net.h"
#include "dvb_frontend.h"

#define STBx25xx_MAX_FEED		29
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

#ifdef CONFIG_DVB_STBx25xx_AV

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

struct stbx25xx_video_data {
	/* Video decoder */
	struct dvb_device *video;
	struct video_status state;
	wait_queue_head_t write_wq;
	struct dvb_video_events events;
	video_system_t system;
	u32 sfm;
	
	/* Video Framebuffer */
	void *vfb_memory;
	u32 vfb_data[6];
	u32 vfb_size[6];
	
	/* MPEG data */
	void *mpeg_memory;
	u8 *user_data;
	void *vbi0_data;
	void *vbi1_data;
	void *rb_data;
	
	/* Clip mode */
	phys_addr_t clip_phys[2];
	void *clip_data[2];
	ssize_t clip_size[2];
	volatile u8 clip_busy[2];
	u8 clip_rptr;
	u8 clip_wptr;
	wait_queue_head_t clip_avail;
	
	/* Still image */
	video_stream_source_t old_source;
	u8 still_mode;
	struct completion still_done;
	
	/* OSD */
	struct fb_info *fb_info[STBx25xx_FB_COUNT];
	void *osd_memory;
	void *osdg_data;
	void *osdi_data;
	void *osdc_data;
	struct mutex osd_mode_mutex;
};

struct stbx25xx_clip_dev {
	/* Memory */
	void *memory;
	size_t size;
	phys_addr_t phys;

	/* Registers */
	unsigned int qar;
	unsigned int qlr;

	/* Buffer queue */
	volatile unsigned int *buf_queue;
	volatile unsigned int buf_r;
	volatile unsigned int buf_w;
	unsigned int buf_num;
	volatile unsigned int buf_full;
	wait_queue_head_t buf_wait;

	/* Clip queue */
	volatile unsigned int *clip_queue;
	volatile unsigned int clip_r;
	volatile unsigned int clip_w;
	unsigned int clip_num;
	volatile unsigned int clip_full;
	wait_queue_head_t clip_wait;

	/* Kernel thread */
	struct task_struct *thread;
	wait_queue_head_t done;
	wait_queue_head_t flushed;
	volatile unsigned int cur_clips[2];
	volatile unsigned int cur_w;
	volatile unsigned int cur_r;
};

struct stbx25xx_audio_data {
	/* Audio decoder */
	struct dvb_device *audio;
	struct audio_status state;
	wait_queue_head_t write_wq;
	int stream_type;

	/* Audio memory */
	void *memory;

	/* Main clip device */
	struct stbx25xx_clip_dev clip;

	/* Mixer clip device */
	struct stbx25xx_clip_dev mixer;
};

#endif

struct stbx25xx_demux_data {
	/* Demux */
	struct dvb_demux 	demux;
	struct dmxdev 		dmxdev;
	struct dmx_frontend 	hw_frontend;
	struct dmx_frontend 	mem_frontend;
	
	/* Clock recovery */
	u32 XpClkNpcrs;
	u32 XpClkErrs;
	u32 wXpClkPrevErr;
	u32 XpClkPrevPcr;
	u32 XpClkPrevStc;
	s32 lXpClkPrevDelta;
	u32 uwXpClkThreshold;
	
	/* Volatiles */
	volatile u8 sync_av;
	volatile u8 adjust_clk;
	
	/* Channel change */
	struct semaphore vcc_wait;
	struct semaphore acc_wait;
	
	/* Queue */
	struct workqueue_struct *workqueue;
};

/* STBx25xx DVB device helper structure */
struct stbx25xx_dvb_data {
	/* general */
	struct device	*dev;
	struct module	*owner;
	
	/* interrupts */
#define STBx25xx_IRQ_DEMUX	0
#define STBx25xx_IRQ_AUDIO	1
#define STBx25xx_IRQ_VIDEO	2
#define STBx25xx_IRQ_TSDMA	3
	unsigned int irq_num[4];

	/* dvb stuff */
	struct i2c_adapter 	*i2c;
	struct dvb_adapter 	dvb_adapter;
	struct dvb_frontend 	*fe;
	struct dvb_net 		dvbnet;
	
	/* Private data of driver modules */
#ifdef CONFIG_DVB_STBx25xx_AV
	struct stbx25xx_video_data	video;
	struct stbx25xx_audio_data	audio;
#endif
	struct stbx25xx_demux_data	demux;
};

extern struct proc_dir_entry *stbx25xx_proc_dir;

/* Function prototypes */
extern int stbx25xx_demux_connect_frontend(struct dmx_demux* demux,
				 struct dmx_frontend* frontend);
extern int stbx25xx_demux_disconnect_frontend(struct dmx_demux* demux);
extern int stbx25xx_demux_get_stc(struct dmx_demux* demux, unsigned int num,
			u64 *stc, unsigned int *base);
extern void stbx25xx_demux_before_after_tune(fe_status_t fe_status, void *data);
extern int stbx25xx_demux_start_feed(struct dvb_demux_feed *feed);
extern int stbx25xx_demux_stop_feed(struct dvb_demux_feed *feed);

#ifdef CONFIG_DVB_STBx25xx_AV
extern ssize_t stbx25xx_video_write(struct file *file, const char *buf, size_t count, loff_t *ppos);
extern int stbx25xx_video_open(struct inode *inode, struct file *file);
extern int stbx25xx_video_release(struct inode *inode, struct file *file);
extern unsigned int stbx25xx_video_poll(struct file *file, poll_table *wait);
extern int stbx25xx_video_ioctl(struct inode *inode, struct file *file, unsigned int cmd, void *parg);
extern void stbx25xx_video_sync_stc(u32 stcl, u32 stch);
extern void stbx25xx_video_disable_sync(void);
extern void stbx25xx_video_enable_sync(void);

extern ssize_t stbx25xx_audio_write(struct file *file, const char *buf, size_t count, loff_t *ppos);
extern int stbx25xx_audio_open(struct inode *inode, struct file *file);
extern int stbx25xx_audio_release(struct inode *inode, struct file *file);
extern unsigned int stbx25xx_audio_poll(struct file *file, poll_table *wait);
extern int stbx25xx_audio_ioctl(struct inode *inode, struct file *file, unsigned int cmd, void *parg);
extern void stbx25xx_audio_sync_stc(u32 stcl, u32 stch);
#endif
 
#ifdef CONFIG_DVB_STBx25xx_AV
extern int stbx25xx_video_init(struct stbx25xx_dvb_data *);
extern int stbx25xx_audio_init(struct stbx25xx_dvb_data *);
#endif
extern int stbx25xx_demux_init(struct stbx25xx_dvb_data *);
extern int stbx25xx_frontend_init(struct stbx25xx_dvb_data *);
#ifdef CONFIG_DVB_STBx25xx_OSD
extern int stbx25xx_osd_init(struct stbx25xx_dvb_data *);
#endif

#ifdef CONFIG_DVB_STBx25xx_AV
extern void stbx25xx_video_exit(struct stbx25xx_dvb_data *);
extern void stbx25xx_audio_exit(struct stbx25xx_dvb_data *);
#endif
extern void stbx25xx_demux_exit(struct stbx25xx_dvb_data *);
extern void stbx25xx_frontend_exit(struct stbx25xx_dvb_data *);
#ifdef CONFIG_DVB_STBx25xx_OSD
extern void stbx25xx_osd_exit(struct stbx25xx_dvb_data *);
#endif

#endif
