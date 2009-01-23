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

#include "dmxdev.h"
#include "dvb_demux.h"
#include "dvb_filter.h"
#include "dvb_net.h"
#include "dvb_frontend.h"

#define STBx25xx_MAX_FEED		30
#define STBx25xx_QUEUE_COUNT		32
#define STBx25xx_DEMUX1_IRQ_COUNT	15
#define STBx25xx_LOG_PREFIX	"dvb-stbx25xx"

/* Stolen from usb.h */
#undef err
#define err(format,  arg...) printk(KERN_ERR     STBx25xx_LOG_PREFIX ": " format "\n" , ## arg)
#undef info
#define info(format, arg...) printk(KERN_INFO    STBx25xx_LOG_PREFIX ": " format "\n" , ## arg)
#undef warn
#define warn(format, arg...) printk(KERN_WARNING STBx25xx_LOG_PREFIX ": " format "\n" , ## arg)

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
	int (*fe_sleep) (struct dvb_frontend *);

	struct module *owner;

	/* options and status */
	int extra_feedcount;
	int feedcount;
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
 
extern int stbx25xx_video_init(struct stbx25xx_dvb_dev *);
extern int stbx25xx_audio_init(struct stbx25xx_dvb_dev *);
extern int stbx25xx_demux_init(struct stbx25xx_dvb_dev *);
extern int stbx25xx_frontend_init(struct stbx25xx_dvb_dev *);

extern void stbx25xx_video_exit(struct stbx25xx_dvb_dev *);
extern void stbx25xx_audio_exit(struct stbx25xx_dvb_dev *);
extern void stbx25xx_demux_exit(struct stbx25xx_dvb_dev *);
extern void stbx25xx_frontend_exit(struct stbx25xx_dvb_dev *);

#endif
