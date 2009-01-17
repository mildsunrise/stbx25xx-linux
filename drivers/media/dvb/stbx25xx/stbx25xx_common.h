/*
 * This file is part of linux driver the digital TV devices equipped with IBM STBx25xx SoC
 *
 * stbx25xx-common.h - common header file for device-specific source files.
 *
 * see stbx25xx.c for copyright information.
 */

#ifndef __STBx25xx_COMMON_H__
#define __STBx25xx_COMMON_H__

#include "dmxdev.h"
#include "dvb_demux.h"
#include "dvb_filter.h"
#include "dvb_net.h"
#include "dvb_frontend.h"

struct stbx25xx_dvb_dev {
	/* general */
	struct device *dev; /* for firmware_class */

#define FC_STATE_DVB_INIT 0x01
#define FC_STATE_I2C_INIT 0x02
#define FC_STATE_FE_INIT  0x04
	int init_state;

	/* dvb stuff */
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

#endif
