/*
 * stbx25xx.c - driver for digital TV devices equipped with IBM STBx25xx SoC
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

#define DRIVER_NAME "IBM STBx25xx digital TV hardware driver"
#define DRIVER_AUTHOR "Tomasz Figa <tomasz.figa@gmail.com>"

#ifdef CONFIG_DVB_STBx25xx_DEBUG
#define DEBSTATUS ""
#else
#define DEBSTATUS " (debugging is not enabled)"
#endif

int dvb_stbx25xx_debug;
module_param_named(debug, dvb_stbx25xx_debug,  int, 0644);
MODULE_PARM_DESC(debug, "set debug level (1=info,2=tuner,4=i2c,8=ts,16=sram,32=reg (|-able))." DEBSTATUS);
#undef DEBSTATUS

struct stbx25xx_dvb_dev stbx25xx_dvb_dev;

int dvb_stbx25xx_device_initialize(struct stbx25xx_dvb_dev *dev)
{
	int ret;
	ibi_zero.raw = 0;

	dvb_stbx25xx_reset(dev);
	dvb_stbx25xx_determine_revision(dev);
	dvb_stbx25xx_sram_init(dev);
	dvb_stbx25xx_hw_filter_init(dev);

	dvb_stbx25xx_smc_ctrl(dev, 0);

	if ((ret = dvb_stbx25xx_dvb_init(dev)))
		goto error;

	if ((ret = dvb_stbx25xx_frontend_init(dev)))
		goto error;

	dvb_stbx25xx_device_name(dev,"initialization of","complete");

	return 0;

error:
	dvb_stbx25xx_device_exit(dev);
	return ret;
}

void dvb_stbx25xx_device_exit(struct stbx25xx_dvb_dev *dev)
{
	dvb_stbx25xx_frontend_exit(dev);
	dvb_stbx25xx_i2c_exit(dev);
	dvb_stbx25xx_dvb_exit(dev);
}

static int dvb_stbx25xx_module_init(void)
{
	info(DRIVER_NAME " loaded successfully");
	info(DRIVER_NAME " initializing hardware...");
	dvb_stbx25xx_device_init(&stbx25xx_dvb_dev);
	return 0;
}

static void dvb_stbx25xx_module_cleanup(void)
{
	info(DRIVER_NAME " unloaded successfully");
}

module_init(dvb_stbx25xx_module_init);
module_exit(dvb_stbx25xx_module_cleanup);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_NAME);
MODULE_LICENSE("GPL");
