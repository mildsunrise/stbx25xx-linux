/*
 * stbx25xx_tuner.c - DVB Front-end/tuner driver
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

#include "stbx25xx_common.h"
#include "stv0288.h"
#include "stb6000.h"
#include <linux/i2c.h>
#include <linux/gpio.h>

static struct stv0288_config tm9101_config = {
	.demod_address = 0x68,
	.min_delay_ms = 100,
};

static int tm9101_set_voltage(struct dvb_frontend *fe, fe_sec_voltage_t voltage)
{
	if(voltage == SEC_VOLTAGE_OFF) {
		gpio_set_value(231, 1);
	} else {
		gpio_set_value(231, 0);
	}
	
	msleep(10);
	
	if (voltage == SEC_VOLTAGE_18)
		gpio_set_value(229, 0);
	else
		gpio_set_value(229, 1);
	
	msleep(10);

	return 0;
}

static void stv0288_reset(void)
{
	gpio_set_value(228, 0);
	
	msleep(2);
	
	gpio_set_value(228, 1);
	
	msleep(20);
}

int stbx25xx_frontend_init(struct stbx25xx_dvb_dev *dvb)
{
	int ret;
	
	gpio_direction_output(228, 1);
	
	stv0288_reset();

	dvb->i2c = i2c_get_adapter(0);
	if(!dvb->i2c) {
		dev_err(dvb->dev, "could not get i2c adapter\n");
		return -ENODEV;
	}
	
	if (!dvb->fe) {
		dvb->fe = dvb_attach(stv0288_attach, &tm9101_config, dvb->i2c);
		if (dvb->fe) {
			dvb->fe->ops.set_voltage = tm9101_set_voltage;
			dvb_attach(stb6000_attach, dvb->fe, 0x60,
					dvb->i2c);
		}
	}

	if (!dvb->fe) {
		dev_err(dvb->dev, "could not attach frontend\n");
		return -ENODEV;
	}

	ret = dvb_register_frontend(&dvb->dvb_adapter, dvb->fe);
	if (ret < 0) {
		if (dvb->fe->ops.release)
			dvb->fe->ops.release(dvb->fe);
		dvb->fe = NULL;
		return ret;
	}
	
	gpio_direction_output(231, 1);
	gpio_direction_output(229, 0);

	return 0;
}

void stbx25xx_frontend_exit(struct stbx25xx_dvb_dev *dvb)
{
	if(dvb->fe)
		dvb_unregister_frontend(dvb->fe);
}
