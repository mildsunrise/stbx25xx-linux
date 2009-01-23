  /*
     Driver for ST STB6000 DVBS Silicon tuner

     Copyright (C) 2008 Igor M. Liplianin (liplianin@me.by)

     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation; either version 2 of the License, or
     (at your option) any later version.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the

     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program; if not, write to the Free Software
     Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

  */

#include <linux/module.h>
#include <linux/dvb/frontend.h>
#include <asm/types.h>

#include "ix2476.h"

static int debug;
#define dprintk(args...) \
	do { \
		if (debug) \
			printk(KERN_DEBUG "ix2476: " args); \
	} while (0)

struct ix2476_priv {
	/* i2c details */
	int i2c_address;
	struct i2c_adapter *i2c;
	u32 frequency;
};

static u32 calculate_pll_lpf_cutoff(u32 baud)
{
	if ( baud>=40000 )
		return 34000;//LPF=34MHz
		
	if ( (40000>baud) && (baud>=39000) )
		return 30000;//LPF=30MHz
		
	if ( (39000>baud) && (baud>=35000) )       
		return 28000;//LPF=28MHz
		
	if ( (35000>baud) && (baud>=30000) )       
		return 26000;//LPF=26MHz
		
	if ( (30000>baud) && (baud>=26000) )       
		return 24000;//LPF=24MHz
		
	if ( (26000>baud) && (baud>=23000) )       
		return 22000;//LPF=22MHz
		
	if ( (23000>baud) && (baud>=21000) )       
		return 20000;//LPF=20MHz
		
	if ( (21000>baud) && (baud>=19000) )       
		return 18000;//LPF=18MHz
		
	if ( (19000>baud) && (baud>=18000) )       
		return 16000;//LPF=16MHz
		
	if ( (18000>baud) && (baud>=17000) )       
		return 14000;//LPF=14MHz
		
	if ( (17000>baud) && (baud>=16000) )       
		return 12000;//LPF=12MHz
		
/*	if ( 16000>baud) 
		return 10000;//LPF=10MHz */
		
	return 10000; //LPF=10MHz
}

static int ix2476_release(struct dvb_frontend *fe)
{
	kfree(fe->tuner_priv);
	fe->tuner_priv = NULL;
	return 0;
}

static int ix2476_sleep(struct dvb_frontend *fe)
{
	struct ix2476_priv *priv = fe->tuner_priv;
	int ret;
	u8 buf[] = { 10, 0 };
	struct i2c_msg msg = {
		.addr = priv->i2c_address,
		.flags = 0,
		.buf = buf,
		.len = 2
	};

	dprintk("%s:\n", __func__);

	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 1);

	ret = i2c_transfer(priv->i2c, &msg, 1);
	if (ret != 1)
		dprintk("%s: i2c error\n", __func__);

	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 0);

	return (ret == 1) ? 0 : ret;
}

#define TUNER_XTAL_KHZ 40000

static int ix2476_set_params(struct dvb_frontend *fe,
				struct dvb_frontend_parameters *params)
{
	struct ix2476_priv *priv = fe->tuner_priv;
	u32 data, pd2, pd3, pd4, pd5, cutoff, pll_step, R10, Nref, div, ba, freq, baud, byte2, byte3;
	int ret = 0;
	u8 buf [4] = {0, 0, 0, 0xE5};
	struct i2c_msg msg[] = {
		{.addr = priv->i2c_address,	.flags = 0,	.buf = buf,	.len = 4 },
		{.addr = priv->i2c_address,	.flags = 0,	.buf = &buf[2],	.len = 1 },
		{.addr = priv->i2c_address,	.flags = 0,	.buf = &buf[2],	.len = 2 },
	};
	
	freq = params->frequency;
	baud = params->u.qpsk.symbol_rate / 1000;
	
	// Divisor
	R10 = buf[3] & 3;
	Nref = 4 * (1 << R10);
	pll_step = TUNER_XTAL_KHZ / Nref;
	data = ((freq * 2) / (pll_step * 2) + 1)/2;
	buf[0] = (data >> 8) & 0x7f;
	buf[1] = data & 0xff;
	
	// Bandwidth
	cutoff = calculate_pll_lpf_cutoff(baud);
	data = ((cutoff / 1000) / 2) - 2;
	pd2 = (data >> 1) & 0x04;
	pd3 = (data << 1) & 0x08;
	pd4 = (data << 2) & 0x08;
	pd5 = (data << 4) & 0x10;
	buf[2] &= 0xE7;
	buf[3] &= 0xF3;
	buf[2] |= (pd5 | pd4);
	buf[3] |= (pd3 | pd2);
	
	// Frequency
	div	= 1;
	ba	= 5;

	if ( (950000<=freq) && (freq<970000) )		div=1,ba=6; //div=1,ba=5;
	else if ( (970000<=freq) && (freq<1065000) )	div=1,ba=6;
	else if ( (1065000<=freq) && (freq<1170000) )	div=1,ba=7;
	else if ( (1170000<=freq) && (freq<1300000) )	div=0,ba=1;
	else if ( (1300000<=freq) && (freq<1445000) )	div=0,ba=2;
	else if ( (1445000<=freq) && (freq<1607000) )	div=0,ba=3;
	else if ( (1607000<=freq) && (freq<1778000) )	div=0,ba=4;
	else if ( (1778000<=freq) && (freq<1942000) )	div=0,ba=5;
	else if ( (1942000<=freq) && (freq<2131000) )	div=0,ba=6;
	else if (2131000<=freq)				div=0,ba=6;

	buf[3] &= 0xFD;
	buf[3] |= (div << 1);
	buf[3] &= 0x1F;
	buf[3] |= (ba << 5);
	
	// Initialize
	byte2 = buf[2];
	byte3 = buf[3];
	buf[2] &= 0xE3;	//TM=0,LPF=4MHz
	buf[3] &= 0xF3;	//LPF=4MHz
	
	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 1);
	if ((ret = i2c_transfer (priv->i2c, &msg[0], 1)) != 1) {
		dprintk("%s: i2c error\n", __func__);
	}
	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 0);
	
	buf[2] |= 0x04;	//TM=1
	
	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 1);
	if ((ret = i2c_transfer (priv->i2c, &msg[1], 1)) != 1) {
		dprintk("%s: i2c error\n", __func__);
	}
	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 0);
	
	msleep(20);
	
	buf[2] = byte2;
	buf[3] = byte3;
	
	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 1);
	if ((ret = i2c_transfer (priv->i2c, &msg[2], 1)) != 1) {
		dprintk("%s: i2c error\n", __func__);
	}
	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 0);
	
	return 0;
}

static int ix2476_get_frequency(struct dvb_frontend *fe, u32 *frequency)
{
	struct ix2476_priv *priv = fe->tuner_priv;
	*frequency = priv->frequency;
	return 0;
}

static struct dvb_tuner_ops ix2476_tuner_ops = {
	.info = {
		.name = "Sharp IX2476",
		.frequency_min = 950000,
		.frequency_max = 2150000
	},
	.release = ix2476_release,
	.sleep = ix2476_sleep,
	.set_params = ix2476_set_params,
	.get_frequency = ix2476_get_frequency,
};

struct dvb_frontend *ix2476_attach(struct dvb_frontend *fe, int addr,
						struct i2c_adapter *i2c)
{
	struct ix2476_priv *priv = NULL;
	u8 b0[] = { 0 };
	struct i2c_msg msg[1] = {
		{
			.addr = addr,
			.flags = I2C_M_RD,
			.buf = b0,
			.len = 1
		}
	};
	int ret;

	dprintk("%s:\n", __func__);

	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 1);

	/* is some i2c device here ? */
	ret = i2c_transfer(i2c, msg, 1);
	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 0);

	if (ret != 1)
		return NULL;

	priv = kzalloc(sizeof(struct ix2476_priv), GFP_KERNEL);
	if (priv == NULL)
		return NULL;

	priv->i2c_address = addr;
	priv->i2c = i2c;

	memcpy(&fe->ops.tuner_ops, &ix2476_tuner_ops,
				sizeof(struct dvb_tuner_ops));

	fe->tuner_priv = priv;

	return fe;
}
EXPORT_SYMBOL(ix2476_attach);

module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Turn on/off frontend debugging (default:off).");

MODULE_DESCRIPTION("DVB STB6000 driver");
MODULE_AUTHOR("Igor M. Liplianin <liplianin@me.by>");
MODULE_LICENSE("GPL");
