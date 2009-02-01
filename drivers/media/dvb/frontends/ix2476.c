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
	u8 regs[4];
};

static u32 calculate_pll_lpf_cutoff(u32 baud)
{
	if ( baud>=39000000 )				return 30000;//LPF=30MHz
	if ( (39000000>baud) && (baud>=35000000) )	return 28000;//LPF=28MHz
	if ( (35000000>baud) && (baud>=30000000) )	return 26000;//LPF=26MHz
	if ( (30000000>baud) && (baud>=26000000) )	return 24000;//LPF=24MHz
	if ( (26000000>baud) && (baud>=23000000) )	return 22000;//LPF=22MHz
	if ( (23000000>baud) && (baud>=21000000) )	return 20000;//LPF=20MHz
	if ( (21000000>baud) && (baud>=19000000) )	return 18000;//LPF=18MHz
	if ( (19000000>baud) && (baud>=18000000) )	return 16000;//LPF=16MHz
	if ( (18000000>baud) && (baud>=17000000) )	return 14000;//LPF=14MHz
	if ( (17000000>baud) && (baud>=16000000) )	return 12000;//LPF=12MHz
	else						return 10000;//LPF=10MHz
}

static void calculate_pll_vco(u32 freq, u32 *div, u32 *ba)
{
	BUG_ON(!div);
	BUG_ON(!ba);
	
	*div	= 1;
	*ba	= 6;

	if ( (950000<=freq) && (freq<1065000) )		*div=1,*ba=6;
	else if ( (1065000<=freq) && (freq<1170000) )	*div=1,*ba=7;
	else if ( (1170000<=freq) && (freq<1300000) )	*div=0,*ba=1;
	else if ( (1300000<=freq) && (freq<1445000) )	*div=0,*ba=2;
	else if ( (1445000<=freq) && (freq<1607000) )	*div=0,*ba=3;
	else if ( (1607000<=freq) && (freq<1778000) )	*div=0,*ba=4;
	else if ( (1778000<=freq) && (freq<1942000) )	*div=0,*ba=5;
	else if (1942000<=freq)				*div=0,*ba=6;
}

static int ix2476_release(struct dvb_frontend *fe)
{
	kfree(fe->tuner_priv);
	fe->tuner_priv = NULL;
	return 0;
}

static int ix2476_sleep(struct dvb_frontend *fe)
{
	dprintk("%s: not implemented\n", __func__);
	
	return 0;
}

#define TUNER_XTAL_KHZ		(4000)
#define TUNER_PLL_STEP		(TUNER_XTAL_KHZ / 8)

static int ix2476_set_params(struct dvb_frontend *fe,
				struct dvb_frontend_parameters *params)
{
	struct ix2476_priv *priv = fe->tuner_priv;
	u32 data, reg2, reg3, div, ba, cutoff, pd2, pd3, pd4, pd5;
	int ret = 0;
	struct i2c_msg msg[] = {
		{.addr = priv->i2c_address,	.flags = 0,	.buf = &priv->regs[0],	.len = 4 },
		{.addr = priv->i2c_address,	.flags = 0,	.buf = &priv->regs[2],	.len = 1 },
		{.addr = priv->i2c_address,	.flags = 0,	.buf = &priv->regs[2],	.len = 2 },
	};
	
	dprintk("%s: Tunning to frequency %d and symbol-rate %d\n", __func__, params->frequency, params->u.qpsk.symbol_rate);

	// Frequency
	data = params->frequency / TUNER_PLL_STEP;
	
	priv->regs[0] = (data >> 8) & 0x1f;
	priv->regs[1] = data;
	
	// Local oscillator
	calculate_pll_vco(params->frequency, &div, &ba);
	
	priv->regs[3] = (ba << 5) | ((div & 1) << 1);
	
	// LPF cut-off
	cutoff = calculate_pll_lpf_cutoff(params->u.qpsk.symbol_rate);
	cutoff = ((cutoff / 1000) / 2) - 2;
	pd2 = (cutoff >> 1) & 0x04;
	pd3 = (cutoff << 1) & 0x08;
	pd4 = (cutoff << 2) & 0x08;
	pd5 = (cutoff << 4) & 0x10;
	priv->regs[2] &= 0xe7;
	priv->regs[2] |= (pd5 | pd4);
	priv->regs[3] &= 0xf3;
	priv->regs[3] |= (pd3 | pd2);
	
	dprintk("%s: ix2476 registers: %02x, %02x, %02x, %02x\n", __func__,
		 priv->regs[0], priv->regs[1], priv->regs[2], priv->regs[3]);
	
	// Initialize
	reg2 = priv->regs[2];
	reg3 = priv->regs[3];
	priv->regs[2] &= 0xE3;	// PD5=0, PD4=0, TM=0
	priv->regs[3] &= 0xF3;	// PD3=0, PD2=0
	
	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 1);
	if ((ret = i2c_transfer (priv->i2c, &msg[0], 1)) != 1) {
		dprintk("%s: i2c #1 error\n", __func__);
	}
	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 0);
	
	priv->regs[2] |= 0x04;	//TM=1
	
	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 1);
	if ((ret = i2c_transfer (priv->i2c, &msg[1], 1)) != 1) {
		dprintk("%s: i2c #2 error\n", __func__);
	}
	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 0);
	
	msleep(10);
	
	priv->regs[2] = reg2 | 0x04;
	priv->regs[3] = reg3;
	
	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 1);
	if ((ret = i2c_transfer (priv->i2c, &msg[2], 1)) != 1) {
		dprintk("%s: i2c #3 error\n", __func__);
	}
	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 0);
	
	return 0;
}

static int ix2476_get_frequency(struct dvb_frontend *fe, u32 *frequency)
{
	struct ix2476_priv *priv = fe->tuner_priv;
	u32 data;
	
	data = ((priv->regs[0] & 0x1f) << 8) | (priv->regs[1]);
	*frequency = TUNER_PLL_STEP * data;
	
	dprintk("%s: Frequency = %d kHz\n", __func__, *frequency);
	
	return 0;
}

static int ix2476_get_status(struct dvb_frontend *fe, u32 *status)
{
	int ret;
	u8 buf = 0;
	struct ix2476_priv *priv = fe->tuner_priv;
	struct i2c_msg msg = {
		.addr = priv->i2c_address,	.flags = I2C_M_RD,	.buf = &buf,	.len = 1,
	};
	
	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 1);
	if ((ret = i2c_transfer (priv->i2c, &msg, 1)) != 1) {
		dprintk("%s: i2c read error\n", __func__);
	}
	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 0);
	
	*status = 0;
	
	if(buf & 0x40)
		*status |= TUNER_STATUS_LOCKED;
	
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
	.get_status = ix2476_get_status,
};

struct dvb_frontend *ix2476_attach(struct dvb_frontend *fe, int addr,
						struct i2c_adapter *i2c)
{
	struct ix2476_priv *priv = NULL;
	int ret;

	priv = kzalloc(sizeof(struct ix2476_priv), GFP_KERNEL);
	if (priv == NULL)
		return NULL;

	priv->i2c_address = addr;
	priv->i2c = i2c;
	priv->regs[0] = 0x00;
	priv->regs[1] = 0x00;
	priv->regs[2] = 0xE1;
	priv->regs[3] = 0x00;

	memcpy(&fe->ops.tuner_ops, &ix2476_tuner_ops,
				sizeof(struct dvb_tuner_ops));

	fe->tuner_priv = priv;

	return fe;
}
EXPORT_SYMBOL(ix2476_attach);

module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Turn on/off frontend debugging (default:off).");

MODULE_DESCRIPTION("Sharp IX2410/IX2476 driver");
MODULE_AUTHOR("Tomasz Figa <tomasz.figa@gmail.com>");
MODULE_LICENSE("GPL");
