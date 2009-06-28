/*
	Driver for ST STx0288 demodulator
	Copyright (C) 2006 Georg Acher, BayCom GmbH, acher (at) baycom (dot) de
		for Reel Multimedia
	Copyright (C) 2008 TurboSight.com, Bob Liu <bob@turbosight.com>
	Copyright (C) 2008 Igor M. Liplianin <liplianin@me.by>
		Removed stb6000 specific tuner code and revised some
		procedures.

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

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <asm/div64.h>

#include "dvb_frontend.h"
#include "stx0288.h"
#include "stx0288_regs.h"

struct stx0288_state {
	struct i2c_adapter *i2c;
	const struct stx0288_config *config;
	struct dvb_frontend frontend;

	u8 initialised:1;
	u32 tuner_frequency;
	u32 symbol_rate;
	fe_code_rate_t fec_inner;
	int errmode;
	u32 master_clock;
	u32 mclk_div;
	u32 ucblocks;
};

#define STATUS_BER 0
#define STATUS_UCBLOCKS 1
#define DEMOD_XTAL_HZ	4000000
#define ABS(X) ((X)<0 ? (-(X)) : (X))
#define LSB(X) ((X & 0xFF))
#define MSB(Y) ((Y>>8)& 0xFF)

static int debug;
static int debug_legacy_dish_switch;
#define dprintk(args...) \
	do { \
		if (debug) \
			printk(KERN_DEBUG "stx0288: " args); \
	} while (0)


static int stx0288_writeregI(struct stx0288_state *state, u8 reg, u8 data)
{
	int ret;
	u8 buf[] = { reg, data };
	struct i2c_msg msg = {
		.addr = state->config->demod_address,
		.flags = 0,
		.buf = buf,
		.len = 2
	};

	ret = i2c_transfer(state->i2c, &msg, 1);

	if (ret != 1)
		dprintk("%s: writereg error (reg == 0x%02x, val == 0x%02x, "
			"ret == %i)\n", __func__, reg, data, ret);

	return (ret != 1) ? -EREMOTEIO : 0;
}

static int stx0288_write(struct dvb_frontend *fe, u8 *buf, int len)
{
	struct stx0288_state *state = fe->demodulator_priv;

	if (len != 2)
		return -EINVAL;

	return stx0288_writeregI(state, buf[0], buf[1]);
}

static u8 stx0288_readreg(struct stx0288_state *state, u8 reg)
{
	int ret;
	u8 b0[] = { reg };
	u8 b1[] = { 0 };
	struct i2c_msg msg[] = {
		{
			.addr = state->config->demod_address,
			.flags = 0,
			.buf = b0,
			.len = 1
		}, {
			.addr = state->config->demod_address,
			.flags = I2C_M_RD,
			.buf = b1,
			.len = 1
		}
	};

	ret = i2c_transfer(state->i2c, msg, 2);

	if (ret != 2)
		dprintk("%s: readreg error (reg == 0x%02x, ret == %i)\n",
				__func__, reg, ret);

	return b1[0];
}

static int stx0288_set_symbolrate(struct dvb_frontend *fe, u32 srate)
{
	struct stx0288_state *state = fe->demodulator_priv;
	unsigned int temp;
	unsigned char b[3];

	if ((srate < 1000000) || (srate > 45000000))
		return -EINVAL;

	temp = (unsigned int)srate / 1000;

	temp = temp * 32768;
	temp = temp / 25;
	temp = temp / 125;
	b[0] = (unsigned char)((temp >> 12) & 0xff);
	b[1] = (unsigned char)((temp >> 4) & 0xff);
	b[2] = (unsigned char)((temp << 4) & 0xf0);
	stx0288_writeregI(state, R288_SFRH, 0x80); /* SFRH */
	stx0288_writeregI(state, R288_SFRM, 0); /* SFRM */
	stx0288_writeregI(state, R288_SFRL, 0); /* SFRL */
	
	msleep(1);

	stx0288_writeregI(state, R288_SFRH, b[0]);
	stx0288_writeregI(state, R288_SFRM, b[1]);
	stx0288_writeregI(state, R288_SFRL, b[2]);
	dprintk("stx0288: stx0288_set_symbolrate\n");
	
	stx0288_writeregI(state, R288_RTFM, 0);
	stx0288_writeregI(state, R288_RTFL, 0);

	return 0;
}

static int stx0288_send_diseqc_msg(struct dvb_frontend *fe,
				    struct dvb_diseqc_master_cmd *m)
{
	struct stx0288_state *state = fe->demodulator_priv;

	int i;

	dprintk("%s\n", __func__);

	stx0288_writeregI(state, 0x09, 0);
	msleep(30);
	stx0288_writeregI(state, 0x05, 0x16);

	for (i = 0; i < m->msg_len; i++) {
		if (stx0288_writeregI(state, 0x06, m->msg[i]))
			return -EREMOTEIO;
		msleep(12);
	}

	return 0;
}

static int stx0288_send_diseqc_burst(struct dvb_frontend *fe,
						fe_sec_mini_cmd_t burst)
{
	struct stx0288_state *state = fe->demodulator_priv;

	dprintk("%s\n", __func__);

	if (stx0288_writeregI(state, 0x05, 0x16))/* burst mode */
		return -EREMOTEIO;

	if (stx0288_writeregI(state, 0x06, burst == SEC_MINI_A ? 0x00 : 0xff))
		return -EREMOTEIO;

	if (stx0288_writeregI(state, 0x06, 0x12))
		return -EREMOTEIO;

	return 0;
}

static int stx0288_set_tone(struct dvb_frontend *fe, fe_sec_tone_mode_t tone)
{
	struct stx0288_state *state = fe->demodulator_priv;

	switch (tone) {
	case SEC_TONE_ON:
		if (stx0288_writeregI(state, 0x05, 0x10))/* burst mode */
			return -EREMOTEIO;
		return stx0288_writeregI(state, 0x06, 0xff);

	case SEC_TONE_OFF:
		if (stx0288_writeregI(state, 0x05, 0x13))/* burst mode */
			return -EREMOTEIO;
		return stx0288_writeregI(state, 0x06, 0x00);

	default:
		return -EINVAL;
	}
}

static u8 stx0288_inittab[] = {
	0x01, 0x15,
	0x02, 0x20,
	0x09, 0x0,
	0x0a, 0x4,
	0x0b, 0x0,
	0x0c, 0x0,
	0x0d, 0x0,
	0x0e, 0xd4,
	0x0f, 0x30,
	0x11, 0x80,
	0x12, 0x03,
	0x13, 0x48,
	0x14, 0x84,
	0x15, 0x45,
	0x16, 0xb7,
	0x17, 0x9c,
	0x18, 0x0,
	0x19, 0xa6,
	0x1a, 0x88,
	0x1b, 0x8f,
	0x1c, 0xf0,
	0x20, 0x0b,
	0x21, 0x54,
	0x22, 0x0,
	0x23, 0x0,
	0x2b, 0xff,
	0x2c, 0xf7,
	0x30, 0x0,
	0x31, 0x1e,
	0x32, 0x14,
	0x33, 0x0f,
	0x34, 0x09,
	0x35, 0x0c,
	0x36, 0x05,
	0x37, 0x2f,
	0x38, 0x16,
	0x39, 0xbe,
	0x3a, 0x0,
	0x3b, 0x13,
	0x3c, 0x11,
	0x3d, 0x30,
	0x40, 0x63,
	0x41, 0x04,
	0x42, 0x60,
	0x43, 0x00,
	0x44, 0x00,
	0x45, 0x00,
	0x46, 0x00,
	0x47, 0x00,
	0x4a, 0x00,
	0x50, 0x10,
	0x51, 0x38,
	0x52, 0x21,
	0x58, 0x54,
	0x59, 0x86,
	0x5a, 0x0,
	0x5b, 0x9b,
	0x5c, 0x08,
	0x5d, 0x7f,
	0x5e, 0x0,
	0x5f, 0xff,
	0x70, 0x0,
	0x71, 0x0,
	0x72, 0x0,
	0x74, 0x0,
	0x75, 0x0,
	0x76, 0x0,
	0x81, 0x0,
	0x82, 0x3f,
	0x83, 0x3f,
	0x84, 0x0,
	0x85, 0x0,
	0x88, 0x0,
	0x89, 0x0,
	0x8a, 0x0,
	0x8b, 0x0,
	0x8c, 0x0,
	0x90, 0x0,
	0x91, 0x0,
	0x92, 0x0,
	0x93, 0x0,
	0x94, 0x1c,
	0x97, 0x0,
	0xa0, 0x48,
	0xa1, 0x0,
	0xb0, 0xb8,
	0xb1, 0x3a,
	0xb2, 0x10,
	0xb3, 0x82,
	0xb4, 0x80,
	0xb5, 0x82,
	0xb6, 0x82,
	0xb7, 0x82,
	0xb8, 0x20,
	0xb9, 0x0,
	0xf0, 0x0,
	0xf1, 0x0,
	0xf2, 0xc0,
	0x51, 0x36,
	0x52, 0x09,
	0x53, 0x94,
	0x54, 0x62,
	0x55, 0x29,
	0x56, 0x64,
	0x57, 0x2b,
	0xff, 0xff,
};

static int stx0288_set_voltage(struct dvb_frontend *fe, fe_sec_voltage_t volt)
{
	dprintk("%s: %s\n", __func__,
		volt == SEC_VOLTAGE_13 ? "SEC_VOLTAGE_13" :
		volt == SEC_VOLTAGE_18 ? "SEC_VOLTAGE_18" : "??");

	return 0;
}

static inline u32 stx0288_real_get_mclk(struct stx0288_state *state)
{
//Hz
	u32 pll_divider;   /* pll divider */
	u32 pll_selratio;  /* 4 or 6 ratio */
	u32 pll_bypass;    /* pll bypass */
	u32 ext_clk_hz;
	u32 mclk_hz;
	u8 reg40, reg41;

	ext_clk_hz=DEMOD_XTAL_HZ;
	reg41 = stx0288_readreg(state, 0x41);
	pll_bypass=reg41 & 1;
	
	if(pll_bypass) {
		mclk_hz = ext_clk_hz;
	} else {
		reg40 = stx0288_readreg(state, 0x40);
		pll_divider=reg40 + 1;
		pll_selratio=((reg41 & 0x04) ? 4 : 6);
		mclk_hz = (ext_clk_hz*pll_divider)/pll_selratio;
	}
	
	dprintk("%s: mclk = %d Hz\n", __func__, mclk_hz);

	return mclk_hz;
}

static inline u32 stx0288_get_mclk(struct stx0288_state *state)
{
	BUG_ON(!state->initialised);
		
	return state->master_clock;
}

static int stx0288_init(struct dvb_frontend *fe)
{
	struct stx0288_state *state = fe->demodulator_priv;
	int i;
	u8 reg;
	u8 val;

	dprintk("stx0288: init chip\n");
	stx0288_writeregI(state, 0x41, 0x04);
	msleep(50);

	/* we have default inittab */
	if (state->config->inittab == NULL) {
		for (i = 0; !(stx0288_inittab[i] == 0xff &&
				stx0288_inittab[i + 1] == 0xff); i += 2)
			stx0288_writeregI(state, stx0288_inittab[i],
					stx0288_inittab[i + 1]);
	} else {
		for (i = 0; ; i += 2)  {
			reg = state->config->inittab[i];
			val = state->config->inittab[i+1];
			if (reg == 0xff && val == 0xff)
				break;
			stx0288_writeregI(state, reg, val);
		}
	}
	
	state->master_clock = stx0288_real_get_mclk(state);
	state->mclk_div = state->master_clock / 65536;
	state->initialised = 1;
	
	return 0;
}

static int stx0288_read_status(struct dvb_frontend *fe, fe_status_t *status)
{
	struct stx0288_state *state = fe->demodulator_priv;

	u8 sync = stx0288_readreg(state, 0x24);
	if (sync == 255)
		sync = 0;

	dprintk("%s : FE_READ_STATUS : VSTATUS: 0x%02x\n", __func__, sync);

	*status = 0;

	if (sync & 0x80)
		*status |= FE_HAS_CARRIER;

	if (sync & 0x10)
		*status |= FE_HAS_VITERBI;

	if (sync & 0x08)
		*status |= FE_HAS_SYNC;

	if ((sync & 0x98) == 0x98)
		*status |= FE_HAS_LOCK;

	return 0;
}

#if 0
static int stx0288_read_ber(struct dvb_frontend *fe, u32 *ber)
{
	struct stx0288_state *state = fe->demodulator_priv;

	*ber = (stx0288_readreg(state, R288_ECNTM) << 8) |
		stx0288_readreg(state, R288_ECNTL);

	*ber = (stx0288_readreg(state, R288_ECNTM) << 8) |
		stx0288_readreg(state, R288_ECNTL);
					
	dprintk("stx0288_read_ber %d\n", *ber);

	return 0;
}
#endif

static inline u16 stx0288_get_error_count(struct stx0288_state *state)
{
	u16 err;

	err = stx0288_readreg(state, R288_ECNTM) << 8;
	err |= stx0288_readreg(state, R288_ECNTL);

	return err;
}

static int stx0288_read_ber(struct dvb_frontend *fe, u32 *berval)
{
	u32 ber = 0,i;
	struct stx0288_state *state = fe->demodulator_priv;
	u8 vstatus = stx0288_readreg(state, R288_VSTATUS);
	u8 errctrl = stx0288_readreg(state, R288_ERRCTRL);
	u8 fecm = stx0288_readreg(state, R288_FECM);

	stx0288_get_error_count(state); /* remove first counter value */
	/* Average 5 ber values */
	ber = stx0288_get_error_count(state);
	
	/*	Check for carrier	*/
	if(vstatus & 0x80) {
		if(!(errctrl & 0x80)) {
			/*	Error Rate	*/
			ber *= 9766;
			/*  theses two lines => ber = ber * 10^7	*/
			ber /= (u32)(1 << (2 + 2 * (errctrl & 0x03)));

			switch(vstatus & 0x30) {
			case 0x00 :				/*	QPSK bit errors	*/
				ber /= 8;

				switch(vstatus & 0x07) {
				case	0x00:		/*	PR 1/2	*/
					ber *= 1;
					ber /= 2;
					break;

				case	0x01:		/*	PR 2/3	*/
					ber *= 2;
					ber /= 3;
					break;

				case	0x02:		/*	PR 3/4	*/
					ber *= 3;
					ber /= 4;
				break;

				case	0x03:		/*	PR 5/6	*/
					ber *= 5;
					ber /= 6;
					break	;

				case	0x04:		/*	PR 6/7	*/
					ber *= 6;
					ber /= 7;
					break;

				case	0x05:		/*	PR 7/8	*/
					ber *= 7;
					ber /= 8;
					break;

				default	:
					ber = 0;
					break;
				}
				break;

			case 0x10:		/*	Viterbi bit errors	*/
				ber /= 8;
				break;

			case 0x20:		/*	Viterbi	byte errors	*/
				break;

			case 0x30:		/*	Packet errors	*/
				if((fecm >> 4) != 0x04)
					ber *= 204;	/* DVB */
				else
					ber *= 147; /* DirecTV */
				break;
			}
		}
	}

	*berval = ber;

	dprintk("stx0288_read_ber %d\n", *berval);

	return 0;
}

static int stx0288_read_signal_strength(struct dvb_frontend *fe, u16 *strength)
{
	struct stx0288_state *state = fe->demodulator_priv;
	s32 agc_gain;

	agc_gain = (s8)stx0288_readreg(state, R288_AGC1IN);
	agc_gain += 128;
	agc_gain *= 256;

	*strength = (agc_gain > 0xffff) ? 0xffff : (agc_gain < 0) ? 0 : agc_gain;
	
	dprintk("stx0288_read_signal_strength %d\n", *strength);

	return 0;
}

static int stx0288_sleep(struct dvb_frontend *fe)
{
	struct stx0288_state *state = fe->demodulator_priv;

	stx0288_writeregI(state, 0x41, 0x84);
	state->initialised = 0;

	return 0;
}

static int stx0288_read_snr(struct dvb_frontend *fe, u16 *snr)
{
	struct stx0288_state *state = fe->demodulator_priv;

	s32 xsnr;

	if(stx0288_readreg(state, R288_VSTATUS) & F288_CF) {
		xsnr = 0xffff - ((stx0288_readreg(state, R288_NIRM) << 8)
			| stx0288_readreg(state, R288_NIRL));

		xsnr = 3 * (xsnr - 0xa100);

		*snr = (xsnr > 0xffff) ? 0xffff : (xsnr < 0) ? 0 : xsnr;
	} else {
		*snr = 0;
	}

	dprintk("stx0288_read_snr %d\n", *snr);

	return 0;
}

static int stx0288_read_ucblocks(struct dvb_frontend *fe, u32 *ucblocks)
{
	struct stx0288_state *state = fe->demodulator_priv;
	u8 mode;

	if(state->errmode != STATUS_UCBLOCKS)
		return 0;

	mode = stx0288_readreg(state, R288_ERRCTRL);
	stx0288_writeregI(state, R288_ERRCTRL, 0x80);

	stx0288_readreg(state, R288_ECNTM);
	stx0288_readreg(state, R288_ECNTL);

	state->ucblocks += (stx0288_readreg(state, R288_ECNTM) << 8) |
			stx0288_readreg(state, R288_ECNTL);

	stx0288_writeregI(state, R288_ERRCTRL, mode);

	*ucblocks = state->ucblocks;
			
	dprintk("stx0288_read_ucblocks %d\n", *ucblocks);

	return 0;
}

static int stx0288_set_property(struct dvb_frontend *fe, struct dtv_property *p)
{
	dprintk("%s(..)\n", __func__);
	return 0;
}

static int stx0288_get_property(struct dvb_frontend *fe, struct dtv_property *p)
{
	dprintk("%s(..)\n", __func__);
	return 0;
}

static inline void stx0288_set_froze_lock(struct stx0288_state* state, int on)
{
	stx0288_writeregI(state, R288_ASCTRL, (stx0288_readreg(state, R288_ASCTRL) & 0xdf) | ((on) ? 0x20 : 0));
}

static inline void stx0288_set_autocenter(struct stx0288_state* state, int on)
{
	stx0288_writeregI(state, R288_ASCTRL, (stx0288_readreg(state, R288_ASCTRL) & 0xfb) | ((on) ? 0x04 : 0));
}

static inline void stx0288_set_frequency_offset_detector(struct stx0288_state* state, int on)
{
	stx0288_writeregI(state, R288_CFD, (stx0288_readreg(state, R288_CFD) & 0x7f) | ((on) ? 0x80 : 0));
}

static inline s16 stx0288_get_timing_loop_freq(struct stx0288_state *state)
{
	u8 rtfm, rtfl;
	
	rtfm = stx0288_readreg(state, R288_RTFM);
	rtfl = stx0288_readreg(state, R288_RTFL);
	
	return (s16)((u16)((rtfm << 8) | rtfl));
}

static inline void stx0288_set_alpha_beta(struct stx0288_state *state, int alpha, int beta)
{
	u8 aclc, bclc;
	alpha	&= 0x0f;
	beta	&= 0x3f;
	
	aclc = stx0288_readreg(state, R288_ACLC) & ~0x0f;
	bclc = stx0288_readreg(state, R288_BCLC) & ~0x3f;
	
	stx0288_writeregI(state, R288_ACLC, aclc | alpha);
	stx0288_writeregI(state, R288_BCLC, bclc | beta);
}

static inline void stx0288_set_fine(struct stx0288_state *state, int on)
{
	stx0288_writeregI(state, R288_ASCTRL, (stx0288_readreg(state, R288_ASCTRL) & 0xfd) | ((on) ? 0x02 : 0));
}

static inline int stx0288_set_FEC (struct stx0288_state* state, fe_code_rate_t fec)
{
	dprintk ("%s\n", __func__);

	stx0288_writeregI(state, R288_VITPROG, (stx0288_readreg(state, R288_VITPROG) & 0xfc) | 2);
	
	switch (fec) {
	case FEC_AUTO:
	{
		return stx0288_writeregI (state, R288_PR, 0x2f);
	}
	case FEC_1_2:
	{
		return stx0288_writeregI (state, R288_PR, 0x01);
	}
	case FEC_2_3:
	{
		return stx0288_writeregI (state, R288_PR, 0x02);
	}
	case FEC_3_4:
	{
		return stx0288_writeregI (state, R288_PR, 0x04);
	}
	case FEC_5_6:
	{
		return stx0288_writeregI (state, R288_PR, 0x08);
	}
	case FEC_6_7:
	{
		return stx0288_writeregI (state, R288_PR, 0x10);
	}
	case FEC_7_8:
	{
		return stx0288_writeregI (state, R288_PR, 0x20);
	}
	default:
	{
		return -EINVAL;
	}
    }
}

static inline u32 stx0288_get_mclk_div(struct stx0288_state *state)
{
	BUG_ON(!state->initialised);
	
	return state->mclk_div;
}

static inline void stx0288_set_derot_freq(struct stx0288_state *state, u32 DerotFreq_Hz)
{
	u32 temp;

	temp = DerotFreq_Hz / stx0288_get_mclk_div(state);
	
	stx0288_writeregI(state, R288_CFRM, MSB(temp));
	stx0288_writeregI(state, R288_CFRL, LSB(temp));
}

static inline void stx0288_set_noe(struct stx0288_state *state, int noe)
{
	noe &= 3;
	
	stx0288_writeregI(state, R288_ERRCTRL, (stx0288_readreg(state, R288_ERRCTRL) & 0xfb) | noe);
}

static int stx0288_get_frontend_algo(struct dvb_frontend* fe)
{
	return 1;
}

static int stx0288_tune(struct dvb_frontend* fe,
		    struct dvb_frontend_parameters* c,
		    unsigned int mode_flags,
		    unsigned int *delay,
		    fe_status_t *status)
{
	struct stx0288_state *state = fe->demodulator_priv;
	int timeout;
	s16 timing;
	u32 fmin, fmax;

	dprintk("%s : FE_SET_FRONTEND\n", __func__);
	
	if(c == NULL)
		goto get_status;

	if (fe->dtv_property_cache.delivery_system != SYS_DVBS) {
			dprintk("%s: unsupported delivery "
				"system selected (%d)\n",
				__func__, fe->dtv_property_cache.delivery_system);
			return -EOPNOTSUPP;
	}

	if (state->config->set_ts_params)
		state->config->set_ts_params(fe, 0);

	/* only frequency & symbol_rate are used for tuner*/
	if (fe->ops.tuner_ops.set_params) {
		fe->ops.tuner_ops.set_params(fe, c);
		if (fe->ops.i2c_gate_ctrl)
			fe->ops.i2c_gate_ctrl(fe, 0);
	}

	udelay(10);

	stx0288_set_frequency_offset_detector(state, 0);
	stx0288_set_autocenter(state, 1);
	
	timeout = 100;
	
	do {
		timing = stx0288_get_timing_loop_freq(state);
		
		if(ABS(timing) < 300)
			break;
		
		dprintk("%s: Timing=%d, timeout=%d\n", __func__, timing, timeout);
		
		msleep(1);
	} while(--timeout);
	
	stx0288_set_autocenter(state, 0);
	
	/** Pseudocode:
				
		u32 b = p->u.qpsk->symbol_rate / 2000
		u32 d = b
		b += 200000
		d += 30000
		
		u32 c = p->u.qpsk->symbol_rate / 1000
		
		b /= c = 2 + (200 / p->u.qpsk->symbol_rate)
		d /= c = 2 + (30 / p->u.qpsk->symbol_rate)
		
		stx0288_readreg(state, R288_VAVSRCH);
		
		...
	*/
	
	if(c->u.qpsk.symbol_rate >= 15000000)
		stx0288_set_noe(state, 3);
	else if(c->u.qpsk.symbol_rate >= 8000)
		stx0288_set_noe(state, 2);
	else if(c->u.qpsk.symbol_rate >= 3000)
		stx0288_set_noe(state, 1);
	else
		stx0288_set_noe(state, 0);
	
	stx0288_set_alpha_beta(state, 7, 28);
	stx0288_writeregI(state, R288_COARP1, 58);
	stx0288_set_froze_lock(state, 1);
	stx0288_writeregI(state, R288_ACCU1VAL, 0);
	stx0288_writeregI(state, R288_ACCU2VAL, (u8)-1);

	fmin = ((((u32)(c->u.qpsk.symbol_rate/(u32)1000))*((95*32768)/100))/(u32)(stx0288_get_mclk(state)/(u32)1000));   /* 2^15=32768*/
	fmax = ((((u32)(c->u.qpsk.symbol_rate/1000))*((105*32768)/100))/(u32)(stx0288_get_mclk(state)/1000));
	stx0288_writeregI(state, R288_FMINM, MSB(fmin) | 0x80);
	stx0288_writeregI(state, R288_FMINL, LSB(fmin));
	stx0288_writeregI(state, R288_FMAXM, MSB(fmax) | 0x80);
	stx0288_writeregI(state, R288_FMAXL, LSB(fmax));
	stx0288_writeregI(state, R288_FINEINC, c->u.qpsk.symbol_rate/1000000);
	stx0288_set_fine(state, 1);
	
	timeout = 100;
	
	while((stx0288_readreg(state, R288_ASCTRL) & 0x02) && (timeout--)) {
		dprintk("%s: Finetunning... (timeout=%d)\n", __func__, timeout);
		msleep(1);
	}
	
	stx0288_set_fine(state, 0);
	stx0288_set_derot_freq(state, c->u.qpsk.symbol_rate / 10);
	
	if(c->u.qpsk.symbol_rate < 5000)
		stx0288_set_alpha_beta(state, 8, 17);
	else if(c->u.qpsk.symbol_rate < 35000)
		stx0288_set_alpha_beta(state, 7, 28);
	else
		stx0288_set_alpha_beta(state, 8, 36);
	
	stx0288_set_symbolrate(fe, c->u.qpsk.symbol_rate);
	stx0288_set_FEC(state, c->u.qpsk.fec_inner);
	
	stx0288_set_frequency_offset_detector(state, 1);

	state->tuner_frequency = c->frequency;
	state->fec_inner = FEC_AUTO;
	state->symbol_rate = c->u.qpsk.symbol_rate;
	state->ucblocks = 0;
	
get_status:
	if (!(mode_flags & FE_TUNE_MODE_ONESHOT))
		stx0288_read_status(fe, status);
	*delay = HZ/10;

	return 0;
}

static int stx0288_i2c_gate_ctrl(struct dvb_frontend *fe, int enable)
{
	struct stx0288_state *state = fe->demodulator_priv;

	if (enable)
#ifdef CONFIG_STBx25xx
		stx0288_writeregI(state, 0x01, 0x95);
#else
		stx0288_writeregI(state, 0x01, 0xb5);
#endif
	else
#ifdef CONFIG_STBx25xx
		stx0288_writeregI(state, 0x01, 0x15);
#else
		stx0288_writeregI(state, 0x01, 0x35);
#endif
	

	udelay(1);

	return 0;
}

static void stx0288_release(struct dvb_frontend *fe)
{
	struct stx0288_state *state = fe->demodulator_priv;
	kfree(state);
}

static struct dvb_frontend_ops stx0288_ops = {

	.info = {
		.name			= "ST STx0288 DVB-S",
		.type			= FE_QPSK,
		.frequency_min		= 950000,
		.frequency_max		= 2150000,
		.frequency_stepsize	= 1000,	 /* kHz for QPSK frontends */
		.frequency_tolerance	= 0,
		.symbol_rate_min	= 1000000,
		.symbol_rate_max	= 45000000,
		.symbol_rate_tolerance	= 500,	/* ppm */
		.caps = FE_CAN_FEC_1_2 | FE_CAN_FEC_2_3 | FE_CAN_FEC_3_4 |
		      FE_CAN_FEC_5_6 | FE_CAN_FEC_7_8 |
		      FE_CAN_QPSK |
		      FE_CAN_FEC_AUTO
	},

	.release = stx0288_release,
	.init = stx0288_init,
	.sleep = stx0288_sleep,
	.write = stx0288_write,
	.i2c_gate_ctrl = stx0288_i2c_gate_ctrl,
	.read_status = stx0288_read_status,
	.read_ber = stx0288_read_ber,
	.read_signal_strength = stx0288_read_signal_strength,
	.read_snr = stx0288_read_snr,
	.read_ucblocks = stx0288_read_ucblocks,
	.diseqc_send_master_cmd = stx0288_send_diseqc_msg,
	.diseqc_send_burst = stx0288_send_diseqc_burst,
	.set_tone = stx0288_set_tone,
	.set_voltage = stx0288_set_voltage,

	.set_property = stx0288_set_property,
	.get_property = stx0288_get_property,
	.tune = stx0288_tune,
	.get_frontend_algo = stx0288_get_frontend_algo,
};

struct dvb_frontend *stx0288_attach(const struct stx0288_config *config,
				    struct i2c_adapter *i2c)
{
	struct stx0288_state *state = NULL;
	int id;

	/* allocate memory for the internal state */
	state = kzalloc(sizeof(struct stx0288_state), GFP_KERNEL);
	if (state == NULL)
		goto error;

	/* setup the state */
	state->config = config;
	state->i2c = i2c;
	state->initialised = 0;
	state->tuner_frequency = 0;
	state->symbol_rate = 0;
	state->fec_inner = 0;
	state->errmode = STATUS_BER;

	stx0288_writeregI(state, 0x41, 0x04);
	msleep(200);
	id = stx0288_readreg(state, 0x00);
	dprintk("stx0288 id %x\n", id);

	/* register 0x00 contains 0x11 for STx0288  */
	if (id != 0x11)
		goto error;

	/* create dvb_frontend */
	memcpy(&state->frontend.ops, &stx0288_ops,
			sizeof(struct dvb_frontend_ops));
	state->frontend.demodulator_priv = state;
	return &state->frontend;

error:
	kfree(state);

	return NULL;
}
EXPORT_SYMBOL(stx0288_attach);

module_param(debug_legacy_dish_switch, int, 0444);
MODULE_PARM_DESC(debug_legacy_dish_switch,
		"Enable timing analysis for Dish Network legacy switches");

module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Turn on/off frontend debugging (default:off).");

MODULE_DESCRIPTION("ST STx0288 DVB Demodulator driver");
MODULE_AUTHOR("Georg Acher, Bob Liu, Igor liplianin");
MODULE_LICENSE("GPL");

