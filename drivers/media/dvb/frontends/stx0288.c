/*
    Driver for ST STx0288 demodulator (mainly for Sharp BS2F7VZ0295 DVB-S tuner module)

    Copyright (C) 2001-2002 Convergence Integrated Media GmbH
	<ralph@convergence.de>,
	<holger@convergence.de>,
	<js@convergence.de>

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

struct stx0288_state {
	struct i2c_adapter *i2c;
	const struct stx0288_config *config;
	struct dvb_frontend frontend;

	u8 initialised:1;
	u32 tuner_frequency;
	u32 symbol_rate;
	fe_code_rate_t fec_inner;
	int errmode;
	u32 ucblocks;
};

#define STATUS_BER 0
#define STATUS_UCBLOCKS 1

static int debug;
static int debug_legacy_dish_switch;
#define dprintk(args...) \
	do { \
		if (debug) \
			printk(KERN_DEBUG "stx0288: " args); \
	} while (0)
	
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

static int stx0288_writeregI (struct stx0288_state* state, u8 reg, u8 data)
{
	int ret;
	u8 buf [] = { reg, data };
	struct i2c_msg msg = { .addr = state->config->demod_address, .flags = 0, .buf = buf, .len = 2 };

	ret = i2c_transfer (state->i2c, &msg, 1);

	if (ret != 1)
		dprintk("%s: writereg error (reg == 0x%02x, val == 0x%02x, "
			"ret == %i)\n", __func__, reg, data, ret);

	return (ret != 1) ? -EREMOTEIO : 0;
}

static int stx0288_write(struct dvb_frontend* fe, u8 *buf, int len)
{
	struct stx0288_state* state = fe->demodulator_priv;

	if (len != 2)
		return -EINVAL;

	return stx0288_writeregI(state, buf[0], buf[1]);
}

static u8 stx0288_readreg (struct stx0288_state* state, u8 reg)
{
	int ret;
	u8 b0 [] = { reg };
	u8 b1 [] = { 0 };
	struct i2c_msg msg [] = { { .addr = state->config->demod_address, .flags = 0, .buf = b0, .len = 1 },
			   { .addr = state->config->demod_address, .flags = I2C_M_RD, .buf = b1, .len = 1 } };

	ret = i2c_transfer (state->i2c, msg, 2);

	if (ret != 2)
		dprintk("%s: readreg error (reg == 0x%02x, ret == %i)\n",
				__func__, reg, ret);

	return b1[0];
}

static int stx0288_readregs (struct stx0288_state* state, u8 reg1, u8 *b, u8 len)
{
	int ret;
	struct i2c_msg msg [] = { { .addr = state->config->demod_address, .flags = 0, .buf = &reg1, .len = 1 },
			   { .addr = state->config->demod_address, .flags = I2C_M_RD, .buf = b, .len = len } };

	ret = i2c_transfer (state->i2c, msg, 2);

	if (ret != 2)
		dprintk("%s: readreg error (ret == %i)\n", __func__, ret);

	return ret == 2 ? 0 : ret;
}

#define DEMOD_XTAL_HZ	4000000
#define ABS(X) ((X)<0 ? (-(X)) : (X))
#define LSB(X) ((X & 0xFF))
#define MSB(Y) ((Y>>8)& 0xFF)
#define MAX(X,Y) ((X)>=(Y) ? (X) : (Y))
#define MIN(X,Y) ((X)<=(Y) ? (X) : (Y))
#define INRANGE(X,Y,Z) (((X<=Y) && (Y<=Z))||((Z<=Y) && (Y<=X)) ? 1 : 0)
#define BYTES2WORD(X,Y) ((X<<8)+(Y))

typedef enum FE_288_SIGNALTYPE {
	NOAGC1=0,
	AGC1OK,
	NOTIMING,
	ANALOGCARRIER,
	TIMINGOK,
	NOAGC2,
	AGC2OK,
	NOCARRIER,
	CARRIEROK,
	NODATA,
	FALSELOCK,
	DATAOK,
	OUTOFRANGE,
	RANGEOK
} FE_288_SIGNALTYPE_t;

static inline u32 stx0288_get_mclk(struct stx0288_state *state)
{
//Hz
	u32 pll_divider;   /* pll divider */
	u32 pll_selratio;  /* 4 or 6 ratio */
	u32 pll_bypass;    /* pll bypass */
	u32 ext_clk_hz;
	u8 reg40, reg41;

	ext_clk_hz=DEMOD_XTAL_HZ;
	reg41 = stx0288_readreg(state, 0x41);
	pll_bypass=reg41 & 1;
	
	if(pll_bypass)
		return ext_clk_hz;
	
	reg40 = stx0288_readreg(state, 0x40);
	pll_divider=reg40 + 1;
	pll_selratio=(((reg41 >> 2) & 1) ? 4 : 6);

	return (ext_clk_hz*pll_divider)/pll_selratio;
}

static inline u32 binary_float_div(u32 n1, u32 n2, int precision)
{
	int i=0;
	long result=0;
	/* division de N1 par N2 avec N1<N2 */
	while (i<=precision) { /* n1>0 */
		if (n1<n2 ) {
			result<<=1;
			n1<<=1;
		} else {
			result=(result<<1)+1;
			n1=(n1-n2)<<1;
		}
		i++;
	}

	return result;
}

static int stx0288_set_FEC (struct stx0288_state* state, fe_code_rate_t fec)
{
	dprintk ("%s\n", __func__);

	switch (fec) {
	case FEC_AUTO:
	{
		return stx0288_writeregI (state, 0x37, 0x2f);
	}
	case FEC_1_2:
	{
		return stx0288_writeregI (state, 0x37, 0x01);
	}
	case FEC_2_3:
	{
		return stx0288_writeregI (state, 0x37, 0x02);
	}
	case FEC_3_4:
	{
		return stx0288_writeregI (state, 0x37, 0x04);
	}
	case FEC_5_6:
	{
		return stx0288_writeregI (state, 0x37, 0x08);
	}
	case FEC_6_7:
	{
		return stx0288_writeregI (state, 0x37, 0x10);
	}
	case FEC_7_8:
	{
		return stx0288_writeregI (state, 0x37, 0x20);
	}
	default:
	{
		return -EINVAL;
	}
    }
}

static fe_code_rate_t stx0299_get_fec (struct stx0288_state* state)
{
	static fe_code_rate_t fec_tab [] = { FEC_1_2, FEC_2_3, FEC_3_4, FEC_5_6,
					     FEC_6_7, FEC_7_8 };
	u8 index;

	dprintk ("%s\n", __func__);

	index = stx0288_readreg (state, 0x24);
	index &= 0x7;

	if (index > 5)
		return FEC_AUTO;

	return fec_tab [index];
}

static int stx0288_set_symbolrate (struct dvb_frontend* fe, u32 srate)
{
	struct stx0288_state *state = fe->demodulator_priv;
	u32 temp;

	if ((srate < 1000000) || (srate > 45000000))
		return -EINVAL;

	temp = (unsigned int)srate / 1000;
	temp = binary_float_div(temp, stx0288_get_mclk(state), 20);
	
	stx0288_writeregI(state, 0x28, 0x80);
	stx0288_writeregI(state, 0x29, 0x00);
	stx0288_writeregI(state, 0x2a, 0x00);
	
	stx0288_writeregI(state, 0x28, temp >> 12);
	stx0288_writeregI(state, 0x29, temp >> 4);
	stx0288_writeregI(state, 0x2a, temp << 4);

	return 0;
}

static int stx0288_get_symbolrate (struct stx0288_state* state)
{
	u32 Mclk = stx0288_get_mclk(state) / 4096L;
	u32 srate;
	s32 offset;
	u8 sfr[3];
	s8 rtf[2];

	dprintk ("%s\n", __func__);

	stx0288_readregs (state, 0x28, sfr, 3);
	stx0288_readregs (state, 0x22, rtf, 2);

	srate = (sfr[0] << 8) | sfr[1];
	srate *= Mclk;
	srate /= 16;
	srate += (sfr[2] >> 4) * Mclk / 256;
	offset = (s32) ((rtf[0] << 8) | rtf[1]) * (srate / 4096L);
	offset /= 128;

	dprintk ("%s : srate = %i\n", __func__, srate);
	dprintk ("%s : ofset = %i\n", __func__, offset);

	srate += offset;

	srate += 1000;
	srate /= 2000;
	srate *= 2000;

	return srate;
}

static int stx0288_send_diseqc_msg (struct dvb_frontend* fe,
				    struct dvb_diseqc_master_cmd *m)
{
	struct stx0288_state* state = fe->demodulator_priv;
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

static int stx0288_send_diseqc_burst (struct dvb_frontend* fe, fe_sec_mini_cmd_t burst)
{
	struct stx0288_state* state = fe->demodulator_priv;

	dprintk("%s\n", __func__);

	if (stx0288_writeregI(state, 0x05, 0x16))/* burst mode */
		return -EREMOTEIO;

	if (stx0288_writeregI(state, 0x06, burst == SEC_MINI_A ? 0x00 : 0xff))
		return -EREMOTEIO;

	if (stx0288_writeregI(state, 0x06, 0x12))
		return -EREMOTEIO;

	return 0;
}

static int stx0288_set_tone (struct dvb_frontend* fe, fe_sec_tone_mode_t tone)
{
	struct stx0288_state* state = fe->demodulator_priv;

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

static int stx0288_set_voltage (struct dvb_frontend* fe, fe_sec_voltage_t volt)
{
	dprintk("%s: %s\n", __func__,
		volt == SEC_VOLTAGE_13 ? "SEC_VOLTAGE_13" :
		volt == SEC_VOLTAGE_18 ? "SEC_VOLTAGE_18" : "??");

	return 0;
}

static int stx0288_init (struct dvb_frontend* fe)
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
	
	return 0;
}

static int stx0288_read_status(struct dvb_frontend* fe, fe_status_t* status)
{
	struct stx0288_state* state = fe->demodulator_priv;

	u8 signal = 0xff - stx0288_readreg (state, 0x18);
	u8 sync = stx0288_readreg (state, 0x1b);

	dprintk ("%s : FE_READ_STATUS : VSTATUS: 0x%02x\n", __func__, sync);
	*status = 0;

	if (signal > 10)
		*status |= FE_HAS_SIGNAL;

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

static int stx0288_read_ber(struct dvb_frontend* fe, u32* ber)
{
	struct stx0288_state* state = fe->demodulator_priv;

	if (state->errmode != STATUS_BER)
		return 0;
	*ber = (stx0288_readreg(state, 0x26) << 8) |
					stx0288_readreg(state, 0x27);
	dprintk("stv0288_read_ber %d\n", *ber);

	return 0;
}

static int stx0288_read_signal_strength(struct dvb_frontend* fe, u16* strength)
{
	struct stx0288_state* state = fe->demodulator_priv;

	s32 signal =  0xffff - ((stx0288_readreg(state, 0x10) << 8));


	signal = signal * 5 / 4;
	*strength = (signal > 0xffff) ? 0xffff : (signal < 0) ? 0 : signal;
	dprintk("stx0288_read_signal_strength %d\n", *strength);

	return 0;
}

static int stx0288_read_snr(struct dvb_frontend* fe, u16* snr)
{
	struct stx0288_state* state = fe->demodulator_priv;

	s32 xsnr = 0xffff - ((stx0288_readreg(state, 0x2d) << 8)
			   | stx0288_readreg(state, 0x2e));
	xsnr = 3 * (xsnr - 0xa100);
	*snr = (xsnr > 0xffff) ? 0xffff : (xsnr < 0) ? 0 : xsnr;
	dprintk("stx0288_read_snr %d\n", *snr);

	return 0;
}

static int stx0288_read_ucblocks(struct dvb_frontend* fe, u32* ucblocks)
{
	struct stx0288_state* state = fe->demodulator_priv;

	if (state->errmode != STATUS_BER)
		return 0;
	*ucblocks = (stx0288_readreg(state, 0x26) << 8) |
					stx0288_readreg(state, 0x27);
	dprintk("stx0288_read_ber %d\n", *ucblocks);

	return 0;
}

static void stx0288_freeze_lock(struct stx0288_state* state, int on)
{
	stx0288_writeregI(state, 0x50, (stx0288_readreg(state, 0x50) & 0xdf) | ((on != 0) << 5));
}

static void stx0288_set_derot_freq(struct stx0288_state* state, u32 DerotFreq_Hz)
{
	u32 temp;
	u32 MasterClock_Hz;
	int F288_CARRIER_FREQUENCY_MSB;
	int F288_CARRIER_FREQUENCY_LSB;
	
	MasterClock_Hz = stx0288_get_mclk(state); //Hz
	temp= DerotFreq_Hz / (MasterClock_Hz / 65536);
	F288_CARRIER_FREQUENCY_MSB = (temp >> 8) & 0xff;
	F288_CARRIER_FREQUENCY_LSB = temp & 0xff;
	
	stx0288_writeregI(state, 0x2b, F288_CARRIER_FREQUENCY_MSB);
	stx0288_writeregI(state, 0x2c, F288_CARRIER_FREQUENCY_LSB);
}

static void stx0288_set_frequency_offset_detector(struct stx0288_state* state, int flag)
{
	stx0288_writeregI(state, 0x15, (stx0288_readreg(state, 0x15) & 0x7f) | ((flag !=0 ) << 7));
}

static void stx0288_start_coarse_algo(struct stx0288_state* state, int flag)
{
	stx0288_writeregI(state, 0x50, (stx0288_readreg(state, 0x50) & 0xfe) | (flag != 0));
}

static u32 stx0288_get_derot_freq(struct stx0288_state* state)
{
	u32 derotmsb;
	u32 derotlsb;
	u32 fm;
	u32 dfreq;
	u32 Itmp;

	derotmsb=stx0288_readreg(state, 0x2b);
	derotlsb=stx0288_readreg(state, 0x2b);
	fm = stx0288_get_mclk(state);

	Itmp = (derotmsb << 8) + derotlsb;
	if (Itmp > 0x10000/2) {
		//2's complement
		Itmp = 0x10000 - Itmp;
		Itmp *= -1;
	}

	dfreq = Itmp * (fm / 10000L);
	dfreq = dfreq / 65536L;
	dfreq *= 10;
	
	return dfreq;
}

static u32 stx0288_coarse(struct stx0288_state* state, u32 *Offset_Khz)
{
	u32 symbolrate_Bds = 0;
	int F288_AUTOCENTRE=1;
	int F288_FINE=0;
	int F288_COARSE=1;

	stx0288_writeregI(state, 0x50, (stx0288_readreg(state, 0x50) & 0xf8)
		| (F288_AUTOCENTRE << 2) | (F288_FINE << 1) | (F288_COARSE << 0));

	msleep(50);
	
	symbolrate_Bds = stx0288_get_symbolrate(state);

	*Offset_Khz = stx0288_get_derot_freq(state);
	stx0288_start_coarse_algo(state, 0);
	/* stop coarse algorithm */
	dprintk("  (demod_288_coarse: symbolrate_Bds=%u )\n",symbolrate_Bds);
	dprintk("  (demod_288_coarse: *Offset_Khz=%u)\n",*Offset_Khz);

	//stx0288_start_coarse_algo(state, 0);
	/* stop coarse algorithm */

	return symbolrate_Bds;
}

static void stx0288_start_fine_algo(struct stx0288_state* state, int flag)
{
	stx0288_writeregI(state, 0x50, (stx0288_readreg(state, 0x50) & 0xfd) | ((flag != 0) << 1));
}

static void stx0288_fine(struct stx0288_state* state, u32 Symbolrate_Bds, int Known)
{
	int F288_FINE;
	int flag;
	int i=0;
	u32 fmin=0;
	u32 fmax=0;
	u32 MasterClock_Hz;
	
	MasterClock_Hz=stx0288_get_mclk(state);

	if (Known) {
		/* +/- 1% search range */
		fmin=((Symbolrate_Bds/1000)*32440) /(MasterClock_Hz/1000); /* 2^15=32768*/
		fmax=((Symbolrate_Bds/1000)*36044) /(MasterClock_Hz/1000);
		/* +/- 5% search range */
		// fmin=long( ((Symbolrate_Bds/1000)*0.95*32768) /(MasterClock_Hz/1000)); /* 2^15=32768*/
		// fmax=long( ((Symbolrate_Bds/1000)*1.05*32768) /(MasterClock_Hz/1000));
		// t.Format("::demod_288_fine[Known]/fmin=%.4X,fmax=%.4X",fmin,fmax); message(t);
	} else {
		/* +/- 15% search range */
		fmin=((Symbolrate_Bds/1000)*27852)/(MasterClock_Hz/1000); /* 2^15=32768*/
		fmax=((Symbolrate_Bds/1000)*37683)/(MasterClock_Hz/1000);
		/* fine loop start from high limit when symbol rate is unknown */
		stx0288_set_symbolrate(&state->frontend, Symbolrate_Bds + (Symbolrate_Bds/10));
	}

	stx0288_writeregI(state, 0x53, (1<<7)|MSB(fmin));
	stx0288_writeregI(state, 0x54, LSB(fmin));
	stx0288_writeregI(state, 0x55, MSB(fmax));
	stx0288_writeregI(state, 0x56, LSB(fmax));
	stx0288_writeregI(state, 0x57, MAX( (Symbolrate_Bds/1000000),1 ));

	stx0288_start_fine_algo(state, 1); 
	/* start fine algorithm */

	i=0;

	do {
		msleep(10);

		i++;
		F288_FINE = stx0288_readreg(state, 0x50);
		flag=(F288_FINE>>1)&0x01;

		dprintk("  (demod_288_fine/[%d] flag=%d)\n",i,flag);
	} while ( (flag) && (i < 100) ); 
	/* wait for end of fine algorithm */
	// <- loop num is 100 times in spite of wait time modified :1ms -> 10ms

	//}while(flag); /* wait for end of fine algorithm */

	stx0288_start_fine_algo(state, 0);
}

static u32 stx0288_get_timing_loop(struct stx0288_state* state)
{
	int F288_TIMING_LOOP_FREQ_MSB;
	int F288_TIMING_LOOP_FREQ_LSB;
	long timing;
	
	F288_TIMING_LOOP_FREQ_MSB = stx0288_readreg(state, 0x22);
	F288_TIMING_LOOP_FREQ_LSB = stx0288_readreg(state, 0x23);

	timing = BYTES2WORD(F288_TIMING_LOOP_FREQ_MSB, F288_TIMING_LOOP_FREQ_LSB);
	
	//t.Format("  (demod_288_get_timing_loop: timing=%ld \t(%.8X) )",timing,timing); message(t);
	
	if (timing>0x10000/2) {
		//2's complement
		timing=0x10000-timing;
		timing*=-1;
	}
	
	return timing;
}

static void stx0288_start_autocenter_algo(struct stx0288_state* state, int flag)
{
	stx0288_writeregI(state, 0x50, (stx0288_readreg(state, 0x50) & 0xfb) | ((flag != 0) << 2));
}

static u32 stx0288_autocenter(struct stx0288_state* state)
{
	s32 timeout = 0, timing;
	//  int address;
	//  address=0x22;
	//  int F288_TIMING_LOOP_FREQ_MSB;
	//  int F288_TIMING_LOOP_FREQ_LSB;
	stx0288_start_autocenter_algo(state, 1);
	/* Start autocentre algorithm */
	do {
		dprintk("  (%.3d )\n", timeout);

		msleep(10); /* wait 10 ms */

		timeout++;
		timing = stx0288_get_timing_loop(state);
		//      demod_getdata(address,&F288_TIMING_LOOP_FREQ_MSB);
		//      demod_getdata(address+1,&F288_TIMING_LOOP_FREQ_LSB);
		//      timing=BYTES2WORD(F288_TIMING_LOOP_FREQ_MSB,F288_TIMING_LOOP_FREQ_LSB);
		//t.Format("  (demod_288_autocenter: timing=%ld \t(%.8X) )",timing,timing); message(t);
		//
		//      if(timing>0x10000/2) {
		//2's complement
		//          timing=0x10000-timing;
		//          timing*=-1;
		//      }

		dprintk("  (demod_288_autocenter: timing=%d \t(%.8X) )\n", timing, timing);
	} while ((ABS(timing) > 300) && (timeout < 100));
	/* timing loop is centered or timeout limit is reached */
	stx0288_start_autocenter_algo(state, 0);
	/* Stop autocentre algorithm */

	return timing;
}

static int stx0288_get_cf(struct stx0288_state* state)
{
	int CF;
	int data;
	
	data = stx0288_readreg(state, 0x24);
	CF = (data >> 7) & 1;
	
	return CF;
}

static int stx0288_set_frontend(struct dvb_frontend* fe, struct dvb_frontend_parameters * p)
{
	struct stx0288_state* state = fe->demodulator_priv;
	//int address;
	int flag=0;
	u32 MasterClock_Hz;
	u32 tunfreq_Khz; //kHz
	u32 SearchFreq_Khz; //kHz
	u32 SymbolRate_Bds; //sps??
	u32 coarseOffset_Khz=0;
	//u32 carrierOffset_Khz=0;
	u32 coarseSymbolRate_Bds=0;
	// u32 timeout;
	u32 tdata;
	u32 kt;
	// u32 nbErr;
	// u32 maxPckError;
	u32 timing = 0;
	// int retry = 0;
	int symbolrate_ok;
	int lock=0;
	int known = 0;
	int direcTV = 0;
	// int enabledRates;
	int tunerIQ;
	u32 MinOffset_Khz=-5000; //-5MHz
	u32 MaxOffset_Khz=5000; //+5MHz
	//---pIntResults---
	u32 pIntResults_SymbolRate_Bds;
	fe_code_rate_t pIntResults_PunctureRate;
	u32 pIntResults_Frequency_Khz;
	//-----------------
	FE_288_SIGNALTYPE_t pIntResults_SignalType;
	FE_288_SIGNALTYPE_t signalType;
	int F288_FECMODE;
	int F288_ALPHA=7;
	int F288_BETA=28;
	int F288_SYM;
	int F288_IND1_ACC=0;
	int F288_IND2_ACC=0xFF;
	int F288_TMG_LOCK,F288_CF;
	int iii;
	u32 lambda_4; //sps
	u32 offset;

	dprintk ("%s : FE_SET_FRONTEND\n", __func__);
	if (state->config->set_ts_params)
		state->config->set_ts_params(fe, 0);
	
	SearchFreq_Khz=p->frequency; //kHz
	SymbolRate_Bds=p->u.qpsk.symbol_rate; //sps?

	signalType=NOAGC1;

	dprintk("SEARCH>> FE_288_Algo::Begin\n");
	dprintk("SEARCH>> FE_288_Algo::Searched frequency=%d MHz\n", SearchFreq_Khz / 1000);
	dprintk("SEARCH>> FE_288_Algo::Search range=+/-%d MHz\n", MaxOffset_Khz / 1000);

	if (fe->ops.tuner_ops.set_params) {
		fe->ops.tuner_ops.set_params(fe, p);
		if (fe->ops.i2c_gate_ctrl) fe->ops.i2c_gate_ctrl(fe, 0);
	}
	
	tunfreq_Khz = SearchFreq_Khz;
	
	if (fe->ops.tuner_ops.get_frequency) {
		fe->ops.tuner_ops.get_frequency(fe, &tunfreq_Khz); /* Read tuner frequency */
		if (fe->ops.i2c_gate_ctrl) fe->ops.i2c_gate_ctrl(fe, 0);
	}
	
	/* ( Check tuner status ) */
	msleep(100);

	tunerIQ = (p->inversion == INVERSION_ON) ? -1 : 1; /* Get tuner IQ wiring */
	kt = 56;
	known = (SymbolRate_Bds!=0);
	coarseOffset_Khz=tunerIQ*(SearchFreq_Khz-tunfreq_Khz); /* begin coarse algorithm with tuner residual offset */

	F288_FECMODE = stx0288_readreg(state, 0x30);
	F288_FECMODE >>= 4;
	F288_FECMODE &= 0x0f;
	direcTV = (F288_FECMODE == 4); /* Store current FEC mode */

	do {
		/* ( Set maximum tuner bandwidth ) TunerSetBandwidth(hTuner,50000000)*/
		dprintk("SEARCH>> FE_288_Algo::Kt=%d\n",kt);

		/* Setup of non-modified parameters */
		F288_ALPHA = 7;
		F288_BETA = 28;

		stx0288_writeregI(state, 0x16, (stx0288_readreg(state, 0x16) & 0xf0) | F288_ALPHA);
		stx0288_writeregI(state, 0x17, (stx0288_readreg(state, 0x17) & 0xc0) | F288_BETA);

		if (direcTV) {
			F288_SYM = 0;
			/* force IQ normal */
			stx0288_writeregI(state, 0x30, (stx0288_readreg(state, 0x30) & 0xfe) | F288_SYM);
		}

		/* Set Kt value */
		stx0288_writeregI(state, 0x51, kt);
		stx0288_freeze_lock(state, 1);

		F288_IND1_ACC = 0;
		F288_IND2_ACC = 0xFF;
		stx0288_writeregI(state, 0x5e, F288_IND1_ACC);
		stx0288_writeregI(state, 0x5f, F288_IND2_ACC);

		stx0288_set_symbolrate(fe, 1000000); //sps
		/* Set symbolrate to 1.000MBps (minimum  symbol rate) */
		stx0288_set_derot_freq(state, coarseOffset_Khz*1000); //Hz
		/* Set carrier loop offset to 0KHz or previous iteration value */
		stx0288_set_frequency_offset_detector(state, 1);
		/* Frequency offset detector on */
		coarseSymbolRate_Bds = stx0288_coarse(state, &coarseOffset_Khz);
		/* Symbol rate coarse search */
		coarseOffset_Khz *= tunerIQ;

		dprintk("SEARCH>> FE_288_Algo::dF=%u\n", stx0288_get_derot_freq(state));
		
		//V0.05[061204] [TILT2]
		//in manual search mode,
		//move coarseOffset_Khz value as Fs/4, if coarseOffset_Khz is out of range(+/-5MHz)
		if (1) {
			if (known) {
				//manual search mode
				dprintk(" [061201_test_tilt]  coarseOffset_Khz=%u\n",coarseOffset_Khz);
				
				if (coarseOffset_Khz < MinOffset_Khz) {
					dprintk(" [061201_test_tilt] %u + [%u]\n",coarseOffset_Khz,SymbolRate_Bds/4000);
					coarseOffset_Khz+=SymbolRate_Bds/4000;
				} else if (MaxOffset_Khz < coarseOffset_Khz) {
					dprintk(" [061201_test_tilt] %u - [%u]\n",coarseOffset_Khz,SymbolRate_Bds/4000);
					coarseOffset_Khz-=SymbolRate_Bds/4000;
				}
				dprintk(" [061201_test_tilt] coarseOffset_Khz='%u\n",coarseOffset_Khz);
				
				//
				//set the derot value with moved "coarseOffset_Khz" value
				//demod_288_set_derot_freq(coarseOffset_Khz*1000,register_);
				//Hz
				/* Set carrier loop offset to 0KHz or previous iteration value */
				//demod_288_set_frequency_offset_detector(1,register_);
				/* Frequency offset detector on */
				//Sleep(50);
			}
		}
		
		dprintk("SEARCH>> FE_288_Algo::dF=%u\n", stx0288_get_derot_freq(state));
		
		if (known)	/* symbol rate is already known, so keep only the offset and force the symbol rate */
			stx0288_set_symbolrate(fe, SymbolRate_Bds);
		else		/* take into account the symbol rate returned by the coarse algorithm */
			SymbolRate_Bds = coarseSymbolRate_Bds;

		dprintk("SEARCH>> FE_288_Algo::SymbolRate=%d Kbds\n",SymbolRate_Bds/1000);
		dprintk("SEARCH>> FE_288_Algo::Offset=%d KHz\n",coarseOffset_Khz);
		
		MasterClock_Hz = stx0288_get_mclk(state);
		if (SymbolRate_Bds > 1000) {
			symbolrate_ok = (MasterClock_Hz / (SymbolRate_Bds / 1000)) > 2100;
			/* to avoid a divide by zero error */
		} else {
			symbolrate_ok = 1;
		}

		if ( (SymbolRate_Bds >= 1000000)             /* Check min symbolrate value */
		        //<--for 1Msps [060529]
		        //if(   (SymbolRate_Bds > 1000000)               /* Check min symbolrate value */
		        &&  (coarseOffset_Khz>=MinOffset_Khz)    /* Check minimum derotator offset criteria */
		        &&  (coarseOffset_Khz<MaxOffset_Khz ) /* Check maximum derotator offset criteria */ && (symbolrate_ok))
			/* Check shannon criteria */
		{
			//V0.05[061204] [TILT5]
			//search with false lock loop
			//-="-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-="
			iii = 0;
			lambda_4 = SymbolRate_Bds / 4; //sps
			
			do {
				//dprintk(" ");
				//dprintk("[061201_test_tilt] False Lock loop [ %d ] ",iii);
				//dprintk(" ");
				
				dprintk("[ %d ]\n",iii);
				
				//[
				//false lock search loop setting.
				//if carrier/timing lock and FEC unlock -> junm to the next false lock point(+/- Fs/4)
				if (iii==0)
					offset=coarseOffset_Khz*1000; //Hz
				if (iii==1)
					offset=coarseOffset_Khz*1000 + lambda_4; //Hz
				if (iii==2)
					offset=coarseOffset_Khz*1000 - lambda_4; //Hz
					
				stx0288_set_derot_freq(state, offset); //Hz
				stx0288_set_frequency_offset_detector(state, 1);
				
				/* Frequency offset detector on */
				msleep(50);

				dprintk("[061201_test_tilt]  coarseOffset_Khz*1000 = %u [MHz]\n", coarseOffset_Khz / 1000);
				dprintk("[061201_test_tilt]  lambda_4 = %u [MHz]\n", lambda_4 / 1000000);
				dprintk("[061201_test_tilt]  DEROT = %u [MHz]\n", offset / 1000000);
				
				//________________________________________________________________________________
				//ORIGINAL ORIGINAL ORIGINAL ORIGINAL ORIGINAL ORIGINAL ORIGINAL ORIGINAL ORIGINAL
				//              if(known)
				//                  TunerSetBandwidth(hTuner,(SymbolRate_Bds*1.4)+4000000); /* Change tuner bandwidth */
				//              else
				//                  TunerSetBandwidth(hTuner,SymbolRate_Bds*3); /* Change tuner bandwidth */

				/* Adjust carrier loop setting */
				if (SymbolRate_Bds < 5000000) {
					F288_ALPHA = 8;
					F288_BETA = 17;
				} else if (SymbolRate_Bds > 35000000) {
					F288_ALPHA = 8;
					F288_BETA = 36;
				}
				stx0288_writeregI(state, 0x16, (stx0288_readreg(state, 0x16) & 0xf0) | F288_ALPHA);
				stx0288_writeregI(state, 0x17, (stx0288_readreg(state, 0x17) & 0xc0) | F288_BETA);

				stx0288_freeze_lock(state, 0);

				stx0288_fine(state, SymbolRate_Bds, known);

				//V0.05[061204] [TILT3]
				//need wait time?? (CARRIER : UNLOCK -> LOCK)
				msleep(50);

				F288_TMG_LOCK = stx0288_readreg(state, 0x1e) & 0x80;
				F288_CF = stx0288_readreg(state, 0x24) & 0x80;

				//V0.05[061204] [TILT6]
				//if timing and carrier OK, lock or false lock.
				// -> memorize the demod_288_get_derot_freq() into coarseOffset_Khz .
				if (F288_TMG_LOCK&&F288_CF)
					coarseOffset_Khz = stx0288_get_derot_freq(state);

				dprintk("SEARCH>> FE_288_Algo::Timing=%s,Carrier=%s\n",
					 F288_TMG_LOCK ? "LOCK" : "UNLOCK",
					 F288_CF ? "LOCK" : "UNLOCK");
				dprintk("SEARCH>> FE_288_Algo::dF=%u\n", stx0288_get_derot_freq(state));
				
				stx0288_set_frequency_offset_detector(state, 0);
				/* Frequency offset detector off */
				timing = stx0288_autocenter(state);
				
				if ( !stx0288_get_cf(state) && (SymbolRate_Bds > 18000000)) {
					stx0288_set_symbolrate(fe, SymbolRate_Bds);
					stx0288_set_frequency_offset_detector(state, 1);
					stx0288_set_derot_freq(state, 0);
					stx0288_set_frequency_offset_detector(state, 0);
					timing = stx0288_autocenter(state);
				}
				
				dprintk("SEARCH>> FE_288_Algo::Timing offset=%d(%.2X)\n", ABS(timing), ABS(timing));
				
				if (ABS(timing) <= 300) {
					pIntResults_SymbolRate_Bds = stx0288_get_symbolrate(state);
					tdata = 10 + (2 * stx0288_data_timing_constant(state, pIntResults_SymbolRate_Bds));

					lock=stx0288_waitlock(state, tdata);

					if (lock)
						pIntResults_PunctureRate = stx0299_get_fec(state);
					
					// if (direcTV) { }
					// if(direcTV)
					// {
					//
					/* workaround for IQ invertion in DIRECTV mode */
					// WAIT_N_MS(4);
					// nbErr="FE_288_GetErrorCount(hDemod,COUNTER2);"
					// maxPckError="0x01<<(11+2*ChipGetField(hDemod,F288_NOE2));"
					/* packet error rate="50%" */
					// if((!lock) || (lock && (pIntResults->PunctureRate == FE_6_7) && (nbErr>maxPckError)))
					//                      {
					//                          /* more than 50% packet errors rate ==> swap I and Q */
					//
					//                          #if defined(DBG_SEARCHALGO) && !defined(NO_GUI)
					//                              ReportInsertMessage(text);
					//                          #endif
					//
					//                          ChipSetField(hDemod,F288_SYM,1);
					//                          lock=FE_288_WaitLock(hDemod,tdata);
					//                      }
					//                  }
					//

					dprintk("SEARCH>> FE_288_Algo::FEC=%s\n", lock ? "LOCK" : "UNLOCK");
					
					if (lock) {
						//V0.02[060613] invert the polarity
						/* update results */
						
						tunfreq_Khz = SearchFreq_Khz;
						
						if (fe->ops.tuner_ops.get_frequency) {
							fe->ops.tuner_ops.get_frequency(fe, &tunfreq_Khz); /* Read tuner frequency */
							if (fe->ops.i2c_gate_ctrl) fe->ops.i2c_gate_ctrl(fe, 0);
						}
						
						pIntResults_Frequency_Khz = tunfreq_Khz - (tunerIQ * stx0288_get_derot_freq(state));
						// pIntResults_Frequency_Khz=tun_getfreq(byte_)+(tunerIQ*demod_288_get_derot_freq(register_));

						dprintk("SEARCH>> FE_288_Algo::Transponder freq=%u MHz\n", pIntResults_Frequency_Khz / 1000);
						
						if (ABS(SearchFreq_Khz - pIntResults_Frequency_Khz) > ABS(MaxOffset_Khz)) {
							signalType = OUTOFRANGE;
						} else {
							signalType = RANGEOK;
							//V0.04[061109] [STB0288+ix2410]
							//set lock status for return value
							flag=1;
							//
						}
						
						pIntResults_SignalType = signalType;
						
						state->tuner_frequency	= pIntResults_Frequency_Khz;
						state->symbol_rate	= pIntResults_SymbolRate_Bds;
						state->fec_inner	= pIntResults_PunctureRate;
					}
				}
				//ORIGINAL ORIGINAL ORIGINAL ORIGINAL ORIGINAL ORIGINAL ORIGINAL ORIGINAL ORIGINAL
				//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

				dprintk("[061201_test_tilt] lock,TMG,CF = %d,%d,%d\n", lock, stx0288_check_TMG_LOCK(), stx0288_check_CF() );
				
				if (known)
					iii+=1; 
				//manual search : false lock loop, next derot.
				else
					iii=3;
				//blind search mode : loop finish
			} while ( (lock==0) && stx0288_check_TMG_LOCK() && stx0288_check_CF() && (iii<=2) );
			//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
		} else {
			stx0288_freeze_lock(state, 0);
			dprintk("  ( FE_288_Algo:: ABS(timing)<=300:NG )\n");
		}
		
		kt-=10; /* decrease Kt for next trial */
	} while ((signalType != RANGEOK) && (signalType != OUTOFRANGE) && (symbolrate_ok) && (kt>=46));

	printk("SEARCH>> FE_288_Algo::End >>ret=%d\n", flag);

	return 0;
}

/*
static int stx0288_get_frontend(struct dvb_frontend* fe, struct dvb_frontend_parameters * p)
{
	struct stx0288_state* state = fe->demodulator_priv;
	s32 derot_freq;
	int invval;

	derot_freq = (s32)(s16) ((stx0288_readreg (state, 0x22) << 8)
				| stx0288_readreg (state, 0x23));

	derot_freq *= (state->config->mclk >> 16);
	derot_freq += 500;
	derot_freq /= 1000;

	p->frequency += derot_freq;

	invval = stx0288_readreg (state, 0x0c) & 1;
	if (state->config->invert) invval = (~invval) & 1;
	p->inversion = invval ? INVERSION_ON : INVERSION_OFF;

	p->u.qpsk.fec_inner = stx0288_get_fec (state);
	p->u.qpsk.symbol_rate = stx0288_get_symbolrate (state);

	return 0;
}
*/

static int stx0288_sleep(struct dvb_frontend* fe)
{
	struct stx0288_state* state = fe->demodulator_priv;

	stx0288_writeregI(state, 0x41, 0x84);
	state->initialised = 0;

	return 0;
}

static int stx0288_i2c_gate_ctrl(struct dvb_frontend* fe, int enable)
{
	struct stx0288_state* state = fe->demodulator_priv;

	if (enable) {
		stx0288_writeregI(state, 0x01, 0x95);
	} else {
		stx0288_writeregI(state, 0x01, 0x15);
	}
	udelay(1);
	return 0;
}

static int stx0288_get_tune_settings(struct dvb_frontend* fe, struct dvb_frontend_tune_settings* fesettings)
{
	struct stx0288_state* state = fe->demodulator_priv;

	fesettings->min_delay_ms = state->config->min_delay_ms;
	if (fesettings->parameters.u.qpsk.symbol_rate < 10000000) {
		fesettings->step_size = fesettings->parameters.u.qpsk.symbol_rate / 32000;
		fesettings->max_drift = 5000;
	} else {
		fesettings->step_size = fesettings->parameters.u.qpsk.symbol_rate / 16000;
		fesettings->max_drift = fesettings->parameters.u.qpsk.symbol_rate / 2000;
	}
	return 0;
}

static void stx0288_release(struct dvb_frontend* fe)
{
	struct stx0288_state* state = fe->demodulator_priv;
	kfree(state);
}

static struct dvb_frontend_ops stx0288_ops;

struct dvb_frontend* stx0288_attach(const struct stx0288_config* config,
				    struct i2c_adapter* i2c)
{
	struct stx0288_state *state = NULL;
	int id;

	/* allocate memory for the internal state */
	state = kmalloc(sizeof(struct stx0288_state), GFP_KERNEL);
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

	/* register 0x00 contains 0x11 for STV0288  */
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

static struct dvb_frontend_ops stx0288_ops = {

	.info = {
		.name			= "ST STx0288 DVB-S",
		.type			= FE_QPSK,
		.frequency_min		= 950000,
		.frequency_max		= 2150000,
		.frequency_stepsize	= 125,	 /* kHz for QPSK frontends */
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

	.set_frontend = stx0288_set_frontend,
//	.get_frontend = stx0288_get_frontend,
	.get_tune_settings = stx0288_get_tune_settings,

	.read_status = stx0288_read_status,
	.read_ber = stx0288_read_ber,
	.read_signal_strength = stx0288_read_signal_strength,
	.read_snr = stx0288_read_snr,
	.read_ucblocks = stx0288_read_ucblocks,

	.diseqc_send_master_cmd = stx0288_send_diseqc_msg,
	.diseqc_send_burst = stx0288_send_diseqc_burst,
	.set_tone = stx0288_set_tone,
	.set_voltage = stx0288_set_voltage,
};

module_param(debug_legacy_dish_switch, int, 0444);
MODULE_PARM_DESC(debug_legacy_dish_switch, "Enable timing analysis for Dish Network legacy switches");

module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Turn on/off frontend debugging (default:off).");

MODULE_DESCRIPTION("ST STx0288 DVB Demodulator driver");
MODULE_AUTHOR("Ralph Metzler, Holger Waechtler, Peter Schildmann, Felix Domke, "
	      "Andreas Oberritter, Andrew de Quincey, Kenneth Aafly");
MODULE_LICENSE("GPL");

EXPORT_SYMBOL(stx0288_attach);
