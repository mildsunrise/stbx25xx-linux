/*
 * This file is part of linux driver the digital TV devices equipped with IBM STBx25xx SoC
 *
 * stbx25xx.h - private header file for all stbx25xx-chip-source files.
 *
 * see stbx25xx.c for copyright information.
 */
#ifndef __STBx25xx_H__
#define __STBx25xx_H___

#define FC_LOG_PREFIX "dvb_stbx25xx"
#include "stbx25xx_common.h"

extern int dvb_stbx25xx_debug;

/* debug */
#ifndef DBG_LEVEL
#define DBG_LEVEL 0
#endif

#if DBG_LEVEL > 0
#	define dprintk(args...) printk(KERN_DEBUG FC_LOG_PREFIX ": " args)
#else
#	define dprintk(args...)
#endif

#define deb_info(args...)  dprintk(0x01,args)
#define deb_tuner(args...) dprintk(0x02,args)
#define deb_i2c(args...)   dprintk(0x04,args)
#define deb_ts(args...)    dprintk(0x08,args)
#define deb_sram(args...)  dprintk(0x10,args)
#define deb_rdump(args...)  dprintk(0x20,args)

#endif
