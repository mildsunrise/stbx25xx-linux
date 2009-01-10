/*
 * Old U-boot compatibility for TM9101
 *
 * Author: Josh Boyer <jwboyer@linux.vnet.ibm.com>
 *
 * Copyright 2008 IBM Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 */

#include "ops.h"
#include "io.h"
#include "dcr.h"
#include "stdio.h"
#include "4xx.h"
#include "cuboot.h"
#define TARGET_4xx
#include "ppcboot.h"

static bd_t bd;

static void get_clocks(void)
{
	dt_fixup_cpu_clocks(bd.bi_procfreq, bd.bi_pci_busfreq, bd.bi_plb_busfreq);
	dt_fixup_clock("/plb/ebc", bd.bi_opbfreq);
	dt_fixup_clock("/plb/opb", bd.bi_opbfreq);
	dt_fixup_clock("/plb/opb/serial@40040000", 21000000);
	dt_fixup_clock("/plb/opb/serial@40000000", 21000000);
	dt_fixup_clock("/plb/opb/serial@40010000", 21000000);
}

static void tm9101_fixups(void)
{
	dt_fixup_memory(bd.bi_memstart, bd.bi_memsize);
	get_clocks();
}
	
void platform_init(unsigned long r3, unsigned long r4, unsigned long r5,
		unsigned long r6, unsigned long r7)
{
	CUBOOT_INIT();
	platform_ops.fixups = tm9101_fixups;
	platform_ops.exit = ibm40x_dbcr_reset;
	fdt_init(_dtb_start);
	serial_console_init();
}
