/*
 * Boot code for DM500
 *
 * Copyright 2015 Alba Mendez
 *   Based on cuboot-83xx.c, which is:
 * Copyright (c) 2007 Freescale Semiconductor, Inc.
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

BSS_STACK(4096);

void dm500_fixups(void)
{
}

void platform_init(void)
{
	unsigned long end_of_ram = 0x2000000;
	unsigned long avail_ram = end_of_ram - (unsigned long)_end;

	platform_ops.fixups = dm500_fixups;
	platform_ops.exit = ibm40x_dbcr_reset;

	simple_alloc_init(_end, avail_ram, 32, 64);
	fdt_init(_dtb_start);
	serial_console_init();
}
