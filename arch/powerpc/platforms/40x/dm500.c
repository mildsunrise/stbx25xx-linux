/*
 * Generic PowerPC 40x platform support
 *
 * Copyright 2008 IBM Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; version 2 of the License.
 *
 * This implements simple platform support for PowerPC 44x chips.  This is
 * mostly used for eval boards or other simple and "generic" 44x boards.  If
 * your board has custom functions or hardware, then you will likely want to
 * implement your own board.c file to accommodate it.
 */

#include <asm/machdep.h>
#include <asm/pci-bridge.h>
#include <asm/dm500.h>
#include <asm/ppc4xx.h>
#include <asm/prom.h>
#include <asm/time.h>
#include <asm/udbg.h>
#include <asm/uic.h>
#include <asm/dcr.h>
#include <asm/dcr-regs.h>
#include <asm/reg.h>

#include <linux/init.h>
#include <linux/of_platform.h>

int _board_is_dm500 = 0;
EXPORT_SYMBOL(_board_is_dm500);

static __initdata struct of_device_id dm500_of_bus[] = {
	{ .compatible = "ibm,plb3", },
	{ .compatible = "ibm,opb", },
	{ .compatible = "ibm,ebc", },
	{},
};

struct resource ne2000_resources[] = {
        [0] = { .flags  = IORESOURCE_IO },
        [1] = { .flags  = IORESOURCE_IRQ },
};

static int __init dm500_device_probe(void)
{
	struct device_node *np;

	of_platform_bus_probe(NULL, dm500_of_bus, NULL);

	/* Pass parameters to NE2000 driver */
	if ((np = of_find_compatible_node(NULL, NULL, "novell,ne2000"))) {
		ne2000_resources[0].start = of_iomap(np, 0);
		ne2000_resources[0].end = ne2000_resources[0].start + 0x3f;
		ne2000_resources[1].start = irq_of_parse_and_map(np, 0);
		of_node_put(np);
		platform_device_register_simple("ne", -1, ne2000_resources, ARRAY_SIZE(ne2000_resources));
	}

	return 0;
}
machine_device_initcall(dm500, dm500_device_probe);

static int __init dm500_probe(void)
{
	unsigned long root = of_get_flat_dt_root();

	if (!of_flat_dt_is_compatible(root, "dreambox,dm500"))
		return 0;

	_board_is_dm500 = 1;

	return 1;
}

/* The original ppc4xx_reset_system applies a system
   reset, that doesn't work. */
void dm500_reset_system(char *cmd)
{
	mtspr(SPRN_DBCR0, mfspr(SPRN_DBCR0) | DBCR0_RST_CHIP);
	while (1) ; /* Just in case the reset doesn't work */
}

define_machine(dm500) {
	.name = "Dreambox DM500",
	.probe = dm500_probe,
	.progress = udbg_progress,
	.init_IRQ = uic_init_tree,
	.get_irq = uic_get_irq,
	.restart = dm500_reset_system,
	.calibrate_decr = generic_calibrate_decr,
};
