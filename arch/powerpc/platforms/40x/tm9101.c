/*
 * Architecture- / platform-specific boot-time initialization code for
 * IBM PowerPC 4xx based boards. Adapted from original
 * code by Gary Thomas, Cort Dougan <cort@fsmlabs.com>, and Dan Malek
 * <dan@net4x.com>.
 *
 * Copyright(c) 1999-2000 Grant Erickson <grant@lcse.umn.edu>
 *
 * Rewritten and ported to the merged powerpc tree:
 * Copyright 2007 IBM Corporation
 * Josh Boyer <jwboyer@linux.vnet.ibm.com>
 *
 * 2002 (c) MontaVista, Software, Inc.  This file is licensed under
 * the terms of the GNU General Public License version 2.  This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#include <linux/init.h>
#include <linux/of_platform.h>
#include <linux/rtc.h>

#include <asm/machdep.h>
#include <asm/prom.h>
#include <asm/udbg.h>
#include <asm/time.h>
#include <asm/uic.h>
#include <asm/ppc4xx.h>
#include <asm/tm9101.h>
#include <linux/dm9000.h>

static __initdata struct of_device_id tm9101_of_bus[] = {
	{ .compatible = "ibm,plb3", },
	{ .compatible = "ibm,opb", },
	{ .compatible = "ibm,ebc", },
	{},
};

static struct resource dm9000_resources[] = {
	[0] = {
		.start	= DM9000_MEM_ADDR,
		.end	= DM9000_MEM_ADDR + 3,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= DM9000_MEM_DATA,
		.end	= DM9000_MEM_DATA + 3,
		.flags	= IORESOURCE_MEM,
	},
	[2] = {
		.start	= DM9000_IRQ,
		.end	= DM9000_IRQ,
		.flags	= IORESOURCE_IRQ | IRQF_TRIGGER_LOW,
	},
};

/* TODO: Use DMA in outblk and inblk */
/* Manually byte-swap data on 16-bit reads */
static void tm9101_dm9000_outblk(void __iomem *reg, void *buf, int count)
{
	volatile u16 __iomem *port = reg;
	const u16 *tbuf = buf;
	
	count = (count+1) >> 1;

        if (unlikely(count <= 0))
                return;
        asm volatile("sync");
        do {
                *port = __cpu_to_le16(*tbuf++);
        } while (--count != 0);
        asm volatile("sync");
}

/* Manually byte-swap data on 16-bit writes */
static void tm9101_dm9000_inblk(void __iomem *reg, void *buf, int count)
{
	const volatile u16 __iomem *port = reg;
        u16 *tbuf = buf;
        u16 tmp;
	
	count = (count+1) >> 1;

        if (unlikely(count <= 0))
                return;
        asm volatile("sync");
        do {
                tmp = *port;
                eieio();
                *tbuf++ = __le16_to_cpu(tmp);
        } while (--count != 0);
        asm volatile("twi 0,%0,0; isync" : : "r" (tmp));
}

static struct dm9000_plat_data tm9101_dm9000_pdata = {
	.flags		= DM9000_PLATF_16BITONLY,
	.inblk		= tm9101_dm9000_inblk,
	.outblk		= tm9101_dm9000_outblk,
};

static struct platform_device dm9000_device = {
	.name		= "dm9000",
	.num_resources  = ARRAY_SIZE(dm9000_resources),
	.resource	= dm9000_resources,
	.dev		= {	
		.platform_data	= &tm9101_dm9000_pdata,
	},
};

static struct platform_device *tm9101_devs[] __initdata = {
	&dm9000_device,
};

static int __init tm9101_device_probe(void)
{
	struct device_node *np;
	int irq;

	of_platform_bus_probe(NULL, tm9101_of_bus, NULL);
	
	np = of_find_compatible_node(NULL, NULL, "davicom,dm9000");
	if (np == NULL) {
		printk(KERN_ERR __FILE__ ": Unable to find DM9000\n");
		goto exit;
	}

	irq = irq_of_parse_and_map(np, 0);
	of_node_put(np);
	if (irq  == NO_IRQ) {
		printk(KERN_ERR __FILE__ ": Unable to get DM9000 irq\n");
		goto exit;
	}
	
	dm9000_resources[2].start	= irq;
	dm9000_resources[2].end		= irq;
	
	platform_add_devices(tm9101_devs, ARRAY_SIZE(tm9101_devs));
	
exit:
	return 0;
}
machine_device_initcall(tm9101, tm9101_device_probe);

static int __init tm9101_probe(void)
{
	unsigned long root = of_get_flat_dt_root();

	if (!of_flat_dt_is_compatible(root, "technomate,tm9101"))
		return 0;

	return 1;
}

define_machine(tm9101) {
	.name			= "Technomate TM9101",
	.probe			= tm9101_probe,
	.progress		= udbg_progress,
	.init_IRQ		= uic_init_tree,
	.get_irq		= uic_get_irq,
	.restart		= ppc4xx_reset_system,
	.calibrate_decr		= generic_calibrate_decr,
};
