/*
 * arch/ppc/platforms/4xx/dm56xx.h
 *
 * Macros, definitions, and data structures specific to the IBM PowerPC
 * STBx25xx "Redwood6" evaluation board.
 *
 * Author: Armin Kuster <akuster@mvista.com>
 *
 * 2002 (c) MontaVista, Software, Inc.  This file is licensed under
 * the terms of the GNU General Public License version 2.  This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#ifdef __KERNEL__
#ifndef __ASM_TM9100_H__
#define __ASM_TM9100_H__

//#include <asm/ibmstbx25.h>

#ifndef __ASSEMBLY__
typedef struct bd_info {
/* Legacy data from the old bd_info struct */
	unsigned char	bi_s_version[4];	/* Version of this structure */
	unsigned char	bi_r_version[30];	/* Version of the IBM ROM */
	unsigned int	bi_memsize_legacy;	/* DRAM installed, in bytes */
	unsigned int	bi_dummy;		/* field shouldn't exist */
	unsigned char	bi_enetaddr[6];		/* Ethernet MAC address */
	unsigned int	bi_intfreq;		/* Processor speed, in Hz */
	unsigned int	bi_busfreq;		/* Bus speed, in Hz */
	unsigned int	bi_tbfreq;		/* Software timebase freq */
	unsigned int	bi_opbfreq;		/* OPB Bus speed, in Hz */
	int		bi_iic_fast[2];		/* Use fast i2c mode */
/* End of legacy data */
	unsigned long	bi_memstart;
	unsigned long	bi_memsize;
	unsigned long	bi_flashstart;	/* start of FLASH memory */
	unsigned long	bi_flashsize;	/* size  of FLASH memory */
	unsigned long	bi_flashoffset;	/* reserved area for startup monitor */
	unsigned long	bi_sramstart;	/* start of SRAM memory */
	unsigned long	bi_sramsize;	/* size  of SRAM memory */
	unsigned long	bi_baudrate;	/* Console Baudrate */
	unsigned int	bi_procfreq;	/* Processor speed, in Hz */
	unsigned long	bi_bootflags;	/* boot / reboot flag (for LynxOS) */
	unsigned int	bi_plb_busfreq;	/* PLB Bus speed, in Hz */
	unsigned long	bi_ip_addr;	/* IP Address */
} bd_t;
#endif				/* !__ASSEMBLY__ */

#define DM9000_MEM_ADDR		0xf2000000
#define DM9000_MEM_DATA		0xf2000004
#define DM9000_BUS_WIDTH	2
#define DM9000_IRQ		26

#ifdef MAX_HWIFS
#undef MAX_HWIFS
#endif
#define MAX_HWIFS		1

#define _IO_BASE	0
#define _ISA_MEM_BASE	0
#define PCI_DRAM_OFFSET	0

#define UARTCLK			(378000000 / 18)

#define PPC4xx_MACHINE_NAME	"tm9101"

/* GPIO */
#define PPC4xx_GPIO_BASE	224
#define TM9101_GPIO_FE_RST_N	228
#define TM9101_GPIO_FE_18V	229
#define TM9101_GPIO_FE_LNB	231
#define TM9101_GPIO_CIMAX_RST	252
#define TM9101_GPIO_UNKNOWN	253
#define TM9101_GPIO_ETH_RST_N	255

#endif				/* __ASM_TM9100_H__ */
#endif				/* __KERNEL__ */
