/*
 * arch/powerpc/boot/cuboot-ip250.c
 * 
 * Old U-boot compatibility for AB IPBox 250 
 *
 * Author: (C) Robert Burger <robert_burger@web.de>
 *
 * This file is licensed under the terms of the GNU General Public License 
 * version 2. This file is licensed "as is" without any warranty of any 
 * kind, whether express or implied.
 */

#include "ops.h"
#include "io.h"
#include "dcr.h"
#include "stdio.h"
#include "4xx.h"
#include "cuboot.h"

typedef struct bd_info {
/* Legacy data from the old bd_info struct */
  unsigned char bi_s_version[4];    /* Version of this structure */
  unsigned char bi_r_version[30];   /* Version of the IBM ROM */
  unsigned int  bi_memsize_legacy;  /* DRAM installed, in bytes */
  unsigned int  bi_dummy;           /* field shouldn't exist */
  unsigned char bi_enetaddr[6];     /* Ethernet MAC address */
  unsigned int  bi_intfreq;         /* Processor speed, in Hz */
  unsigned int  bi_busfreq;         /* Bus speed, in Hz */
  unsigned int  bi_tbfreq;          /* Software timebase freq */
  unsigned int  bi_opbfreq;         /* OPB Bus speed, in Hz */
  int bi_iic_fast[2];               /* Use fast i2c mode */
/* End of legacy data */
  unsigned long bi_memstart;
  unsigned long bi_memsize;
  unsigned long bi_flashstart;  /* start of FLASH memory */
  unsigned long bi_flashsize;   /* size  of FLASH memory */
  unsigned long bi_flashoffset; /* reserved area for startup monitor */
  unsigned long bi_sramstart;   /* start of SRAM memory */
  unsigned long bi_sramsize;    /* size  of SRAM memory */
  unsigned long bi_baudrate;    /* Console Baudrate */
  unsigned int  bi_procfreq;    /* Processor speed, in Hz */
  unsigned long bi_bootflags;   /* boot / reboot flag (for LynxOS) */
  unsigned int  bi_plb_busfreq; /* PLB Bus speed, in Hz */
  unsigned long bi_ip_addr;     /* IP Address */
} bd_t;
static bd_t bd;

static void ip250_fixups(void)
{
}
	
void platform_init(unsigned long r3, unsigned long r4, unsigned long r5,
		unsigned long r6, unsigned long r7)
{
	CUBOOT_INIT();
	platform_ops.fixups = ip250_fixups;
	platform_ops.exit = ibm40x_dbcr_reset;
	fdt_init(_dtb_start);
	serial_console_init();
}
