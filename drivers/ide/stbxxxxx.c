/*
 *    Copyright 2001 MontaVista Software Inc.
 *      Completed implementation.
 *      Author: MontaVista Software, Inc.  <source@mvista.com>
 *		Hai-Pao Fan <hpfan@mvista.com>
 *
 *    Module name: stbxxxxx.c
 *
 *    Description:
 *	This option provides support for IDE on IBM STB034xx Redwood-4,
 *	IBM STBx25xx Redwood-6, TiGATE tgs100
 *	system.
 */

#include <linux/version.h>
#include <linux/config.h>
#include <linux/types.h>
#include <linux/hdreg.h>
#include <linux/delay.h>
#include <linux/ide.h>
#include <asm/ocp.h>
#include "../ide-timing.h"

#include <asm/io.h>
#include <asm/ppc4xx_dma.h>

#ifdef IDE0_IRQ
#undef IDE0_IRQ
#endif

#define IDE_VER			"2.0"

//#define WAIT_WHILE_DMA
#define SUPPORT_LBA48

#define DCRXBCR_WRITE		0x20000000
#define DCRXBCR_MDMA2		0xC0000000

#define IDE0_IRQ 	25		/* interrupt level(int0) */

/* use DMA channel 2 for IDE DMA operations */

#define IDE_DMA_INT	6		/* IDE dma channel 2 interrupt */

/* DMACR2 */
#define IDE_DMACR_CE	0x80000000	/* 00 channel enable */
#define IDE_DMACR_CIE	0x40000000	/* 01 channel interrupt enable */
#define IDE_DMACR_TD	0x20000000	/* 02 transfer direction */
					/* 0 = write 1 = read */
#define IDE_DMACR_PL	0x10000000	/* 03 peripheral location */
#define IDE_DMACR_PW	0x0C000000	/* 04-05 peripheral width */
#define IDE_DMACR_DAI	0x02000000	/* 06 destination address increment */
#define IDE_DMACR_SAI	0x01000000	/* 07 source address increment */
#define IDE_DMACR_CP	0x00800000	/* 08 high order channel priority bit*/
#define IDE_DMACR_TM	0x00600000	/* 09-10 transfer mode */
#define IDE_DMACR_PSC	0x00180000	/* 11-12 peripheral setup cycles */
#define IDE_DMACR_PWC	0x0007E000	/* 13-18 peripheral wait cycles */
#define IDE_DMACR_PHC	0x00001C00	/* 19-21 peripheral hold cycles */
#define IDE_DMACR_ETD	0x00000200	/* 22 EOT/TC */
#define IDE_DMACR_TCE	0x00000100	/* 23 TC Enable */
#define IDE_DMACR_CH	0x00000080	/* 24 chaining enable */
#define IDE_DMACR_ECE	0x00000020	/* 26 EOT chain mode enable */
#define IDE_DMACR_TCD	0x00000010	/* 27 TC chain mode enable */
#define IDE_DMACR_DEC	0x00000004	/* 29 destination address decrement */
#define IDE_DMACR_CP1	0x00000001	/* 31 low order channel priority bit */

#define IDE_DMASR_TC	0x20000000
#define IDE_DMASR_EOT	0x02000000
#define IDE_DMASR_ERR	0x00200000
#define IDE_DMASR_CB	0x00000100
#define IDE_DMASR_CT	0x00000020
#define DCRXBCR_ACTIVATE	0x10000000

static unsigned long dmacr_def = 0x0000AB02;	/* pwc=101 phc=10, res:30=1 */
static void ocp_ide_tune_drive(ide_drive_t *, byte);
static int ocp_ide_dma_on(ide_drive_t * drive);
static int ocp_ide_dma_check(ide_drive_t *drive);
static int ocp_ide_dma_end(ide_drive_t *drive);

#define WMODE	0		/* default to DMA line mode */

#ifdef WAIT_WHILE_DMA
static volatile unsigned long dmastat;
#endif

/* Function Prototypes */
static byte ocp_ide_dma_2_pio(byte);
static int ocp_ide_check_dma(ide_drive_t * drive);
#define MK_TIMING(AS, DIOP, DIOY, DH) \
	((FIT((AS),    0, 15) << 27) | \
	 (FIT((DIOP),  0, 63) << 20) | \
	 (FIT((DIOY),  0, 63) << 13) | \
	 (FIT((DH),    0,  7) << 9))

#define UTIMING_SETHLD	(EZ(20 /*tACK*/, SYS_CLOCK_NS) - 1 /*fixed cycles*/)
#define UTIMING_ENV	(EZ(20 /*tENV*/, SYS_CLOCK_NS) - 1 /*fixed cycles*/)
#define UTIMING_SS	(EZ(50 /*tSS */, SYS_CLOCK_NS) - 3 /*fixed cycles*/)
#define MK_UTIMING(CYC, RP) \
	((FIT(UTIMING_SETHLD, 0, 15) << 27) | \
	 (FIT(UTIMING_ENV,    0, 15) << 22) | \
	 (FIT((CYC),          0, 15) << 17) | \
	 (FIT((RP),           0, 63) << 10) | \
	 (FIT(UTIMING_SS,     0, 15) << 5)  | \
	 1 /* Turn on Ultra DMA */)

/* Define the period of the STB clock used to generate the
 * IDE bus timing.  The clock is actually 63 MHz, but it
 * get rounded in a favorable direction.
 */
#define IDE_SYS_FREQ	50	/* MHz */
#define SYS_CLOCK_NS	(1000 / IDE_SYS_FREQ)


/* Function Prototypes */
static void ocp_ide_tune_drive(ide_drive_t *, byte);

static byte ocp_ide_dma_2_pio(byte xfer_rate)
{
	switch(xfer_rate) {
	case XFER_UDMA_5:
	case XFER_UDMA_4:
	case XFER_UDMA_3:
	case XFER_UDMA_2:
	case XFER_UDMA_1:
	case XFER_UDMA_0:
	case XFER_MW_DMA_2:
	case XFER_PIO_4:
		return 4;
	case XFER_MW_DMA_1:
	case XFER_PIO_3:
		return 3;
	case XFER_SW_DMA_2:
	case XFER_PIO_2:
		return 2;
	case XFER_MW_DMA_0:
	case XFER_SW_DMA_1:
	case XFER_SW_DMA_0:
	case XFER_PIO_1:
	case XFER_PIO_0:
	case XFER_PIO_SLOW:
	default:
		return 0;
	}
}

static ide_startstop_t redwood_ide_intr (ide_drive_t *drive)
{
	int dma_stat;
	unsigned char stat;
	ide_hwgroup_t *hwgroup = HWGROUP(drive);
	struct request *rq = hwgroup->rq;
	sector_t rest;

	//printk( "ide interrupt\n" );

	dma_stat = ocp_ide_dma_end(drive);
	stat = HWIF(drive)->INB(IDE_STATUS_REG);

	if (OK_STAT(stat,DRIVE_READY,drive->bad_wstat|DRQ_STAT))
	{
		if (!dma_stat)
		{
			rest = rq->nr_sectors - rq->current_nr_sectors;

			//DRIVER(drive)->end_request(drive, 1, rq->current_nr_sectors);
			ide_end_request(drive, 1, rq->current_nr_sectors);
			//printk( "Ok.\n" );

			if( rest )
				//return __ide_do_rw_disk( drive, rq, rq->sector );
				return ide_do_rw_disk( drive, rq, rq->sector );
			else
				return ide_stopped;
		}

		printk( KERN_WARNING "%s: redwood_ide_intr: bad DMA status (dma_stat %08x)\n",
				drive->name, dma_stat );
	}

	printk( "Error.\n" );
	//return DRIVER(drive)->error(drive, __FUNCTION__, stat);
	return idedisk_error(drive, __FUNCTION__, stat);
}


static int redwood_dma_timer_expiry(ide_drive_t *drive)
{
	ide_hwif_t *hwif	= HWIF(drive);
	u8 dma_stat		= hwif->INB(hwif->dma_status);

	printk(KERN_WARNING "%s: dma_timer_expiry: dma status == 0x%02x\n",
		drive->name, dma_stat);

	if ((dma_stat & 0x18) == 0x18)	/* BUSY Stupid Early Timer !! */
		return WAIT_CMD;

	HWGROUP(drive)->expiry = NULL;	/* one free ride for now */

	/* 1 dmaing, 2 error, 4 intr */
	
	if (dma_stat & 2) {	/* ERROR */
		(void) hwif->ide_dma_end(drive);
		//return DRIVER(drive)->error(drive,
		///	"dma_timer_expiry", hwif->INB(IDE_STATUS_REG));
		return idedisk_error(drive,
			"dma_timer_expiry", hwif->INB(IDE_STATUS_REG));
	}
	if (dma_stat & 1)	/* DMAing */
		return WAIT_CMD;

	if (dma_stat & 4)	/* Got an Interrupt */
		HWGROUP(drive)->handler(drive);

	return 0;
}
static int ocp_ide_dma_set(ide_drive_t *drive, int write)
{
	struct request *rq = HWGROUP(drive)->rq;
	unsigned long length;
	sector_t sectors;
	if (drive->media != ide_disk)
		return 0;
	if (mfdcr(DCRN_DMACR2) & IDE_DMACR_CE)	/* DMA is busy? */
	{
		printk("ocp_ide_dma_set :dma is working now.\n");
		return -1;
	}

	sectors = rq->current_nr_sectors;

	if (write)
	{
		dma_cache_wback_inv((unsigned long) rq->buffer,
				sectors * SECTOR_SIZE);

		mtdcr(DCRN_DMASA2, virt_to_bus(rq->buffer));
#if WMODE
		mtdcr(DCRN_DMADA2,0);
#else
		mtdcr(DCRN_DMADA2,0xfce00000);
#endif
	}
	else
	{
		dma_cache_inv((unsigned long) rq->buffer,
				sectors * SECTOR_SIZE);

#if WMODE
		mtdcr(DCRN_DMASA2,0);
#else
		mtdcr(DCRN_DMASA2,0xfce00000);
#endif
		mtdcr(DCRN_DMADA2, virt_to_bus(rq->buffer));
	}

#ifdef SUPPORT_LBA48
	if( drive->addressing )
	{
		ata_nsector_t nsectors;

		nsectors.all = (u16)sectors;
		HWIF(drive)->OUTB(nsectors.b.high, IDE_NSECTOR_REG);
		HWIF(drive)->OUTB(nsectors.b.low , IDE_NSECTOR_REG);
	}
	else
#endif
	{
		HWIF(drive)->OUTB(sectors, IDE_NSECTOR_REG);
	}
	length = sectors * SECTOR_SIZE;

	/* set_dma_count doesn't do M2M line xfer sizes right. */
#if WMODE
	mtdcr(DCRN_DMACT2, length >> 2);
#else
	mtdcr(DCRN_DMACT2, length >> 4);
#endif

	if (write)
	{
#if WMODE
		mtdcr(DCRN_DMACR2, 0x46000000 | dmacr_def);
#else
		mtdcr(DCRN_DCRXBCR, 0xB0000000);
		mtdcr(DCRN_DMACR2, 0x4D600000 | dmacr_def);
#endif
	}
	else
	{
#if WMODE
		mtdcr(DCRN_DMACR2, 0x66000000 | dmacr_def);
#else
		mtdcr(DCRN_DCRXBCR,0x90000000);
		mtdcr(DCRN_DMACR2, 0x6E600000 | dmacr_def);
#endif
	}

	drive->waiting_for_dma = 1;

	return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10)
static int ocp_ide_dma_begin(ide_drive_t *drive)
{
	/* enable DMA */
        mtdcr(DCRN_DMACR2,mfdcr(DCRN_DMACR2) | IDE_DMACR_CE);
	return 0;
}

static int ocp_ide_dma_read(ide_drive_t *drive)
{
	task_ioreg_t command;

	if( ocp_ide_dma_set(drive,0) )
		return 0;

#ifdef SUPPORT_LBA48
	command = (drive->addressing == 1) ? WIN_READDMA_EXT : WIN_READDMA;
#else
	command = WIN_READDMA;
#endif

	/* issue cmd to drive */
	ide_execute_command(drive, command, &redwood_ide_intr, WAIT_CMD, NULL);

	return ocp_ide_dma_begin( drive );
}

static int ocp_ide_dma_write(ide_drive_t *drive)
{
	task_ioreg_t command;

	if( ocp_ide_dma_set(drive,1) )
		return 0;

#ifdef SUPPORT_LBA48
	command = (drive->addressing == 1) ? WIN_WRITEDMA_EXT : WIN_WRITEDMA;
#else
	command = WIN_WRITEDMA;
#endif

	/* issue cmd to drive */
	ide_execute_command(drive, command, &redwood_ide_intr, WAIT_CMD, NULL);

	return ocp_ide_dma_begin( drive );
}

#else
static int ocp_dma_setup( ide_drive_t *drive )
{
	ide_hwgroup_t *hwgroup = HWGROUP(drive);
	struct request *rq = hwgroup->rq;

	return ocp_ide_dma_set(drive, rq_data_dir(rq));
}

static void ocp_dma_exec_cmd( ide_drive_t *drive, u8 cmd )
{
	/* issue cmd to drive */
	ide_execute_command(drive, cmd, &redwood_ide_intr, WAIT_CMD, NULL);
}

static void ocp_dma_start( ide_drive_t *drive )
{
	/* enable DMA */
	mtdcr(DCRN_DMACR2,mfdcr(DCRN_DMACR2) | IDE_DMACR_CE);
}

#endif

static int ocp_ide_dma_end(ide_drive_t *drive)
{
	unsigned long cr;
	int ret = 0;

	/* check if dma is working */
	cr = mfdcr(DCRN_DMACR2);
	if(cr & IDE_DMACR_CE)
	{
		printk(KERN_WARNING "dma is working.(status %08lx)\n", cr );
		ret = (int)cr;
	}

	/* disable DMA */
	mtdcr(DCRN_DMACR2, cr & ~0x80000000);
	mtdcr( DCRN_DCRXBCR, 0x80000000 );

	drive->waiting_for_dma = 0;

	return ret;
}

static int ocp_ide_dma_check(ide_drive_t *drive)
{
	return ocp_ide_dma_on(drive);
}

static int ocp_ide_dma_on(ide_drive_t * drive)
{
	return ocp_ide_check_dma(drive);
}

static int ocp_ide_tune_chipset(ide_drive_t * drive, byte speed)
{
	int err = 0;

	ocp_ide_tune_drive(drive, ocp_ide_dma_2_pio(speed));

	if (!drive->init_speed)
		drive->init_speed = speed;
	err = ide_config_drive_speed(drive, speed);
	drive->current_speed = speed;
	return err;
}

static int ocp_ide_check_dma(ide_drive_t * drive)
{
	unsigned long flags;
	drive->using_dma = 0;
	if (drive->media != ide_disk)
                return 0;
	mtdcr(DCRN_DMACR2, 0);
	mtdcr(DCRN_DMASR, IDE_DMASR_TC | IDE_DMASR_EOT | IDE_DMASR_ERR
                        | IDE_DMASR_CT);


#if WMODE
	mtdcr(DCRN_DCRXBCR, 0);
	mtdcr(DCRN_CICCR, mfdcr(DCRN_CICCR) | 0x00000400);
#else
	/* Configure CIC reg for line mode dma */
	mtdcr(DCRN_CICCR, mfdcr(DCRN_CICCR) & ~0x00000400);
#endif

	drive->using_dma = 1;

	return 0;
}

static int ocp_ide_dma_off_quietly(ide_drive_t *drive)
{
	drive->using_dma = 0;
	return ocp_ide_dma_end(drive);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10)
static int report_drive_dmaing(ide_drive_t * drive)
{
	struct hd_driveid *id = drive->id;

	if ((id->field_valid & 4) && (eighty_ninty_three(drive)) &&
	    (id->dma_ultra & (id->dma_ultra >> 11) & 7)) {
		if ((id->dma_ultra >> 13) & 1) {
			printk(", UDMA(100)");	/* UDMA BIOS-enabled! */
		} else if ((id->dma_ultra >> 12) & 1) {
			printk(", UDMA(66)");	/* UDMA BIOS-enabled! */
		} else {
			printk(", UDMA(44)");	/* UDMA BIOS-enabled! */
		}
	} else if ((id->field_valid & 4) &&
		   (id->dma_ultra & (id->dma_ultra >> 8) & 7)) {
		if ((id->dma_ultra >> 10) & 1) {
			printk(", UDMA(33)");	/* UDMA BIOS-enabled! */
		} else if ((id->dma_ultra >> 9) & 1) {
			printk(", UDMA(25)");	/* UDMA BIOS-enabled! */
		} else {
			printk(", UDMA(16)");	/* UDMA BIOS-enabled! */
		}
	} else if (id->field_valid & 4) {
		printk(", (U)DMA");	/* Can be BIOS-enabled! */
	} else {
		printk(", DMA");
	}
	return 1;
}

static int ocp_ide_dma_verbose(ide_drive_t *drive)
{
	return report_drive_dmaing(drive);
}

#endif
static int ocp_ide_dma_lostirq(ide_drive_t *drive)
{
	printk(KERN_INFO "%s: chipset supported \n", __FUNCTION__);
	return 1;

}

static int ocp_ide_dma_timeout(ide_drive_t *drive)
{
	printk(KERN_INFO "%s: chipset supported \n", __FUNCTION__);
	return redwood_dma_timer_expiry( drive );
}

static void ocp_ide_tune_drive(ide_drive_t * drive, byte pio)
{
	pio = ide_get_best_pio_mode(drive, pio, 5, NULL);
	printk( "%s: use pio mode %d\n", drive->name, pio );
}

static int ocp_ide_dma_test_irq(ide_drive_t *drive)
{
        unsigned long cr;

        cr = mfdcr( DCRN_DMACR2 );
        if( cr & IDE_DMACR_CE )
                return 0;
        
	return 1;
}

int nonpci_ide_default_irq(unsigned long base)
{
	return IDE0_IRQ;
}

void nonpci_ide_init_hwif_ports(hw_regs_t *hw, unsigned long data_port, unsigned long ctrl_port, int *irq)
{
	unsigned long reg = data_port;
	ide_hwif_t *hwif;
	unsigned long ioaddr;
	int i, index;

	printk("IBM Redwood 4/6 IDE driver version %s\n", IDE_VER);
 	memset(hw, 0, sizeof(*hw));

	mtdcr(DCRN_DCRXICR, 0x40000000);	/* set dcrx internal arbiter */

	/* reconstruct phys addrs from EBIU config regs for CS2# */
	reg = ((mfdcr(DCRN_BRCR2) & 0xff000000) >> 4) | 0xf0000000;
	ioaddr = (unsigned long)ioremap(reg, 0x10);
	for (i = IDE_DATA_OFFSET; i <= IDE_STATUS_OFFSET; i++)
	{
		hw->io_ports[i] = ioaddr;
		ioaddr += 2;
	}

	/* reconstruct phys addrs from EBIU config regs for CS3# */
	reg = (((mfdcr(DCRN_BRCR3) & 0xff000000) >> 4) | 0xf0000000) + 0x0c;
	ioaddr = (unsigned long)ioremap(reg, 0x02);
	hw->io_ports[IDE_CONTROL_OFFSET] = ioaddr;

	hw->irq = IDE0_IRQ;

	/* init CIC control reg to enable IDE interface PIO mode */
	mtdcr(DCRN_CICCR, (mfdcr(DCRN_CICCR) & 0xffff7bff) | 0x0003);

	/* use DMA channel 2 for IDE DMA operations */
	hw->dma = 2;

	mtdcr(DCRN_DMACR2, 0x4d600000 | dmacr_def);
	mtdcr(DCRN_DMASR, 0xffffffff); /* clear status register */

	/* init CIC select2 reg to connect external DMA port 3 to internal
	 * DMA channel 2
	 */
	mtdcr(DCRN_DMAS2, (mfdcr(DCRN_DMAS2) & 0xfffffff0) | 0x00000002);

	index = 0;
	hwif = &ide_hwifs[0];
	hwif->tuneproc = &ocp_ide_tune_drive;
	hwif->drives[0].autotune = 1;
	hwif->autodma = 1;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10)
	hwif->ide_dma_read = &ocp_ide_dma_read;
	hwif->ide_dma_write = &ocp_ide_dma_write;
	hwif->ide_dma_begin = &ocp_ide_dma_begin;
#else
	hwif->dma_setup = &ocp_dma_setup;
	hwif->dma_exec_cmd = &ocp_dma_exec_cmd;
	hwif->dma_start = &ocp_dma_start;
#endif
	hwif->ide_dma_end = &ocp_ide_dma_end;
	hwif->ide_dma_check = &ocp_ide_dma_check;
	hwif->ide_dma_on =&ocp_ide_dma_on;
	hwif->ide_dma_off_quietly = &ocp_ide_dma_off_quietly;
	hwif->ide_dma_host_off =&ocp_ide_dma_off_quietly; 
	hwif->ide_dma_test_irq =&ocp_ide_dma_test_irq;
	hwif->ide_dma_host_on = &ocp_ide_dma_on;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10)
	hwif->ide_dma_verbose = &ocp_ide_dma_verbose;
#endif
	hwif->ide_dma_lostirq = &ocp_ide_dma_lostirq;
	hwif->ide_dma_timeout = &ocp_ide_dma_timeout;
	hwif->speedproc = &ocp_ide_tune_chipset;
	hwif->noprobe = 0;
	/* dma count register is 16 bits. */
#if WMODE
	hwif->rqsize = 64*1024/SECTOR_SIZE*4;
#else
	hwif->rqsize = 64*1024/SECTOR_SIZE*16;
#endif
#ifndef SUPPORT_LBA48
	hwif->no_lba48 = 1;
	hwif->no_lba48_dma = 1;
#endif

	memcpy(hwif->io_ports, hw->io_ports, sizeof(hw->io_ports));
	hwif->irq = IDE0_IRQ;

	probe_hwif_init(hwif);
}
