/*
 * arch/powerpc/include/asm/dm500.h
 *
 * Macros, definitions, and data structures specific to the Dreambox
 * Multimedia DM500 board.
 *
 * Author: Armin Kuster <akuster@mvista.com>
 *
 * 2002 (c) MontaVista, Software, Inc.  This file is licensed under
 * the terms of the GNU General Public License version 2.  This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#ifdef __KERNEL__
#ifndef __ASM_DM500_H__
#define __ASM_DM500_H__

//#include <asm/ibmstbx25.h>

#ifdef MAX_HWIFS
#undef MAX_HWIFS
#endif
#define MAX_HWIFS		1

#define PPC4xx_MACHINE_NAME	"dm500"

/* GPIO */
#define PPC4xx_GPIO_BASE	224
#define DM500_GPIO_LED_GREEN	224
#define DM500_GPIO_LED_RED	225
#define DM500_GPIO_FE_LNB	227
#define DM500_GPIO_FE_13V	231

extern int _board_is_dm500;

#ifdef CONFIG_DM500
static inline int board_is_dm500(void)
{
	return _board_is_dm500;
}
#else
#define board_is_dm500()	(0)
#endif

#endif				/* __ASM_DM500_H__ */
#endif				/* __KERNEL__ */
