#ifndef _ASM_PPC_STBX25XX_DEBUG_
#define _ASM_PPC_STBX25XX_DEBUG_

#define FP_A			0x80
#define FP_B			0x40
#define FP_C			0x20
#define FP_D			0x10
#define FP_E			0x08
#define FP_F			0x04
#define FP_G			0x02
#define FP_DP			0x01
#define FP_0			(FP_A | FP_B | FP_C | FP_D | FP_E | FP_F)
#define FP_1			(FP_B | FP_C)
#define FP_2			(FP_A | FP_B | FP_G | FP_E | FP_D)
#define FP_3			(FP_A | FP_B | FP_G | FP_C | FP_D)
#define FP_4			(FP_F | FP_G | FP_B | FP_C)
#define FP_5			(FP_A | FP_F | FP_G | FP_C | FP_D)
#define FP_6			(FP_A | FP_F | FP_G | FP_C | FP_D | FP_E)
#define FP_7			(FP_A | FP_B | FP_C)
#define FP_8			(FP_A | FP_B | FP_C | FP_D | FP_E | FP_F | FP_G)
#define FP_9			(FP_A | FP_B | FP_C | FP_D | FP_F | FP_G)
#define FP_DCR(n)		(0x314 + (n))

#ifndef __ASSEMBLY__
#include <asm/dcr.h>
#define FP_PUT(n,x)		mtdcr(FP_DCR((n)), ((x) << 16));
#else
#define FP_PUT(n,x)		lis	r9, x; \
				mtdcr	FP_DCR(n), r9; \
				eieio
#endif

#define FP_PRINT(a,b,c,d)	FP_PUT(0, (FP_ ## a) << 8); \
				FP_PUT(1, (FP_ ## b) << 8); \
				FP_PUT(2, (FP_ ## c) << 8); \
				FP_PUT(3, (FP_ ## d) << 8)
#endif

