/*
 *
 * Copyright (C) 2007 IC Nexus Co., LTD.
 *
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 *
 */

/*
 * 2007/07/16: initial
 *
 *
 * Author: Hardy Weng <hardy.weng@icnexus.com.tw>
 *
 */


#ifndef _NXC2600_CPU_FEATURE_OVERRIDES_H_
#define _NXC2600_CPU_FEATURE_OVERRIDES_H_

#define cpu_has_tlb		0
#define cpu_has_4kex		0
#define	cpu_has_watch		0
#define cpu_has_vce		0
#define cpu_has_3k_cache	0
#define cpu_has_4k_cache	1
#define cpu_has_fpu		0
#define cpu_has_32fpr		0
#define cpu_has_counter		0
#define cpu_has_mips16		0
#define cpu_has_divec		0
#define cpu_has_cache_cdex_p	0
#define cpu_has_cache_cdex_s	0
#define cpu_has_prefetch	0
#define cpu_has_mcheck		0
#define cpu_has_ejtag		0
#define cpu_has_inclusive_pcaches       0

#define cpu_has_llsc		0
#define cpu_has_vtag_icache	0
#define cpu_has_dc_aliases	1
#define cpu_has_ic_fills_f_dc	0

#define cpu_has_dsp		0
#define cpu_has_mipsmt		0
#define cpu_has_userlocal	0

#define cpu_has_nofpuex		0
#define cpu_has_64bits		0

#define cpu_has_mips32r1	0
#define cpu_has_mips32r2	0
#define cpu_has_mips64r1	0
#define cpu_has_mips64r2	0

#define cpu_dcache_line_size()  32
#define cpu_icache_line_size()  32
#define cpu_scache_line_size()  0
#if 0
#define K0_TO_K1()                             \
do {                                           \
       unsigned long __k0_addr;                \
                                               \
       __asm__ __volatile__(                   \
       "la %0, 1f\n\t"                         \
       "or     %0, %0, %1\n\t"                 \
       "jr     %0\n\t"                         \
       "nop\n\t"                               \
       "1: nop\n"                              \
       : "=&r"(__k0_addr)                      \
       : "r" (0x20000000) );                   \
} while(0)

#define K1_TO_K0()                             \
do {                                           \
       unsigned long __k0_addr;                \
       __asm__ __volatile__(                   \
       "nop;nop;nop;nop;nop;nop;nop\n\t"       \
       "la %0, 1f\n\t"                         \
       "jr     %0\n\t"                         \
       "nop\n\t"                               \
       "1:     nop\n"                          \
       : "=&r" (__k0_addr));                   \
} while (0)

#define INVALIDATE_BTB()                       \
do {                                           \
       unsigned long tmp;                      \
       __asm__ __volatile__(                   \
       ".set mips32\n\t"                       \
       "mfc0 %0, $16, 7\n\t"                   \
       "nop\n\t"                               \
       "ori %0, 2\n\t"                         \
       "mtc0 %0, $16, 7\n\t"                   \
       "nop\n\t"                               \
       : "=&r" (tmp));                         \
} while (0)

#define SYNC_WB() __asm__ __volatile__ ("sync")
#endif

#endif
