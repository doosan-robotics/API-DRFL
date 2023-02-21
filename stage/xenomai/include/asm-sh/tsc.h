/*
 * Copyright (C) 2011 Philippe Gerum <rpm@xenomai.org>.
 *
 * Xenomai is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation; either version 2 of the License,
 * or (at your option) any later version.
 *
 * Xenomai is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Xenomai; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
 * 02111-1307, USA.
 */

#ifndef _XENO_ASM_SH_TSC_H
#define _XENO_ASM_SH_TSC_H

#ifndef __KERNEL__

#include <endian.h>

struct xnarch_tsc_area {
	struct {
#if __BYTE_ORDER == __BIG_ENDIAN
		unsigned long high;
		unsigned long low;
#else /* __LITTLE_ENDIAN */
		unsigned long low;
		unsigned long high;
#endif /* __LITTLE_ENDIAN */
	} tsc;
	unsigned long counter_pa;
};

extern volatile struct xnarch_tsc_area *xeno_sh_tsc;

extern volatile unsigned long *xeno_sh_tcnt;

static inline unsigned long long __xn_rdtsc(void)
{
	unsigned long long tsc;
	unsigned long low;

	tsc = xeno_sh_tsc->tsc.high;
	low = *xeno_sh_tcnt ^ 0xffffffffUL;
	if (low < xeno_sh_tsc->tsc.low)
		tsc++;
	tsc = (tsc << 32)|low;

	return tsc;
}

#endif /* __KERNEL__ */

#endif /* _XENO_ASM_SH_TSC_H */
