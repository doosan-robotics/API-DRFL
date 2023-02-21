/*
 * Copyright (C) 2005 Philippe Gerum <rpm@xenomai.org>.
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

#ifndef _XENO_ASM_BLACKFIN_TSC_H
#define _XENO_ASM_BLACKFIN_TSC_H

#ifndef __KERNEL__

static inline unsigned long long __xn_rdtsc (void)
{
    union {
	struct {
	    unsigned long l;
	    unsigned long h;
	} s;
	unsigned long long t;
    } u;
    unsigned long cy2;

    __asm__ __volatile__ (	"1: %0 = CYCLES2\n"
				"%1 = CYCLES\n"
				"%2 = CYCLES2\n"
				"CC = %2 == %0\n"
				"if !cc jump 1b\n"
				:"=d" (u.s.h),
				"=d" (u.s.l),
				"=d" (cy2)
				: /*no input*/ : "cc");
    return u.t;
}

#endif

#endif /* _XENO_ASM_BLACKFIN_TSC_H */
