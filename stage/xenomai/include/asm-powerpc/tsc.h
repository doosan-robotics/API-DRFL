/*
 * Copyright (C) 2001,2002,2003,2004 Philippe Gerum <rpm@xenomai.org>.
 *
 * 64-bit PowerPC adoption
 *   copyright (C) 2005 Taneli Vähäkangas and Heikki Lindholm
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

#ifndef _XENO_ASM_POWERPC_TSC_H
#define _XENO_ASM_POWERPC_TSC_H

#ifndef __KERNEL__

static inline unsigned long long __xn_rdtsc(void)
#if defined(__powerpc64__)
{
	unsigned long long t;

	__asm__ __volatile__("mftb %0\n":"=r"(t));
	return t;
}
#else				/* !__powerpc64__ */
{
	union {
		unsigned long long t;
		unsigned long v[2];
	} u;
	unsigned long __tbu;

	__asm__ __volatile__("1: mfspr %0,269\n"
			     "mfspr %1,268\n"
			     "mfspr %2,269\n"
			     "cmpw %2,%0\n"
			     "bne- 1b\n":"=r"(u.v[0]),
			     "=r"(u.v[1]), "=r"(__tbu));
	return u.t;
}
#endif /* __powerpc64__ */

#endif /* __KERNEL__ */

#endif /* _XENO_ASM_POWERPC_TSC_H */
