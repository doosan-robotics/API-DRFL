/*
 * Copyright (C) 2009 Philippe Gerum <rpm@xenomai.org>.
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

#ifndef _XENO_ASM_NIOS2_TSC_H
#define _XENO_ASM_NIOS2_TSC_H

#ifndef __KERNEL__

extern volatile void *xeno_nios2_hrclock;

static inline unsigned long long __xn_rdtsc(void)
{
	volatile unsigned short *hrclock;
	int64_t t0, t1;

	hrclock = xeno_nios2_hrclock;

#define hrclock_wrsnap(reg, val)		\
	(*(hrclock + (12 + ((reg) * 2)))) = (val)

#define hrclock_rdsnap(reg)			\
	(int64_t)(*(hrclock + (12 + ((reg) * 2)))) << (reg * 16)

#define hrclock_peeksnap()						\
	({								\
		int64_t __snap;						\
		__snap = hrclock_rdsnap(3) | hrclock_rdsnap(2) |	\
			hrclock_rdsnap(1) | hrclock_rdsnap(0);		\
		__snap;							\
	})

#define hrclock_getsnap()						\
	({								\
		hrclock_wrsnap(0, 0);					\
		hrclock_peeksnap();					\
	})

	/*
	 * We compete with both the kernel and userland applications
	 * which may request a snapshot as well, but we don't have any
	 * simple mutual exclusion mechanism at hand to avoid
	 * races. In order to keep the overhead of reading the hrclock
	 * from userland low, we make sure to read two consecutive
	 * coherent snapshots. In case both readings do not match, we
	 * have to request a fresh snapshot anew, since it means that
	 * we have been preempted in the middle of the operation.
	 */
	do {
		t0 = hrclock_getsnap(); /* Request snapshot and read it */
		__asm__ __volatile__("": : :"memory");
		t1 = hrclock_peeksnap(); /* Confirm first reading */
	} while (t0 != t1);

#undef hrclock_getsnap
#undef hrclock_rdsnap
#undef hrclock_wrsnap

	return ~t0;
}

#endif /* __KERNEL__ */

#endif /* _XENO_ASM_NIOS2_TSC_H */
