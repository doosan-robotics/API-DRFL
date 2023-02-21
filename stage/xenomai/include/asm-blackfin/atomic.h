/*
 * Copyright (C) 2005, 2012 Philippe Gerum <rpm@xenomai.org>.
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

#ifndef _XENO_ASM_BLACKFIN_ATOMIC_H
#define _XENO_ASM_BLACKFIN_ATOMIC_H

#include <asm/xenomai/features.h>

#ifndef __KERNEL__

typedef struct { int counter; } xnarch_atomic_t;
typedef xnarch_atomic_t atomic_counter_t;
typedef unsigned long atomic_flags_t;

#define xnarch_atomic_get(v)	((v)->counter)
#define xnarch_atomic_set(v, i)	(((v)->counter) = (i))

#endif /* !__KERNEL__ */

#ifdef CONFIG_XENO_ATOMIC_BUILTINS
#define xnarch_memory_barrier()		__sync_synchronize()
#define xnarch_read_memory_barrier()	xnarch_memory_barrier()
#define xnarch_write_memory_barrier()	xnarch_memory_barrier()
#define cpu_relax()			xnarch_memory_barrier()

static inline unsigned long
xnarch_atomic_cmpxchg(xnarch_atomic_t *p,
		      unsigned long o, unsigned long n)
{
	return __sync_val_compare_and_swap(&p->counter, o, n);
}
#else
#include <asm/xenomai/atomic_asm.h>
#endif

#endif /* !_XENO_ASM_BLACKFIN_ATOMIC_H */
