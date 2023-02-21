/*
 * Copyright (C) 2003,2004 Philippe Gerum <rpm@xenomai.org>.
 *
 * ARM port
 *   Copyright (C) 2005 Stelian Pop
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

#ifndef _XENO_ASM_ARM_ATOMIC_H
#define _XENO_ASM_ARM_ATOMIC_H

#include <asm/xenomai/features.h>

typedef struct { unsigned long counter; } xnarch_atomic_t;
typedef xnarch_atomic_t atomic_counter_t;
typedef unsigned long atomic_flags_t;

#define xnarch_atomic_get(v)	(*(volatile unsigned long *)(&(v)->counter))
static __inline__ void
xnarch_atomic_set(xnarch_atomic_t *ptr, unsigned long val)
{
	ptr->counter = val;
}

#ifdef CONFIG_XENO_ATOMIC_BUILTINS
#define xnarch_memory_barrier()	__sync_synchronize()
#define xnarch_read_memory_barrier() xnarch_memory_barrier()
#define xnarch_write_memory_barrier() xnarch_memory_barrier()
#define cpu_relax() xnarch_memory_barrier()
#define xnarch_atomic_cmpxchg(v, o, n)                  \
        __sync_val_compare_and_swap(&(v)->counter,      \
                                    (unsigned long)(o), \
                                    (unsigned long)(n))
#else /* !CONFIG_XENO_ATOMIC_BUILTINS */
#include <asm/xenomai/atomic_asm.h>
#endif /* !CONFIG_XENO_ATOMIC_BUILTINS */

#endif /* !_XENO_ASM_ARM_ATOMIC_H */
