/*
 * Copyright (C) 2007 Philippe Gerum <rpm@xenomai.org>.
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

#ifndef _XENO_ASM_X86_ATOMIC_ASM_H
#define _XENO_ASM_X86_ATOMIC_ASM_H

#ifdef CONFIG_SMP
#define LOCK_PREFIX "lock ; "
#else
#define LOCK_PREFIX ""
#endif

static inline void cpu_relax(void)
{
	__asm__ __volatile__("rep; nop" ::: "memory");
}

#ifdef __i386__

struct __xeno_xchg_dummy { unsigned long a[100]; };
#define __xeno_xg(x) ((struct __xeno_xchg_dummy *)(x))

static inline unsigned long xnarch_atomic_xchg (volatile void *ptr,
						unsigned long x)
{
	__asm__ __volatile__("xchgl %0,%1"
			     :"=r" (x)
			     :"m" (*__xeno_xg(ptr)), "0" (x)
			     :"memory");
	return x;
}

static inline unsigned long
xnarch_atomic_cmpxchg(xnarch_atomic_t *v, unsigned long old, unsigned long newval)
{
	volatile void *ptr = &v->counter;
	unsigned long prev;

	__asm__ __volatile__(LOCK_PREFIX "cmpxchgl %1,%2"
			     : "=a"(prev)
			     : "r"(newval), "m"(*__xeno_xg(ptr)), "0"(old)
			     : "memory");
	return prev;
}

#define xnarch_memory_barrier()		__asm__ __volatile__("": : :"memory")
#define xnarch_read_memory_barrier() \
	__asm__ __volatile__ (LOCK_PREFIX "addl $0,0(%%esp)": : :"memory")
#define xnarch_write_memory_barrier() \
	__asm__ __volatile__ (LOCK_PREFIX "addl $0,0(%%esp)": : :"memory")

#else /* x86_64 */

#define __xeno_xg(x) ((volatile long *)(x))

static inline unsigned long xnarch_atomic_xchg (volatile void *ptr,
						unsigned long x)
{
	__asm__ __volatile__("xchgq %0,%1"
			     :"=r" (x)
			     :"m" (*__xeno_xg(ptr)), "0" (x)
			     :"memory");
	return x;
}

static inline unsigned long
xnarch_atomic_cmpxchg(xnarch_atomic_t *v, unsigned long old, unsigned long newval)
{
	volatile void *ptr = &v->counter;
	unsigned long prev;

	__asm__ __volatile__(LOCK_PREFIX "cmpxchgq %1,%2"
			     : "=a"(prev)
			     : "r"(newval), "m"(*__xeno_xg(ptr)), "0"(old)
			     : "memory");
	return prev;
}

#define xnarch_memory_barrier()		asm volatile("mfence":::"memory")
#define xnarch_read_memory_barrier()	asm volatile("lfence":::"memory")
#define xnarch_write_memory_barrier()	asm volatile("sfence":::"memory")

#endif /* x86_64 */

#endif /* _XENO_ASM_X86_ATOMIC_ASM_H */
