/*
 * Copyright (C) 2003,2004 Philippe Gerum <rpm@xenomai.org>.
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

#ifndef _XENO_ASM_POWERPC_ATOMIC_ASM_H
#define _XENO_ASM_POWERPC_ATOMIC_ASM_H

#ifndef _XENO_ASM_POWERPC_ATOMIC_H
#error "please don't include asm/atomic_asm.h directly"
#endif

#ifdef __KERNEL__

#include <linux/bitops.h>
#include <linux/version.h>
#include <asm/atomic.h>

#define xnarch_atomic_xchg(ptr,v)	xchg(ptr,v)
#define xnarch_memory_barrier()		smp_mb()
#define xnarch_read_memory_barrier()	rmb()
#define xnarch_write_memory_barrier()	wmb()

#ifdef CONFIG_PPC64
static __inline__ void atomic64_clear_mask(unsigned long mask,
					 unsigned long *ptr)
{
    __asm__ __volatile__ ("\n\
1:	ldarx	5,0,%0 \n\
	andc	5,5,%1\n"
"	stdcx.	5,0,%0 \n\
	bne-	1b"
	: /*no output*/
	: "r" (ptr), "r" (mask)
	: "r5", "cc", "memory");
}

static __inline__ void atomic64_set_mask(unsigned long mask,
				       unsigned long *ptr)
{
    __asm__ __volatile__ ("\n\
1:	ldarx	5,0,%0 \n\
	or	5,5,%1\n"
"	stdcx.	5,0,%0 \n\
	bne-	1b"
	: /*no output*/
	: "r" (ptr), "r" (mask)
	: "r5", "cc", "memory");
}

#define xnarch_atomic_set(pcounter,i)          atomic64_set(pcounter,i)
#define xnarch_atomic_get(pcounter)            atomic64_read(pcounter)
#define xnarch_atomic_inc(pcounter)            atomic64_inc(pcounter)
#define xnarch_atomic_dec(pcounter)            atomic64_dec(pcounter)
#define xnarch_atomic_inc_and_test(pcounter)   atomic64_inc_and_test(pcounter)
#define xnarch_atomic_dec_and_test(pcounter)   atomic64_dec_and_test(pcounter)
#define xnarch_atomic_set_mask(pflags,mask)    atomic64_set_mask(mask,pflags)
#define xnarch_atomic_clear_mask(pflags,mask)  atomic64_clear_mask(mask,pflags)
#define xnarch_atomic_cmpxchg(pcounter,old,new) \
      atomic64_cmpxchg((pcounter),(old),(new))

typedef atomic64_t atomic_counter_t;
typedef atomic64_t xnarch_atomic_t;

#else /* !CONFIG_PPC64 */
 /* These are defined in arch/{ppc,powerpc}/kernel/misc[_32].S on 32-bit PowerPC */
void atomic_set_mask(unsigned long mask, unsigned long *ptr);
void atomic_clear_mask(unsigned long mask, unsigned long *ptr);

#define xnarch_atomic_set(pcounter,i)          atomic_set(pcounter,i)
#define xnarch_atomic_get(pcounter)            atomic_read(pcounter)
#define xnarch_atomic_inc(pcounter)            atomic_inc(pcounter)
#define xnarch_atomic_dec(pcounter)            atomic_dec(pcounter)
#define xnarch_atomic_inc_and_test(pcounter)   atomic_inc_and_test(pcounter)
#define xnarch_atomic_dec_and_test(pcounter)   atomic_dec_and_test(pcounter)
#define xnarch_atomic_set_mask(pflags,mask)    atomic_set_mask(mask,pflags)
#define xnarch_atomic_clear_mask(pflags,mask)  atomic_clear_mask(mask,pflags)
#define xnarch_atomic_cmpxchg(pcounter,old,new) \
      atomic_cmpxchg((pcounter),(old),(new))

typedef atomic_t atomic_counter_t;
typedef atomic_t xnarch_atomic_t;

#endif /* !CONFIG_PPC64 */

typedef unsigned long atomic_flags_t;

#else /* !__KERNEL__ */

#ifndef __powerpc64__
/*
 * Always enable the work-around for 405 boards in user-space.
 */
#define PPC405_ERR77(ra,rb)	"dcbt " #ra "," #rb ";"
#else /* __powerpc64__ */
#define PPC405_ERR77(ra,rb)
#endif /* !__powerpc64__ */

#ifdef CONFIG_SMP
#define EIEIO_ON_SMP    "eieio\n"
#define ISYNC_ON_SMP    "\n\tisync"
#ifdef __powerpc64__
#define LWSYNC_ON_SMP    "lwsync\n"
#else
#define LWSYNC_ON_SMP    "sync\n"
#endif
#else
#define EIEIO_ON_SMP
#define ISYNC_ON_SMP
#define LWSYNC_ON_SMP
#endif

/*
 * Atomic exchange
 *
 * Changes the memory location '*ptr' to be val and returns
 * the previous value stored there.
 *
 * (lifted from linux/include/asm-powerpc/system.h)
 */

static __inline__ unsigned long
    __xchg_u32(volatile void *p, unsigned long val)
{
    unsigned long prev;

    __asm__ __volatile__(
    EIEIO_ON_SMP
"1: lwarx	%0,0,%2 \n"
    PPC405_ERR77(0,%2)
"   stwcx.	%3,0,%2 \n\
    bne-	1b"
    ISYNC_ON_SMP
    : "=&r" (prev), "=m" (*(volatile unsigned int *)p)
    : "r" (p), "r" (val), "m" (*(volatile unsigned int *)p)
    : "cc", "memory");

    return prev;
}

#if defined(__powerpc64__)
static __inline__ unsigned long
    __xchg_u64(volatile void *p, unsigned long val)
{
    unsigned long prev;

    __asm__ __volatile__(
    EIEIO_ON_SMP
"1: ldarx	%0,0,%2 \n"
    PPC405_ERR77(0,%2)
"   stdcx.	%3,0,%2 \n\
    bne-	1b"
    ISYNC_ON_SMP
    : "=&r" (prev), "=m" (*(volatile unsigned long *)p)
    : "r" (p), "r" (val), "m" (*(volatile unsigned long *)p)
    : "cc", "memory");

    return prev;
}
#endif

static __inline__ unsigned long
    __xchg(volatile void *ptr, unsigned long x, unsigned int size)
{
    switch (size) {
    case 4:
	return __xchg_u32(ptr, x);
#if defined(__powerpc64__)
    case 8:
	return __xchg_u64(ptr, x);
#endif
    }
    return x;
}

#define xnarch_atomic_xchg(ptr,x) \
    ({                                                                         \
	__typeof__(*(ptr)) _x_ = (x);                                          \
	(__typeof__(*(ptr))) __xchg((ptr), (unsigned long)_x_, sizeof(*(ptr)));\
    })

#define xnarch_memory_barrier()		__asm__ __volatile__ ("sync" : : : "memory")
#define xnarch_read_memory_barrier()	xnarch_memory_barrier()	/* lwsync would do */
#define xnarch_write_memory_barrier()	xnarch_memory_barrier()
#define cpu_relax()			xnarch_memory_barrier()

#ifdef __powerpc64__
static __inline__ unsigned long
__do_cmpxchg(volatile unsigned long *p,
	     unsigned long old, unsigned long newval)
{
	unsigned long prev;

	__asm__ __volatile__ (
	LWSYNC_ON_SMP
"1:	ldarx	%0,0,%2		# __cmpxchg_u64\n\
	cmpd	0,%0,%3\n\
	bne-	2f\n\
	stdcx.	%4,0,%2\n\
	bne-	1b"
	ISYNC_ON_SMP
	"\n\
2:"
	: "=&r" (prev), "+m" (*p)
	: "r" (p), "r" (old), "r" (newval)
	: "cc", "memory");

	return prev;
}
#else
static __inline__ unsigned long
__do_cmpxchg(volatile unsigned int *p,
	     unsigned long old, unsigned long newval)
{
	unsigned int prev;

	__asm__ __volatile__ (
	LWSYNC_ON_SMP
"1:	lwarx	%0,0,%2		# __cmpxchg_u32\n\
	cmpw	0,%0,%3\n\
	bne-	2f\n"
	PPC405_ERR77(0,%2)
"	stwcx.	%4,0,%2\n\
	bne-	1b"
	ISYNC_ON_SMP
	"\n\
2:"
	: "=&r" (prev), "+m" (*p)
	: "r" (p), "r" (old), "r" (newval)
	: "cc", "memory");

	return prev;
}
#endif

#include <asm/xenomai/features.h>

static __inline__ unsigned long
xnarch_atomic_cmpxchg(xnarch_atomic_t *p,
		      unsigned long old, unsigned long newval)
{
	return __do_cmpxchg(&p->counter, old, newval);
}

#endif /* __KERNEL__ */

#endif /* !_XENO_ASM_POWERPC_ATOMIC_ASM_H */
