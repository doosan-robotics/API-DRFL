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

#ifndef _XENO_ASM_SH_ATOMIC_H
#define _XENO_ASM_SH_ATOMIC_H

#ifdef __KERNEL__

#include <linux/bitops.h>
#include <asm/atomic.h>
#include <asm/system.h>

#define xnarch_atomic_xchg(ptr,v)	xchg(ptr,v)
#define xnarch_memory_barrier()		smp_mb()
#define xnarch_read_memory_barrier()	rmb()
#define xnarch_write_memory_barrier()	wmb()

#define xnarch_atomic_set(pcounter,i)          atomic_set(pcounter,i)
#define xnarch_atomic_get(pcounter)            atomic_read(pcounter)
#define xnarch_atomic_inc(pcounter)            atomic_inc(pcounter)
#define xnarch_atomic_dec(pcounter)            atomic_dec(pcounter)
#define xnarch_atomic_inc_and_test(pcounter)   atomic_inc_and_test(pcounter)
#define xnarch_atomic_dec_and_test(pcounter)   atomic_dec_and_test(pcounter)
#define xnarch_atomic_set_mask(pflags,mask)    atomic_set_mask(mask, (atomic_t *)(pflags))
#define xnarch_atomic_clear_mask(pflags,mask)  atomic_clear_mask(mask, (atomic_t *)(pflags))
#define xnarch_atomic_cmpxchg(pcounter,old,new) atomic_cmpxchg((pcounter), (old), (new))

typedef atomic_t atomic_counter_t;
typedef atomic_t xnarch_atomic_t;

#else /* !__KERNEL__ */

#include <asm/xenomai/features.h>
#include <endian.h>

typedef struct { int counter; } xnarch_atomic_t;

#define xnarch_atomic_get(v)		((v)->counter)
#define xnarch_atomic_set(v, i)		(((v)->counter) = i)

/*
 * Shamelessly lifted from the gUSA-compliant xchg() code in kernel
 * space. NOTE: we DO need GUSA_RB available on the platform for this
 * to work.
 */
static inline unsigned long xchg_u32(volatile unsigned int *m, unsigned long val)
{
	unsigned long retval;

	__asm__ __volatile__ (
		"   .align 2              \n\t"
		"   mova    1f,   r0      \n\t" /* r0 = end point */
		"   nop                   \n\t"
		"   mov    r15,   r1      \n\t" /* r1 = saved sp */
		"   mov    #-4,   r15     \n\t" /* LOGIN */
		"   mov.l  @%1,   %0      \n\t" /* load  old value */
		"   mov.l   %2,   @%1     \n\t" /* store new value */
		"1: mov     r1,   r15     \n\t" /* LOGOUT */
		: "=&r" (retval),
		  "+r"  (m)
		: "r"   (val)
		: "memory", "r0", "r1");

	return retval;
}

#define __do_xchg(ptr, x)				\
({							\
	unsigned long __xchg__res;			\
	volatile void *__xchg_ptr = (ptr);		\
	__xchg__res = xchg_u32(__xchg_ptr, x);		\
	__xchg__res;					\
})

#define xnarch_atomic_xchg(ptr,x)	\
	((__typeof__(*(ptr)))__do_xchg((ptr),(unsigned long)(x)))

static inline unsigned long long load_u64(volatile void *p)
{
	union {
#if __BYTE_ORDER == __BIG_ENDIAN
		struct {
			unsigned long high;
			unsigned long low;
		} e;
#else /* __LITTLE_ENDIAN */
		struct {
			unsigned long low;
			unsigned long high;
		} e;
#endif /* __LITTLE_ENDIAN */
		struct {
			unsigned long l1;
			unsigned long l2;
		} v;
	} u;

	__asm__ __volatile__ (
		"   .align 2              \n\t"
		"   mova    1f,   r0      \n\t" /* r0 = end point */
		"   nop                   \n\t"
		"   mov    r15,   r1      \n\t" /* r1 = saved sp */
		"   mov    #-4,   r15     \n\t" /* LOGIN */
		"   mov.l  @%2,   %0      \n\t" /* load first 32bit word */
		"   mov.l  @(4, %2),   %1 \n\t" /* load second 32bit word */
		"1: mov     r1,   r15     \n\t" /* LOGOUT */
		: "=&r" (u.v.l1), "=&r" (u.v.l2)
		: "r" (p)
		: "memory", "r0", "r1");

	return ((unsigned long long)u.e.high << 32) | u.e.low;
}

#define xnarch_memory_barrier()     __asm__ __volatile__("": : :"memory")

#define cpu_relax()			xnarch_memory_barrier()
#define xnarch_read_memory_barrier()	xnarch_memory_barrier()
#define xnarch_write_memory_barrier()	xnarch_memory_barrier()

#endif /* __KERNEL__ */

typedef unsigned long atomic_flags_t;

#endif /* !_XENO_ASM_SH_ATOMIC_H */
