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

#ifndef _XENO_ASM_ARM_ATOMIC_ASM_H
#define _XENO_ASM_ARM_ATOMIC_ASM_H

#ifndef _XENO_ASM_ARM_ATOMIC_H
#error "please don't include asm/atomic_asm.h directly"
#endif

#if __LINUX_ARM_ARCH__ < 6 && defined(CONFIG_SMP)
#error "SMP not supported below armv6 with ad-hoc atomic operations, compile without SMP or with -march=armv6 or above"
#endif

extern void __xnarch_xchg_called_with_bad_pointer(void);

#define xnarch_read_memory_barrier()		xnarch_memory_barrier()
#define xnarch_write_memory_barrier()		xnarch_memory_barrier()

#if __LINUX_ARM_ARCH__ >= 6
#ifndef CONFIG_SMP
#define xnarch_memory_barrier() \
	__asm__ __volatile__ ("": /* */ : /* */ :"memory")
#else /* SMP */
#if __LINUX_ARM_ARCH__ >= 7
#define xnarch_memory_barrier()	\
	__asm__ __volatile__ ("dmb" : /* */ : /* */ : "memory")
#else /* __LINUX_ARM_ARCH == 6 */
#define xnarch_memory_barrier()	\
	__asm__ __volatile__ ("mcr p15, 0, %0, c7, c10, 5"	\
			      : /* */ : "r" (0) : "memory")
#endif /* __LINUX_ARM_ARCH == 6 */
#endif /* CONFIG_SMP */

#ifndef __KERNEL__
#define cpu_relax()				xnarch_memory_barrier()
#endif /* __KERNEL__ */

static inline unsigned long
__xnarch_xchg(volatile void *ptr, unsigned long x, int size)
{
	unsigned long ret;
	unsigned long tmp;

	xnarch_memory_barrier();

	switch (size) {
	case 1:
		__asm__ __volatile__("@	__xchg1\n"
		"1:	ldrexb	%0, [%4]\n"
		"	strexb	%1, %3, [%4]\n"
		"	teq	%1, #0\n"
		"	bne	1b"
			: "=&r" (ret), "=&r" (tmp),
			  "+Qo" (*(char *)ptr)
			: "r" (x), "r" (ptr)
			: "cc");
		break;
	case 4:
		__asm__ __volatile__("@	__xchg4\n"
		"1:	ldrex	%0, [%4]\n"
		"	strex	%1, %3, [%4]\n"
		"	teq	%1, #0\n"
		"	bne	1b"
			: "=&r" (ret), "=&r" (tmp),
			  "+Qo" (*(unsigned *)ptr)
			: "r" (x), "r" (ptr)
			: "cc");
		break;
	default:
		__xnarch_xchg_called_with_bad_pointer(), ret = 0;
		break;
	}
	xnarch_memory_barrier();

	return ret;
}

#define xnarch_atomic_xchg(ptr,x)					\
    ({									\
	    __typeof__(*(ptr)) _x_ = (x);				\
	    (__typeof__(*(ptr)))					\
		    __xnarch_xchg((ptr),(unsigned long)_x_, sizeof(*(ptr))); \
    })

static inline void xnarch_atomic_inc(xnarch_atomic_t *v)
{
	unsigned long tmp;
	unsigned long result;

	__asm__ __volatile__("@ atomic_add\n"
"1:	ldrex	%0, [%3]\n"
"	add	%0, %0, %4\n"
"	strex	%1, %0, [%3]\n"
"	teq	%1, #0\n"
"	bne	1b"
	: "=&r" (result), "=&r" (tmp), "+Qo" (v->counter)
	: "r" (&v->counter), "Ir" (1)
	: "cc");
}

static inline void xnarch_atomic_dec(xnarch_atomic_t *v)
{
	unsigned long tmp;
	unsigned long result;

	__asm__ __volatile__("@ atomic_sub\n"
"1:	ldrex	%0, [%3]\n"
"	sub	%0, %0, %4\n"
"	strex	%1, %0, [%3]\n"
"	teq	%1, #0\n"
"	bne	1b"
	: "=&r" (result), "=&r" (tmp), "+Qo" (v->counter)
	: "r" (&v->counter), "Ir" (1)
	: "cc");
}

static inline void
xnarch_atomic_set_mask(unsigned long *addr, unsigned long mask)
{
    unsigned long tmp, tmp2;

    __asm__ __volatile__("@ atomic_set_mask\n"
"1:	ldrex	%0, [%3]\n\t"
"	orr	%0, %0, %4\n\t"
"	strex	%1, %0, [%3]\n"
"	teq	%1, #0\n"
"	bne	1b"
	: "=&r" (tmp), "=&r" (tmp2), "+Qo" (*addr)
	: "r" (addr), "Ir" (mask)
	: "cc");
}

static inline void
xnarch_atomic_clear_mask(unsigned long *addr, unsigned long mask)
{
	unsigned long tmp, tmp2;

	__asm__ __volatile__("@ atomic_clear_mask\n"
"1:	ldrex	%0, [%3]\n"
"	bic	%0, %0, %4\n"
"	strex	%1, %0, [%3]\n"
"	teq	%1, #0\n"
"	bne	1b"
	: "=&r" (tmp), "=&r" (tmp2), "+Qo" (*addr)
	: "r" (addr), "Ir" (mask)
	: "cc");
}

static inline unsigned long
xnarch_atomic_cmpxchg(xnarch_atomic_t *ptr,
		      unsigned long oldval, unsigned long newval)
{
	unsigned long curval, res;

	xnarch_memory_barrier();

	do {
		__asm__ __volatile__("@ atomic_cmpxchg\n"
		"ldrex	%1, [%3]\n"
		"mov	%0, #0\n"
		"teq	%1, %4\n"
#ifdef __thumb__
		"it	eq\n"
#endif
		"strexeq %0, %5, [%3]\n"
		    : "=&r" (res), "=&r" (curval), "+Qo" (ptr->counter)
		    : "r" (&ptr->counter), "Ir" (oldval), "r" (newval)
		    : "cc");
	} while (res);

	xnarch_memory_barrier();

	return curval;
}

static inline int xnarch_atomic_inc_and_test(xnarch_atomic_t *v)
{
	unsigned long tmp;
	unsigned long result;

	xnarch_memory_barrier();

	__asm__ __volatile__("@ atomic_add_return\n"
"1:	ldrex	%0, [%3]\n"
"	add	%0, %0, %4\n"
"	strex	%1, %0, [%3]\n"
"	teq	%1, #0\n"
"	bne	1b"
	: "=&r" (result), "=&r" (tmp), "+Qo" (v->counter)
	: "r" (&v->counter), "Ir" (1)
	: "cc");

	xnarch_memory_barrier();

	return result == 0;
}

static inline int xnarch_atomic_dec_and_test(xnarch_atomic_t *v)
{
	unsigned long tmp;
	unsigned long result;

	xnarch_memory_barrier();

	__asm__ __volatile__("@ atomic_sub_return\n"
"1:	ldrex	%0, [%3]\n"
"	sub	%0, %0, %4\n"
"	strex	%1, %0, [%3]\n"
"	teq	%1, #0\n"
"	bne	1b"
	: "=&r" (result), "=&r" (tmp), "+Qo" (v->counter)
	: "r" (&v->counter), "Ir" (1)
	: "cc");

	xnarch_memory_barrier();

	return result == 0;
}
#else /* ARM arch <= 5 */

#ifdef __KERNEL__

#include <linux/bitops.h>
#include <asm/atomic.h>
#include <asm/xenomai/wrappers.h>
#include <asm/xenomai/hal.h>

static inline void
xnarch_atomic_set_mask(unsigned long *addr, unsigned long mask)
{
    unsigned long flags;

    local_irq_save_hw(flags);
    *addr |= mask;
    local_irq_restore_hw(flags);
}

static inline void
xnarch_atomic_clear_mask(unsigned long *addr, unsigned long mask)
{
    unsigned long flags;

    local_irq_save_hw(flags);
    *addr &= ~mask;
    local_irq_restore_hw(flags);
}

#define xnarch_memory_barrier() smp_mb()
#define xnarch_atomic_xchg(ptr,x) xchg(ptr,x)
#define xnarch_atomic_inc(pcounter) \
	atomic_inc((atomic_t *)pcounter)
#define xnarch_atomic_dec(pcounter) \
	atomic_dec((atomic_t *)pcounter)
#define xnarch_atomic_cmpxchg(pcounter, oldval, newval) \
	atomic_cmpxchg((atomic_t *)(pcounter), (oldval), (newval))
#define xnarch_atomic_inc_and_test(pcounter) \
	atomic_inc_and_test((atomic_t *)pcounter)
#define xnarch_atomic_dec_and_test(pcounter) \
	atomic_dec_and_test((atomic_t *)pcounter)

#else /* !__KERNEL__ */

#include <asm/xenomai/syscall.h>
#include <nucleus/compiler.h>

/*
 * This function doesn't exist, so you'll get a linker error
 * if something tries to do an invalid xchg().
 */
static __inline__ unsigned long
__xchg(volatile void *ptr, unsigned long x, unsigned int size)
{
    unsigned long ret;

    if (size != 4) {
	__xnarch_xchg_called_with_bad_pointer();
	return 0;
    }

#if defined(CONFIG_XENO_ARM_SA1100)
    XENOMAI_SYSCALL5(__xn_sys_arch,
		     XENOMAI_SYSARCH_XCHG, ptr, x, size, &ret);
#else
    __asm__ __volatile__("@ __xchg4\n"
"   swp	    %0, %1, [%2]"
    : "=&r" (ret)
    : "r" (x), "r" (ptr)
    : "memory", "cc");
#endif
    return ret;
}

#define xnarch_atomic_xchg(ptr,x) \
    ({									       \
    __typeof__(*(ptr)) _x_ = (x);					   \
    (__typeof__(*(ptr))) __xchg((ptr), (unsigned long)_x_, sizeof(*(ptr))); \
    })


#ifdef CONFIG_SMP
static __inline__ unsigned long
xnarch_atomic_add_return(int i, xnarch_atomic_t *v)
{
    unsigned long ret;

    XENOMAI_SYSCALL4(__xn_sys_arch,
		     XENOMAI_SYSARCH_ATOMIC_ADD_RETURN, i, v, &ret);
    return ret;
}

static __inline__ unsigned long
xnarch_atomic_sub_return(int i, xnarch_atomic_t *v)
{
    unsigned long ret;

    XENOMAI_SYSCALL4(__xn_sys_arch,
		     XENOMAI_SYSARCH_ATOMIC_ADD_RETURN, -i, v, &ret);
    return ret;
}

static inline void
xnarch_atomic_set_mask(unsigned long *addr, unsigned long mask)
{
    XENOMAI_SYSCALL3(__xn_sys_arch,
		     XENOMAI_SYSARCH_ATOMIC_SET_MASK, mask, addr);
}

static inline void
xnarch_atomic_clear_mask(unsigned long *addr, unsigned long mask)
{
    XENOMAI_SYSCALL3(__xn_sys_arch,
		     XENOMAI_SYSARCH_ATOMIC_CLEAR_MASK, mask, addr);
}
#else /* ARM_ARCH <= 5 && !CONFIG_SMP */

static __inline__ unsigned long
xnarch_atomic_cmpxchg(xnarch_atomic_t *ptr,
		      unsigned long oldval, unsigned long newval)
{
	register unsigned long asm_old asm("r0") = oldval;
	register unsigned long asm_new asm("r1") = newval;
	register unsigned long *asm_ptr asm("r2") =
		(unsigned long *)&ptr->counter;
	register unsigned long asm_lr asm("lr");
	register unsigned long asm_tmp asm("r3");

	do {
		__asm__ __volatile__ (
			"mov %1, #0xffff0fff\n\t"
			"mov lr, pc\n\t"
			"add pc, %1, #(0xffff0fc0 - 0xffff0fff)\n\t"
			: "+r"(asm_old), "=&r"(asm_tmp), "=r"(asm_lr)
			: "r"(asm_new), "r"(asm_ptr)
			: "ip", "cc", "memory");
		if (likely(!asm_old))
			return oldval;
	} while ((asm_old = *asm_ptr) == oldval);
	return asm_old;
}

static __inline__ unsigned long
xnarch_atomic_add_return(int i, xnarch_atomic_t *v)
{
	register unsigned long asm_old asm("r0");
	register unsigned long asm_new asm("r1");
	register unsigned long *asm_ptr asm("r2") =
		(unsigned long *)&v->counter;
	register unsigned long asm_lr asm("lr");
	register unsigned long asm_tmp asm("r3");

	__asm__ __volatile__ ( \
		"1: @ xnarch_atomic_add\n\t"
		"ldr	%0, [%4]\n\t"
		"mov	%1, #0xffff0fff\n\t"
		"add	lr, pc, #4\n\t"
		"add	%3, %0, %5\n\t"
		"add	pc, %1, #(0xffff0fc0 - 0xffff0fff)\n\t"
		"bcc	1b"
		: "=&r" (asm_old), "=&r"(asm_tmp), "=r"(asm_lr), "=r"(asm_new)
		: "r" (asm_ptr), "rIL"(i)
		: "ip", "cc", "memory");
	return asm_new;
}

static __inline__ unsigned long
xnarch_atomic_sub_return(int i, xnarch_atomic_t *v)
{
	register unsigned long asm_old asm("r0");
	register unsigned long asm_new asm("r1");
	register unsigned long *asm_ptr asm("r2") =
		(unsigned long *)&v->counter;
	register unsigned long asm_lr asm("lr");
	register unsigned long asm_tmp asm("r3");

	__asm__ __volatile__ ( \
		"1: @ xnarch_atomic_sub\n\t"
		"ldr	%0, [%4]\n\t"
		"mov	%1, #0xffff0fff\n\t"
		"add	lr, pc, #4\n\t"
		"sub	%3, %0, %5\n\t"
		"add	pc, %1, #(0xffff0fc0 - 0xffff0fff)\n\t"
		"bcc	1b"
		: "=&r" (asm_old), "=&r"(asm_tmp), "=r"(asm_lr), "=r"(asm_new)
		: "r" (asm_ptr), "rIL"(i)
		: "ip", "cc", "memory");
	return asm_new;
}

static __inline__ void
xnarch_atomic_set_mask(xnarch_atomic_t *v, long mask)
{
	register unsigned long asm_old asm("r0");
	register unsigned long asm_new asm("r1");
	register unsigned long *asm_ptr asm("r2") =
		(unsigned long *)&v->counter;
	register unsigned long asm_lr asm("lr");
	register unsigned long asm_tmp asm("r3");

	__asm__ __volatile__ ( \
		"1: @ xnarch_atomic_set_mask\n\t" \
		"ldr	%0, [%4]\n\t" \
		"mov	%1, #0xffff0fff\n\t" \
		"add	lr, pc, #4\n\t" \
		"orr	%3, %0, %5\n\t"\
		"add	pc, %1, #(0xffff0fc0 - 0xffff0fff)\n\t" \
		"bcc	1b" \
		: "=&r" (asm_old), "=&r"(asm_tmp), "=r"(asm_lr), "=r"(asm_new) \
		: "r" (asm_ptr), "rIL"(mask) \
		: "ip", "cc", "memory");
}

static __inline__ void
xnarch_atomic_clear_mask(xnarch_atomic_t *v, long mask)
{
	register unsigned long asm_old asm("r0");
	register unsigned long asm_new asm("r1");
	register unsigned long *asm_ptr asm("r2") =
		(unsigned long *)&v->counter;
	register unsigned long asm_lr asm("lr");
	register unsigned long asm_tmp asm("r3");

	__asm__ __volatile__ ( \
		"1: @ xnarch_atomic_clear_mask\n\t" \
		"ldr	%0, [%4]\n\t" \
		"mov	%1, #0xffff0fff\n\t" \
		"add	lr, pc, #4\n\t" \
		"bic	%3, %0, %5\n\t" \
		"add	pc, %1, #(0xffff0fc0 - 0xffff0fff)\n\t" \
		"bcc	1b" \
		: "=&r" (asm_old), "=&r"(asm_tmp), "=r"(asm_lr), "=r"(asm_new) \
		: "r" (asm_ptr), "rIL"(mask) \
		: "ip", "cc", "memory");
}


#endif /* ARM_ARCH <= 5 && !CONFIG_SMP */

#if defined(CONFIG_SMP) && defined(CONFIG_XENO_CPU_XSC3)
#define xnarch_memory_barrier() \
	__asm__ __volatile__ ("mcr p15, 0, %0, c7, c10, 5" \
			      : /* */ : "r" (0) : "memory")
#else /* !XSC3 || !SMP */
#define xnarch_memory_barrier() \
	__asm__ __volatile__ ("": /* */ : /* */ :"memory")
#endif /* !XSC3 || !SMP */

#define cpu_relax()				xnarch_memory_barrier()

#define xnarch_atomic_inc(pcounter)		(void) xnarch_atomic_add_return(1, pcounter)
#define xnarch_atomic_dec_and_test(pcounter)	(xnarch_atomic_sub_return(1, pcounter) == 0)
#endif /* __KERNEL__ */
#endif /* ARM arch <= 5 */

#endif /* !_XENO_ASM_ARM_ATOMIC_ASM_H */
