/*
 * Copyright (C) 2001,2002,2003,2004 Philippe Gerum <rpm@xenomai.org>.
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

#ifndef _XENO_ASM_ARM_SYSCALL_H
#define _XENO_ASM_ARM_SYSCALL_H

#include <asm-generic/xenomai/syscall.h>
#include <asm/xenomai/features.h>
#include <asm/xenomai/tsc.h>

#define __xn_mux_code(shifted_id,op) ((op << 24)|shifted_id|(__xn_sys_mux & 0xffff))
#define __xn_mux_shifted_id(id) ((id << 16) & 0xff0000)

#define XENO_ARM_SYSCALL        0x000F0042	/* carefully chosen... */

#ifdef __KERNEL__

#include <linux/errno.h>
#include <asm/uaccess.h>
#include <asm/ptrace.h>

/* Register mapping for accessing syscall args. */

#define __xn_reg_mux(regs)      ((regs)->ARM_ORIG_r0)
#define __xn_reg_rval(regs)     ((regs)->ARM_r0)
#define __xn_reg_arg1(regs)     ((regs)->ARM_r1)
#define __xn_reg_arg2(regs)     ((regs)->ARM_r2)
#define __xn_reg_arg3(regs)     ((regs)->ARM_r3)
#define __xn_reg_arg4(regs)     ((regs)->ARM_r4)
#define __xn_reg_arg5(regs)     ((regs)->ARM_r5)

/* In OABI_COMPAT mode, handle both OABI and EABI userspace syscalls */
#ifdef CONFIG_OABI_COMPAT
#define __xn_reg_mux_p(regs)    ( ((regs)->ARM_r7 == __NR_OABI_SYSCALL_BASE + XENO_ARM_SYSCALL) || \
				  ((regs)->ARM_r7 == __NR_SYSCALL_BASE + XENO_ARM_SYSCALL) )
#define __xn_linux_mux_p(regs, nr) \
				( ((regs)->ARM_r7 == __NR_OABI_SYSCALL_BASE + (nr)) || \
				  ((regs)->ARM_r7 == __NR_SYSCALL_BASE + (nr)) )
#else /* !CONFIG_OABI_COMPAT */
#define __xn_reg_mux_p(regs)      ((regs)->ARM_r7 == __NR_SYSCALL_BASE + XENO_ARM_SYSCALL)
#define __xn_linux_mux_p(regs, nr) ((regs)->ARM_r7 == __NR_SYSCALL_BASE + (nr))
#endif /* !CONFIG_OABI_COMPAT */

#define __xn_mux_id(regs)       ((__xn_reg_mux(regs) >> 16) & 0xff)
#define __xn_mux_op(regs)       ((__xn_reg_mux(regs) >> 24) & 0xff)

/* Purposedly used inlines and not macros for the following routines
   so that we don't risk spurious side-effects on the value arg. */

static inline void __xn_success_return(struct pt_regs *regs, int v)
{
	__xn_reg_rval(regs) = v;
}

static inline void __xn_error_return(struct pt_regs *regs, int v)
{
	__xn_reg_rval(regs) = v;
}

static inline void __xn_status_return(struct pt_regs *regs, int v)
{
	__xn_reg_rval(regs) = v;
}

static inline int __xn_interrupted_p(struct pt_regs *regs)
{
	return __xn_reg_rval(regs) == -EINTR;
}

#else /* !__KERNEL__ */

#include <errno.h>		/* For -ERESTART */

/*
 * Some of the following macros have been adapted from Linux's
 * implementation of the syscall mechanism in <asm-arm/unistd.h>:
 *
 * The following code defines an inline syscall mechanism used by
 * Xenomai's real-time interfaces to invoke the skin module
 * services in kernel space.
 */

#if defined(HAVE___THREAD) && __GNUC__ == 4 && __GNUC_MINOR__ >= 3
#error __thread is too buggy with gcc 4.3 and later, please do not pass --with-__thread to configure
#endif

#define LOADARGS_0(muxcode, dummy...)	\
	__a0 = (unsigned long) (muxcode)
#define LOADARGS_1(muxcode, arg1)	\
	LOADARGS_0(muxcode);		\
	__a1 = (unsigned long) (arg1)
#define LOADARGS_2(muxcode, arg1, arg2)	\
	LOADARGS_1(muxcode, arg1);	\
	__a2 = (unsigned long) (arg2)
#define LOADARGS_3(muxcode, arg1, arg2, arg3) 	\
	LOADARGS_2(muxcode,  arg1, arg2);	\
	__a3 = (unsigned long) (arg3)
#define LOADARGS_4(muxcode,  arg1, arg2, arg3, arg4)	\
	LOADARGS_3(muxcode,  arg1, arg2, arg3);		\
	__a4 = (unsigned long) (arg4)
#define LOADARGS_5(muxcode, arg1, arg2, arg3, arg4, arg5)	\
	LOADARGS_4(muxcode, arg1, arg2, arg3, arg4);		\
	__a5 = (unsigned long) (arg5)

#define CLOBBER_REGS_0 "r0"
#define CLOBBER_REGS_1 CLOBBER_REGS_0, "r1"
#define CLOBBER_REGS_2 CLOBBER_REGS_1, "r2"
#define CLOBBER_REGS_3 CLOBBER_REGS_2, "r3"
#define CLOBBER_REGS_4 CLOBBER_REGS_3, "r4"
#define CLOBBER_REGS_5 CLOBBER_REGS_4, "r5"

#define LOADREGS_0 __r0 = __a0
#define LOADREGS_1 LOADREGS_0; __r1 = __a1
#define LOADREGS_2 LOADREGS_1; __r2 = __a2
#define LOADREGS_3 LOADREGS_2; __r3 = __a3
#define LOADREGS_4 LOADREGS_3; __r4 = __a4
#define LOADREGS_5 LOADREGS_4; __r5 = __a5

#define ASM_INDECL_0							\
	unsigned long __a0; register unsigned long __r0  __asm__ ("r0");
#define ASM_INDECL_1 ASM_INDECL_0;					\
	unsigned long __a1; register unsigned long __r1  __asm__ ("r1")
#define ASM_INDECL_2 ASM_INDECL_1;					\
	unsigned long __a2; register unsigned long __r2  __asm__ ("r2")
#define ASM_INDECL_3 ASM_INDECL_2;					\
	unsigned long __a3; register unsigned long __r3  __asm__ ("r3")
#define ASM_INDECL_4 ASM_INDECL_3;					\
	unsigned long __a4; register unsigned long __r4  __asm__ ("r4")
#define ASM_INDECL_5 ASM_INDECL_4;					\
	unsigned long __a5; register unsigned long __r5  __asm__ ("r5")

#define ASM_INPUT_0 "0" (__r0)
#define ASM_INPUT_1 ASM_INPUT_0, "r" (__r1)
#define ASM_INPUT_2 ASM_INPUT_1, "r" (__r2)
#define ASM_INPUT_3 ASM_INPUT_2, "r" (__r3)
#define ASM_INPUT_4 ASM_INPUT_3, "r" (__r4)
#define ASM_INPUT_5 ASM_INPUT_4, "r" (__r5)

#define __sys2(x)	#x
#define __sys1(x)	__sys2(x)

#ifdef CONFIG_XENO_ARM_EABI
#define __SYS_REG , "r7"
#define __SYS_REG_DECL register unsigned long __r7 __asm__ ("r7")
#define __SYS_REG_SET __r7 = XENO_ARM_SYSCALL
#define __SYS_REG_INPUT ,"r" (__r7)
#define __xn_syscall "swi\t0"
#else
#define __SYS_REG
#define __SYS_REG_DECL
#define __SYS_REG_SET
#define __SYS_REG_INPUT
#define __NR_OABI_SYSCALL_BASE	0x900000
#define __xn_syscall "swi\t" __sys1(__NR_OABI_SYSCALL_BASE + XENO_ARM_SYSCALL) ""
#endif

#define XENOMAI_DO_SYSCALL(nr, shifted_id, op, args...)			\
	({								\
		ASM_INDECL_##nr;					\
		__SYS_REG_DECL;						\
		LOADARGS_##nr(__xn_mux_code(shifted_id,op), args);	\
		__asm__ __volatile__ ("" : /* */ : /* */ :		\
				      CLOBBER_REGS_##nr __SYS_REG);	\
		LOADREGS_##nr;						\
		__SYS_REG_SET;						\
		__asm__ __volatile__ (					\
			__xn_syscall					\
			: "=r" (__r0)					\
			: ASM_INPUT_##nr __SYS_REG_INPUT		\
			: "memory");					\
		(int) __r0;						\
	})

#define XENOMAI_SYSCALL0(op)			\
	XENOMAI_DO_SYSCALL(0,0,op)
#define XENOMAI_SYSCALL1(op,a1)			\
	XENOMAI_DO_SYSCALL(1,0,op,a1)
#define XENOMAI_SYSCALL2(op,a1,a2)		\
	XENOMAI_DO_SYSCALL(2,0,op,a1,a2)
#define XENOMAI_SYSCALL3(op,a1,a2,a3)		\
	XENOMAI_DO_SYSCALL(3,0,op,a1,a2,a3)
#define XENOMAI_SYSCALL4(op,a1,a2,a3,a4)	\
	XENOMAI_DO_SYSCALL(4,0,op,a1,a2,a3,a4)
#define XENOMAI_SYSCALL5(op,a1,a2,a3,a4,a5)		\
	XENOMAI_DO_SYSCALL(5,0,op,a1,a2,a3,a4,a5)
#define XENOMAI_SYSBIND(a1,a2,a3,a4)				\
	XENOMAI_DO_SYSCALL(4,0,__xn_sys_bind,a1,a2,a3,a4)

#define XENOMAI_SKINCALL0(id,op)		\
	XENOMAI_DO_SYSCALL(0,id,op)
#define XENOMAI_SKINCALL1(id,op,a1)		\
	XENOMAI_DO_SYSCALL(1,id,op,a1)
#define XENOMAI_SKINCALL2(id,op,a1,a2)		\
	XENOMAI_DO_SYSCALL(2,id,op,a1,a2)
#define XENOMAI_SKINCALL3(id,op,a1,a2,a3)	\
	XENOMAI_DO_SYSCALL(3,id,op,a1,a2,a3)
#define XENOMAI_SKINCALL4(id,op,a1,a2,a3,a4)	\
	XENOMAI_DO_SYSCALL(4,id,op,a1,a2,a3,a4)
#define XENOMAI_SKINCALL5(id,op,a1,a2,a3,a4,a5)		\
	XENOMAI_DO_SYSCALL(5,id,op,a1,a2,a3,a4,a5)

#ifdef CONFIG_XENO_ARM_TSC_TYPE
#define XNARCH_HAVE_NONPRIV_TSC  1
#endif /* CONFIG_XENO_ARM_TSC_TYPE */

#endif /* __KERNEL__ */

#define XENOMAI_SYSARCH_ATOMIC_ADD_RETURN	0
#define XENOMAI_SYSARCH_ATOMIC_SET_MASK		1
#define XENOMAI_SYSARCH_ATOMIC_CLEAR_MASK	2
#define XENOMAI_SYSARCH_XCHG			3
#define XENOMAI_SYSARCH_TSCINFO                 4

#endif /* !_XENO_ASM_ARM_SYSCALL_H */
