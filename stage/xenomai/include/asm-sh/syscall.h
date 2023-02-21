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

#ifndef _XENO_ASM_SH_SYSCALL_H
#define _XENO_ASM_SH_SYSCALL_H

#include <asm-generic/xenomai/syscall.h>
#include <asm/xenomai/tsc.h>

#define __xn_mux_shifted_id(id)	     (id << 24)
#define __xn_mux_code(shifted_id,op) (shifted_id|((op << 16) & 0xff0000)|(__xn_sys_mux & 0xffff))

#ifdef __KERNEL__

#include <linux/errno.h>
#include <asm/ptrace.h>

/* Register mapping for accessing syscall args. */

#define __xn_reg_mux(regs)    ((regs)->regs[3])
#define __xn_reg_rval(regs)   ((regs)->regs[0])
#define __xn_reg_arg1(regs)   ((regs)->regs[4])
#define __xn_reg_arg2(regs)   ((regs)->regs[5])
#define __xn_reg_arg3(regs)   ((regs)->regs[6])
#define __xn_reg_arg4(regs)   ((regs)->regs[7])
#define __xn_reg_arg5(regs)   ((regs)->regs[0])

#define __xn_reg_mux_p(regs)        ((__xn_reg_mux(regs) & 0xffff) == __xn_sys_mux)
#define __xn_mux_id(regs)           ((__xn_reg_mux(regs) >> 24) & 0xff)
#define __xn_mux_op(regs)           ((__xn_reg_mux(regs) >> 16) & 0xff)

#define __xn_linux_mux_p(regs, nr)  (__xn_reg_mux(regs) == (nr))

/*
 * Purposedly used inlines and not macros for the following routines
 * so that we don't risk spurious side-effects on the value arg.
 */
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

#include <errno.h>
#include <asm/xenomai/atomic.h>

/*
 * The following code defines an inline syscall mechanism used by
 * Xenomai's real-time interfaces to invoke the skin module
 * services in kernel space.
 */

#define SYSCALL_INST_STR0	"trapa #0x10\n\t"
#define SYSCALL_INST_STR1	"trapa #0x11\n\t"
#define SYSCALL_INST_STR2	"trapa #0x12\n\t"
#define SYSCALL_INST_STR3	"trapa #0x13\n\t"
#define SYSCALL_INST_STR4	"trapa #0x14\n\t"
#define SYSCALL_INST_STR5	"trapa #0x15\n\t"
#define SYSCALL_INST_STR6	"trapa #0x16\n\t"

/*
 * Conservatively assume that a known SH-4 silicon bug bites us: 4
 * instruction cycles not accessing cache and TLB are needed after
 * trapa instruction.
 */
#define SYSCALL_INST_PAD "	\
	or r0,r0; or r0,r0; or r0,r0; or r0,r0; or r0,r0"

#define ASMFMT_0
#define ASMFMT_1				\
	, "r" (r4)
#define ASMFMT_2				\
	, "r" (r4), "r" (r5)
#define ASMFMT_3				\
	, "r" (r4), "r" (r5), "r" (r6)
#define ASMFMT_4					\
	, "r" (r4), "r" (r5), "r" (r6), "r" (r7)
#define ASMFMT_5						\
	, "r" (r4), "r" (r5), "r" (r6), "r" (r7), "0" (r0)
#define ASMFMT_6							\
	, "r" (r4), "r" (r5), "r" (r6), "r" (r7), "0" (r0), "r" (r1)
#define ASMFMT_7							\
	, "r" (r4), "r" (r5), "r" (r6), "r" (r7), "0" (r0), "r" (r1), "r" (r2)

#define SUBSTITUTE_ARGS_0()
#define SUBSTITUTE_ARGS_1(arg1)					\
	long int _arg1 = (long int) (arg1);			\
	register long int r4 asm ("%r4") = (long int) (_arg1)
#define SUBSTITUTE_ARGS_2(arg1, arg2)				\
	long int _arg1 = (long int) (arg1);			\
	long int _arg2 = (long int) (arg2);			\
	register long int r4 asm ("%r4") = (long int) (_arg1);	\
	register long int r5 asm ("%r5") = (long int) (_arg2)
#define SUBSTITUTE_ARGS_3(arg1, arg2, arg3)			\
	long int _arg1 = (long int) (arg1);			\
	long int _arg2 = (long int) (arg2);			\
	long int _arg3 = (long int) (arg3);			\
	register long int r4 asm ("%r4") = (long int) (_arg1);	\
	register long int r5 asm ("%r5") = (long int) (_arg2);	\
	register long int r6 asm ("%r6") = (long int) (_arg3)
#define SUBSTITUTE_ARGS_4(arg1, arg2, arg3, arg4)		\
	long int _arg1 = (long int) (arg1);			\
	long int _arg2 = (long int) (arg2);			\
	long int _arg3 = (long int) (arg3);			\
	long int _arg4 = (long int) (arg4);			\
	register long int r4 asm ("%r4") = (long int) (_arg1);	\
	register long int r5 asm ("%r5") = (long int) (_arg2);	\
	register long int r6 asm ("%r6") = (long int) (_arg3);	\
	register long int r7 asm ("%r7") = (long int) (_arg4)
#define SUBSTITUTE_ARGS_5(arg1, arg2, arg3, arg4, arg5)		\
	long int _arg1 = (long int) (arg1);			\
	long int _arg2 = (long int) (arg2);			\
	long int _arg3 = (long int) (arg3);			\
	long int _arg4 = (long int) (arg4);			\
	long int _arg5 = (long int) (arg5);			\
	register long int r4 asm ("%r4") = (long int) (_arg1);	\
	register long int r5 asm ("%r5") = (long int) (_arg2);	\
	register long int r6 asm ("%r6") = (long int) (_arg3);	\
	register long int r7 asm ("%r7") = (long int) (_arg4);	\
	register long int r0 asm ("%r0") = (long int) (_arg5)

#define XENOMAI_DO_SYSCALL(nr, shifted_id, op, args...)			\
	({								\
		unsigned long int __ret;				\
		register long int r3 asm ("%r3") = __xn_mux_code(shifted_id, op); \
		SUBSTITUTE_ARGS_##nr(args);				\
									\
		asm volatile (SYSCALL_INST_STR##nr SYSCALL_INST_PAD	\
			      : "=z" (__ret)				\
			      : "r" (r3) ASMFMT_##nr			\
			      : "memory", "t");				\
									\
		(int) __ret;						\
	})

#define XENOMAI_SYSCALL0(op)                XENOMAI_DO_SYSCALL(0,0,op)
#define XENOMAI_SYSCALL1(op,a1)             XENOMAI_DO_SYSCALL(1,0,op,a1)
#define XENOMAI_SYSCALL2(op,a1,a2)          XENOMAI_DO_SYSCALL(2,0,op,a1,a2)
#define XENOMAI_SYSCALL3(op,a1,a2,a3)       XENOMAI_DO_SYSCALL(3,0,op,a1,a2,a3)
#define XENOMAI_SYSCALL4(op,a1,a2,a3,a4)    XENOMAI_DO_SYSCALL(4,0,op,a1,a2,a3,a4)
#define XENOMAI_SYSCALL5(op,a1,a2,a3,a4,a5) XENOMAI_DO_SYSCALL(5,0,op,a1,a2,a3,a4,a5)
#define XENOMAI_SYSBIND(a1,a2,a3,a4)        XENOMAI_DO_SYSCALL(4,0,__xn_sys_bind,a1,a2,a3,a4)

#define XENOMAI_SKINCALL0(id,op)                XENOMAI_DO_SYSCALL(0,id,op)
#define XENOMAI_SKINCALL1(id,op,a1)             XENOMAI_DO_SYSCALL(1,id,op,a1)
#define XENOMAI_SKINCALL2(id,op,a1,a2)          XENOMAI_DO_SYSCALL(2,id,op,a1,a2)
#define XENOMAI_SKINCALL3(id,op,a1,a2,a3)       XENOMAI_DO_SYSCALL(3,id,op,a1,a2,a3)
#define XENOMAI_SKINCALL4(id,op,a1,a2,a3,a4)    XENOMAI_DO_SYSCALL(4,id,op,a1,a2,a3,a4)
#define XENOMAI_SKINCALL5(id,op,a1,a2,a3,a4,a5) XENOMAI_DO_SYSCALL(5,id,op,a1,a2,a3,a4,a5)

#endif /* __KERNEL__ */

#endif /* !_XENO_ASM_SH_SYSCALL_H */
