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

#ifndef _XENO_ASM_X86_SWITCH_64_H
#define _XENO_ASM_X86_SWITCH_64_H
#define _XENO_ASM_X86_SWITCH_H

#ifndef __KERNEL__
#error "Pure kernel header included from user-space!"
#endif

struct xnarch_x8664_initstack {

#ifdef CONFIG_CC_STACKPROTECTOR
	unsigned long canary;
#endif
	unsigned long rbp;
	unsigned long eflags;
	unsigned long arg;
	unsigned long entry;
};

#ifdef CONFIG_CC_STACKPROTECTOR
/*
 * We have an added complexity with -fstack-protector, due to the
 * hybrid scheduling between user- and kernel-based Xenomai threads,
 * for which we do not have any underlying task_struct.  We update the
 * current stack canary for incoming user-based threads exactly the
 * same way as Linux does. However, we handle the case of incoming
 * kernel-based Xenomai threads differently: in that case, the canary
 * value shall be given by our caller.
 *
 * In the latter case, the update logic is jointly handled by
 * xeno_switch_kernel_prologue and xeno_switch_kernel_canary; the
 * former clears %rax whenever the incoming thread is kernel-based,
 * and with that information, the latter checks %rax to determine
 * whether the canary should be picked from the current task struct,
 * or from %r8. %r8 is set ealier to the proper canary value passed by
 * our caller (i.e. in_tcb->canary).
 *
 * This code is based on the assumption that no scheduler exchange can
 * happen between Linux and Xenomai for kernel-based Xenomai threads,
 * i.e. the only way to schedule a kernel-based Xenomai thread in goes
 * through xnarch_switch_to(), never via schedule(). This is turn
 * means that neither %rax or %r8 could be clobbered once set by
 * xeno_switch_kernel_prologue, since the only way of branching to the
 * incoming kernel-based thread is via retq. Besides, we may take for
 * granted that %rax can't be null upon return from __switch_to, since
 * the latter returns prev_p, which cannot be NULL, so the logic
 * cannot be confused by incoming user-based threads.
 *
 * Yeah, I know, it's awfully convoluted, but I'm not even sorry for
 * this. --rpm
 */
#define xeno_init_kernel_canary						\
	"popq	%%r8\n\t"						\
	"movq %%r8,"__percpu_arg([gs_canary])"\n\t"
#define xeno_init_canary_oparam						\
	[gs_canary] "=m" (per_cpu_var(irq_stack_union.stack_canary))
#define xeno_switch_canary_setup(c)					\
	register long __kcanary __asm__ ("r8") = (c)
#define xeno_switch_kernel_prologue					\
	"xor %%rax, %%rax\n\t"
#define xeno_switch_kernel_canary					\
	"testq %%rax, %%rax\n\t"					\
	"jnz 8f\n\t"							\
	"movq %%r8,"__percpu_arg([gs_canary])"\n\t"			\
	"jmp 9f\n\t"
#define xeno_switch_user_canary						\
  	"movq "__percpu_arg([current_task])",%%rsi\n\t"			\
	"movq %P[user_canary](%%rsi),%%r8\n\t"				\
	"movq %%r8,"__percpu_arg([gs_canary])"\n\t"
#define xeno_switch_canary_oparam					\
	, [gs_canary] "=m" (per_cpu_var(irq_stack_union.stack_canary))
#define xeno_switch_canary_iparam					\
	, [user_canary] "i" (offsetof(struct task_struct, stack_canary)) \
	, [current_task] "m" (per_cpu_var(current_task))		\
	, "r" (__kcanary)
#define __SWITCH_CLOBBER_LIST  , "r9", "r10", "r11", "r12", "r13", "r14", "r15"
#define __HEAD_CLOBBER_LIST    , "rdi", "r8"
#else	/* CC_STACKPROTECTOR */
#define xeno_switch_canary_setup(c)	(void)(c)
#define xeno_init_kernel_canary
#define xeno_init_canary_oparam
#define xeno_switch_kernel_prologue
#define xeno_switch_kernel_canary
#define xeno_switch_user_canary
#define xeno_switch_canary_oparam
#define xeno_switch_canary_iparam
#define __SWITCH_CLOBBER_LIST  , "r8", "r9", "r10", "r11", "r12", "r13", "r14", "r15"
#define __HEAD_CLOBBER_LIST    , "rdi"
#endif	/* CC_STACKPROTECTOR */

#define xnarch_switch_threads(prev,next,p_rsp,n_rsp,p_rip,n_rip,kcanary)	\
	({								\
		long __rdi, __rsi, __rax, __rbx, __rcx, __rdx;		\
		xeno_switch_canary_setup(kcanary);			\
									\
		asm volatile("pushfq\n\t"				\
			     "pushq	%%rbp\n\t"			\
			     "movq	%%rsi, %%rbp\n\t"		\
			     "movq	%%rsp, (%%rdx)\n\t"		\
			     "movq	$1f, (%%rax)\n\t"		\
			     "movq	(%%rcx), %%rsp\n\t"		\
			     "pushq	(%%rbx)\n\t"			\
			     "cmpq	%%rsi, %%rdi\n\t"		\
			     "jz	0f\n\t"				\
			     "testq	%%rsi, %%rsi\n\t"		\
			     "jnz	__switch_to\n\t"		\
			     xeno_switch_kernel_prologue		\
			     "0:\n\t"					\
			     "ret\n\t"					\
			     "1:\n\t"					\
			     xeno_switch_kernel_canary			\
			     "8:\n\t"					\
			     xeno_switch_user_canary			\
			     "9:\n\t"					\
			     "movq	%%rbp, %%rsi\n\t"		\
			     "popq	%%rbp\n\t"			\
			     "popfq\n\t"				\
			     : "=S" (__rsi), "=D" (__rdi), "=a"	(__rax), \
			       "=b" (__rbx), "=c" (__rcx), "=d" (__rdx)	\
			       xeno_switch_canary_oparam		\
			     : "0" (next), "1" (prev), "5" (p_rsp), "4" (n_rsp), \
			       "2" (p_rip), "3" (n_rip)			\
			       xeno_switch_canary_iparam		\
			     : "memory", "cc" __SWITCH_CLOBBER_LIST);	\
	})

#define xnarch_thread_head()						\
	asm volatile(".globl __thread_head\n\t"				\
		     "__thread_head:\n\t"				\
		     xeno_init_kernel_canary				\
		     "popq	%%rbp\n\t"				\
		     "popfq\n\t"					\
		     "popq	%%rdi\n\t"				\
		     "ret\n\t"						\
		     : xeno_init_canary_oparam				\
		     : /* no input */					\
		     : "cc", "memory" __HEAD_CLOBBER_LIST)

asmlinkage void __thread_head(void);

#endif /* !_XENO_ASM_X86_SWITCH_64_H */
