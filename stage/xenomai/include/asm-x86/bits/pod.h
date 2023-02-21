/*
 * Copyright (C) 2001,2002,2003 Philippe Gerum <rpm@xenomai.org>.
 * Copyright (C) 2004 The HYADES Project (http://www.hyades-itea.org).
 * Copyright (C) 2004,2005 Gilles Chanteperdrix <gilles.chanteperdrix@xenomai.org>.
 *
 * x86_64 port:
 * Copyright (C) 2001-2007 Philippe Gerum <rpm@xenomai.org>.
 * Copyright (C) 2004-2006 Gilles Chanteperdrix <gilles.chanteperdrix@xenomai.org>.
 *
 * Xenomai is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
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

#ifndef _XENO_ASM_X86_BITS_POD_H
#define _XENO_ASM_X86_BITS_POD_H
#define _XENO_ASM_X86_BITS_POD_H

#include <asm-generic/xenomai/bits/pod.h>
#include <asm/xenomai/switch.h>

void xnpod_welcome_thread(struct xnthread *, int);

void xnpod_delete_thread(struct xnthread *);

#ifdef CONFIG_GENERIC_CLOCKEVENTS
#define xnarch_start_timer(tick_handler, cpu)	\
	rthal_timer_request(tick_handler, xnarch_switch_htick_mode, xnarch_next_htick_shot, cpu)
#else
#define xnarch_start_timer(tick_handler, cpu)	\
	rthal_timer_request(tick_handler, cpu)
#endif

#define xnarch_stop_timer(cpu)	rthal_timer_release(cpu)

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,0,0)
#error "Xenomai x86 FPU support broken on Linux 4.0 and later"
#endif

static inline void xnarch_leave_root(xnarchtcb_t *rootcb)
{
	rthal_root_preempt_notify();

	/* Remember the preempted Linux task pointer. */
	rootcb->user_task = rootcb->active_task = current;
#ifdef CONFIG_X86_64
	rootcb->spp = &current->thread.x86reg_sp;
	rootcb->ipp = &current->thread.rip;
#endif
	rootcb->ts_usedfpu = !!wrap_test_fpu_used(current);
	rootcb->cr0_ts = (read_cr0() & 8) != 0;
	/* So that xnarch_save_fpu() will operate on the right FPU area. */
	if (rootcb->cr0_ts || rootcb->ts_usedfpu)
		rootcb->fpup = x86_fpustate_ptr(&rootcb->user_task->thread);
	else
		/*
		 * The kernel is currently using fpu in kernel-space,
		 * do not clobber the user-space fpu backup area.
		 */
		rootcb->fpup = &rootcb->i387;
}

static inline void xnarch_enter_root(xnarchtcb_t *rootcb) { }

static inline void xnarch_switch_to(xnarchtcb_t *out_tcb, xnarchtcb_t *in_tcb)
{
	struct task_struct *prev = out_tcb->active_task;
	struct task_struct *next = in_tcb->user_task;
#ifndef CONFIG_X86_64
	unsigned long fs, gs;
#endif

	if (likely(next != NULL)) {
		if (wrap_test_fpu_used(prev))
			/*
			 * __switch_to will try and use __unlazy_fpu,
			 * so we need to clear the ts bit.
			 */
			clts();
		in_tcb->active_task = next;
		rthal_clear_foreign_stack(&rthal_domain);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,13,0)
		next->thread.fpu_counter = 0;
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
		next->fpu_counter = 0;
#endif
	} else {
		in_tcb->active_task = prev;
		rthal_set_foreign_stack(&rthal_domain);
	}

	if (next && next != prev) {
		struct mm_struct *oldmm = prev->active_mm;
		wrap_switch_mm(oldmm, next->active_mm, next);
		if (next->mm == NULL)
			wrap_enter_lazy_tlb(oldmm, next);
	}

#ifdef CONFIG_CC_STACKPROTECTOR
#define xnarch_switch_canary  in_tcb->canary
#else
#define xnarch_switch_canary  0
#endif

#ifndef CONFIG_X86_64
	if (out_tcb->user_task) {
		/* Make sure that __switch_to() will always reload the correct
		   %fs and %gs registers, even if we happen to migrate the task
		   across domains in the meantime. */
		asm volatile ("mov %%fs,%0":"=m" (fs));
		asm volatile ("mov %%gs,%0":"=m" (gs));
	}

	xnarch_switch_threads(out_tcb, in_tcb, prev, next);

	if (xnarch_shadow_p(out_tcb, prev)) {
		loadsegment(fs, fs);
		loadsegment(gs, gs);
		barrier();
		/*
		 * Eagerly reinstate the I/O bitmap of any incoming
		 * shadow thread which has previously requested I/O
		 * permissions. We don't want the unexpected latencies
		 * induced by lazy update from the GPF handler to bite
		 * shadow threads that explicitly told the kernel that
		 * they would need to perform raw I/O ops.
		 */
		wrap_switch_iobitmap(prev, rthal_processor_id());
	}
#else  /* CONFIG_X86_64 */
	xnarch_switch_threads(prev, next,
			      out_tcb->spp, in_tcb->spp,
			      out_tcb->ipp, in_tcb->ipp,
			      xnarch_switch_canary);

#endif	/* CONFIG_X86_64 */

	stts();
}

asmlinkage static void xnarch_thread_trampoline(xnarchtcb_t *tcb)
{
	/* xnpod_welcome_thread() will do clts() if needed. */
	stts();
	xnpod_welcome_thread(tcb->self, tcb->imask);
	tcb->entry(tcb->cookie);
	xnpod_delete_thread(tcb->self);

	xnarch_thread_head();
}

static inline void xnarch_init_thread(xnarchtcb_t *tcb,
				      void (*entry)(void *),
				      void *cookie,
				      int imask,
				      struct xnthread *thread, char *name)
{
#ifdef CONFIG_X86_64
	struct xnarch_x8664_initstack *childregs;
	unsigned long *sp, flags;

	/* Prepare the bootstrap stack. */

	rthal_local_irq_flags_hw(flags);

	sp = (unsigned long *)((unsigned long)tcb->stackbase + tcb->stacksize -
			       sizeof(struct xnarch_x8664_initstack) - 8);

	childregs = (struct xnarch_x8664_initstack *)sp;
	childregs->rbp = 0;
	childregs->eflags = flags & ~X86_EFLAGS_IF;
	childregs->arg = (unsigned long)tcb;
	childregs->entry = (unsigned long)xnarch_thread_trampoline;
#ifdef CONFIG_CC_STACKPROTECTOR
	tcb->canary = (unsigned long)xnarch_get_cpu_tsc() ^ childregs->arg;
	childregs->canary = tcb->canary;
#endif
	tcb->sp = (unsigned long)childregs;
	tcb->ip = (unsigned long)__thread_head; /* Will branch there at startup. */
#else /* CONFIG_X86_32 */
	unsigned long **psp = (unsigned long **)&tcb->sp;

	tcb->ip = (unsigned long)xnarch_thread_trampoline;
	tcb->sp = (unsigned long)tcb->stackbase;
	*psp =
		(unsigned long *)(((unsigned long)*psp + tcb->stacksize - 0x10) &
				  ~0xf);
	*--(*psp) = (unsigned long)tcb;
	*--(*psp) = 0;
#endif /* CONFIG_X86_32 */

	tcb->entry = entry;
	tcb->cookie = cookie;
	tcb->self = thread;
	tcb->imask = imask;
	tcb->name = name;
}

#ifdef CONFIG_XENO_HW_FPU

#define xnarch_fpu_init_p(task)   tsk_used_math(task)
#define xnarch_set_fpu_init(task) set_stopped_child_used_math(task)

static inline void xnarch_init_fpu(xnarchtcb_t * tcb)
{
	struct task_struct *task = tcb->user_task;
	/* Initialize the FPU for a task. This must be run on behalf of the
	   task. */

	__asm__ __volatile__("clts; fninit");
	if (cpu_has_xmm) {
		unsigned long __mxcsr;
		__mxcsr = 0x1f80UL & 0xffbfUL;
		__asm__ __volatile__("ldmxcsr %0"::"m"(__mxcsr));
	}

	if (task) {
		/* Real-time shadow FPU initialization: tell Linux
		   that this thread initialized its FPU hardware. The
		   fpu usage bit is necessary for xnarch_save_fpu to
		   save the FPU state at next switch. */
		xnarch_set_fpu_init(task);
		wrap_set_fpu_used(task);
	}
}

#ifdef CONFIG_X86_64
#define XSAVE_PREFIX	"0x48,"
#define XSAVE_SUFFIX	"q"
#else
#define XSAVE_PREFIX
#define XSAVE_SUFFIX
#endif

static inline void __save_i387(x86_fpustate *fpup)
{
#ifdef cpu_has_xsave
	if (cpu_has_xsave) {
#if defined(CONFIG_AS_AVX)
		asm volatile("xsave" XSAVE_SUFFIX " %0"
			     : "=m" (fpup->xsave) : "a" (-1), "d" (-1)
			     : "memory");
#else /* !CONFIG_AS_AVX */
		asm volatile(".byte " XSAVE_PREFIX "0x0f,0xae,0x27"
			     : : "D" (&fpup->xsave), "m" (fpup->xsave),
			         "a" (-1), "d" (-1)
			     : "memory");
#endif /* !CONFIG_AS_AVX */
		return;
	}
#endif /* cpu_has_xsave */
#ifndef CONFIG_X86_64
	if (cpu_has_fxsr)
		__asm__ __volatile__("fxsave %0; fnclex":"=m"(*fpup));
	else
		__asm__ __volatile__("fnsave %0; fwait":"=m"(*fpup));
#else /* CONFIG_X86_64 */
#ifdef CONFIG_AS_FXSAVEQ
	asm volatile("fxsaveq %0" : "=m" (fpup->fxsave));
#else /* !CONFIG_AS_FXSAVEQ */
	asm volatile("rex64/fxsave (%[fx])"
		     : "=m" (fpup->fxsave)
		     : [fx] "R" (&fpup->fxsave));
#endif /* !CONFIG_AS_FXSAVEQ */
#endif /* CONFIG_X86_64 */
}

static inline void xnarch_save_fpu(xnarchtcb_t *tcb)
{
	struct task_struct *task = tcb->user_task;

	if (!tcb->is_root) {
		if (task) {
			/* fpu not used or already saved by __switch_to. */
			if (wrap_test_fpu_used(task) == 0)
				return;

			/* Tell Linux that we already saved the state
			 * of the FPU hardware of this task. */
			wrap_clear_fpu_used(task);
		}
	} else {
		if (tcb->cr0_ts ||
		    (tcb->ts_usedfpu && wrap_test_fpu_used(task) == 0))
			return;

		wrap_clear_fpu_used(task);
	}

	clts();

	__save_i387(tcb->fpup);
}

static inline void __restore_i387(x86_fpustate *fpup)
{
#ifdef cpu_has_xsave
	if (cpu_has_xsave) {
#if defined(CONFIG_AS_AVX)
		asm volatile("xrstor" XSAVE_SUFFIX " %0"
			     : : "m" (fpup->xsave), "a" (-1), "d" (-1)
			     : "memory");
#else /* !CONFIG_AS_AVX */
		asm volatile(".byte " XSAVE_PREFIX "0x0f,0xae,0x2f"
			     : : "D" (&fpup->xsave), "m" (fpup->xsave),
			         "a" (-1), "d" (-1)
			     : "memory");
#endif /* !CONFIG_AS_AVX */
		return;
	}
#endif /* cpu_has_xsave */
#ifndef CONFIG_X86_64
	if (cpu_has_fxsr)
		__asm__ __volatile__("fxrstor %0": /* no output */
				     :"m"(*fpup));
	else
		__asm__ __volatile__("frstor %0": /* no output */
				     :"m"(*fpup));
#else /* CONFIG_X86_64 */
#ifdef CONFIG_AS_FXSAVEQ
	asm volatile("fxrstorq %0" : : "m" (fpup->fxsave));
#else /* !CONFIG_AS_FXSAVEQ */
	asm volatile("rex64/fxrstor (%0)"
		     : : "R" (&fpup->fxsave), "m" (fpup->fxsave));
#endif /* !CONFIG_AS_FXSAVEQ */
#endif /* CONFIG_X86_64 */
}

static inline void xnarch_restore_fpu(xnarchtcb_t * tcb)
{
	struct task_struct *task = tcb->user_task;

	if (!tcb->is_root) {
		if (task) {
			if (!xnarch_fpu_init_p(task)) {
				stts();
				return;	/* Uninit fpu area -- do not restore. */
			}

			/* Tell Linux that this task has altered the state of
			 * the FPU hardware. */
			wrap_set_fpu_used(task);
		}
	} else {
		/* Restore state of FPU only if TS bit in cr0 was clear. */
		if (tcb->cr0_ts) {
			wrap_clear_fpu_used(task);
			stts();
			return;
		}

		if (tcb->ts_usedfpu
		    && wrap_test_fpu_used(task) == 0) {
			/* __switch_to saved the fpu context, no need to restore
			   it since we are switching to root, where fpu can be
			   in lazy state. */
			stts();
			return;
		}
	}

	/* Restore the FPU hardware with valid fp registers from a
	   user-space or kernel thread. */
	clts();

	__restore_i387(tcb->fpup);
}

static inline void xnarch_enable_fpu(xnarchtcb_t *tcb)
{
	struct task_struct *task = tcb->user_task;

	if (!tcb->is_root) {
		if (task) {
			if (!xnarch_fpu_init_p(task))
				return;
			/*
			 * We used to test here if __switch_to had not
			 * saved current fpu state, but this can not
			 * happen, since xnarch_enable_fpu may only be
			 * called when switching back to a user-space
			 * task after one or several switches to
			 * non-fpu kernel-space real-time tasks, so
			 * xnarch_switch_to never uses __switch_to.
			 */
		}
	} else if (tcb->cr0_ts)
		return;

	/* The comment in the non-root case applies here too. */

	clts();
}

#else /* !CONFIG_XENO_HW_FPU */

static inline void xnarch_init_fpu(xnarchtcb_t *tcb)
{
}

static inline void xnarch_save_fpu(xnarchtcb_t *tcb)
{
}

static inline void xnarch_restore_fpu(xnarchtcb_t *tcb)
{
}

static inline void xnarch_enable_fpu(xnarchtcb_t *tcb)
{
}

#endif /* CONFIG_XENO_HW_FPU */

static inline int xnarch_escalate(void)
{
	extern int xnarch_escalation_virq;

	if (rthal_current_domain == rthal_root_domain) {
		rthal_trigger_irq(xnarch_escalation_virq);
		return 1;
	}

	return 0;
}

#endif /* !_XENO_ASM_X86_BITS_POD_H */
