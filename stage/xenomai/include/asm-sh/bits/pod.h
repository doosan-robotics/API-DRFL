/*
 * Copyright (C) 2011 Philippe Gerum <rpm@xenomai.org>.
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

#ifndef _XENO_ASM_SH_BITS_POD_H
#define _XENO_ASM_SH_BITS_POD_H

#include <asm-generic/xenomai/bits/pod.h>
#include <asm/xenomai/switch.h>

void xnpod_welcome_thread(struct xnthread *, int);

void xnpod_delete_thread(struct xnthread *);

extern int xnarch_escalation_virq;

static inline int xnarch_start_timer(void (*tick_handler)(void), int cpu)
{
	return rthal_timer_request(tick_handler,
				   xnarch_switch_htick_mode,
				   xnarch_next_htick_shot,
				   cpu);
}

static inline void xnarch_stop_timer(int cpu)
{
	rthal_timer_release(cpu);
}

static inline void xnarch_leave_root(xnarchtcb_t * rootcb)
{
	struct task_struct *p = current;

	rootcb->user_task = rootcb->active_task = p;
	rootcb->tsp = &p->thread;
	rootcb->mm = rootcb->active_mm = rthal_get_active_mm();
#ifdef CONFIG_XENO_HW_FPU
	rootcb->user_fpu_owner = rthal_get_fpu_owner(p);
	rootcb->fpup = rootcb->user_fpu_owner ?
		&rootcb->user_fpu_owner->thread : NULL;
#endif /* CONFIG_XENO_HW_FPU */
}

static inline void xnarch_enter_root(xnarchtcb_t * rootcb)
{
}

static inline void xnarch_switch_to(xnarchtcb_t *out_tcb,
				    xnarchtcb_t *in_tcb)
{
	struct mm_struct *prev_mm = out_tcb->active_mm, *next_mm;
	struct task_struct *prev = out_tcb->active_task;
	struct task_struct *next = in_tcb->user_task;

	if (likely(next != NULL)) {
		in_tcb->active_task = next;
		in_tcb->active_mm = in_tcb->mm;
		rthal_clear_foreign_stack(&rthal_domain);
	} else {
		in_tcb->active_task = prev;
		in_tcb->active_mm = prev_mm;
		rthal_set_foreign_stack(&rthal_domain);
	}

	next_mm = in_tcb->active_mm;

	if (next_mm && likely(prev_mm != next_mm))
		wrap_switch_mm(prev_mm, next_mm, next);

	xnarch_switch_threads(out_tcb, in_tcb, prev, next);
}

asmlinkage void xnarch_thread_trampoline(struct xnarchtcb *tcb)
{
	xnpod_welcome_thread(tcb->self, tcb->imask);
	tcb->entry(tcb->cookie);
	xnpod_delete_thread(tcb->self);
}

static inline void xnarch_init_thread(xnarchtcb_t *tcb,
				      void (*entry) (void *),
				      void *cookie,
				      int imask,
				      struct xnthread *thread, char *name)
{
	unsigned long *sp, sr, gbr;

	/*
	 * Stack space is guaranteed to have been fully zeroed. We do
	 * this earlier in xnthread_init() which runs with interrupts
	 * on, to reduce latency.
	 */
	sp = (void *)tcb->stackbase + tcb->stacksize;
	*--sp = (unsigned long)tcb;
	sr = SR_MD;
#ifdef CONFIG_SH_FPU
	sr |= SR_FD;	/* Disable FPU */
#endif
	*--sp = (unsigned long)sr;
	__asm__ __volatile__ ("stc gbr, %0" : "=r" (gbr));
	*--sp = (unsigned long)gbr;
	tcb->ts.sp = (unsigned long)sp;
	tcb->ts.pc = (unsigned long)rthal_thread_trampoline;
	tcb->entry = entry;
	tcb->cookie = cookie;
	tcb->self = thread;
	tcb->imask = imask;
	tcb->name = name;
}

/* No lazy FPU init on SH. */
#define xnarch_fpu_init_p(task) (1)

#ifdef CONFIG_XENO_HW_FPU

static void xnarch_init_fpu(xnarchtcb_t * tcb)
{
	/*
	 * Initialize the FPU for an emerging kernel-based RT
	 * thread. This must be run on behalf of the emerging thread.
	 * xnarch_init_tcb() guarantees that all FPU regs are zeroed
	 * in tcb.
	 */
	rthal_init_fpu(&tcb->ts);
}

static inline void xnarch_enable_fpu(xnarchtcb_t *tcb)
{
	struct task_struct *task = tcb->user_task;

	if (task && task != tcb->user_fpu_owner)
		rthal_disable_fpu();
	else
		rthal_enable_fpu();
}

static void xnarch_save_fpu(xnarchtcb_t *tcb)
{
	struct pt_regs *regs;

	if (tcb->fpup) {
		rthal_save_fpu(tcb->fpup);
		if (tcb->user_fpu_owner) {
			regs = task_pt_regs(tcb->user_fpu_owner);
			regs->sr |= SR_FD;
		}
	}
}

static void xnarch_restore_fpu(xnarchtcb_t * tcb)
{
	struct pt_regs *regs;

	if (tcb->fpup) {
		rthal_restore_fpu(tcb->fpup);
		/*
		 * Note: Only enable FPU in SR, if it was enabled when
		 * we saved the fpu state.
		 */
		if (tcb->user_fpu_owner) {
			regs = task_pt_regs(tcb->user_fpu_owner);
			regs->sr &= ~SR_FD;
		}
	}

	if (tcb->user_task && tcb->user_task != tcb->user_fpu_owner)
		rthal_disable_fpu();
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

#endif /* !_XENO_ASM_SH_BITS_POD_H */
