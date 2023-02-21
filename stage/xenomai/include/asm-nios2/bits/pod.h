/*
 * Copyright (C) 2009 Philippe Gerum <rpm@xenomai.org>.
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

#ifndef _XENO_ASM_NIOS2_BITS_POD_H
#define _XENO_ASM_NIOS2_BITS_POD_H

#include <asm-generic/xenomai/bits/pod.h>

void xnpod_welcome_thread(struct xnthread *, int);

void xnpod_delete_thread(struct xnthread *);

extern int xnarch_escalation_virq;

/*
 * We don't piggyback the kernel timer on NIOS2, but rather use a
 * dedicated hrtimer.
 */
static inline int xnarch_start_timer(void (*tick_handler)(void), int cpu)
{
	return rthal_timer_request(tick_handler, cpu);
}

#define xnarch_stop_timer(cpu)	rthal_timer_release(cpu)

static inline void xnarch_leave_root(struct xnarchtcb *rootcb)
{
	struct task_struct *p = current;

	/* Remember the preempted Linux task pointer. */
	rootcb->user_task = rootcb->active_task = p;
	rootcb->tsp = &p->thread;
	rootcb->mm = rootcb->active_mm = rthal_get_active_mm();
#ifdef CONFIG_XENO_HW_UNLOCKED_SWITCH
	rootcb->tip = task_thread_info(p);
#endif
}

static inline void xnarch_enter_root(struct xnarchtcb * rootcb)
{
#ifdef CONFIG_XENO_HW_UNLOCKED_SWITCH
	if (!rootcb->mm)
		set_ti_thread_flag(rootcb->tip, TIF_MMSWITCH_INT);
#endif
}

static inline void xnarch_switch_to(struct xnarchtcb *out_tcb, struct xnarchtcb *in_tcb)
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

	rthal_thread_switch(out_tcb->tsp, in_tcb->tsp, next == NULL);
	barrier();
}

asmlinkage static void xnarch_thread_trampoline(struct xnarchtcb *tcb)
{
	xnpod_welcome_thread(tcb->self, tcb->imask);
	tcb->entry(tcb->cookie);
	xnpod_delete_thread(tcb->self);
}

static inline void xnarch_init_thread(struct xnarchtcb *tcb,
				      void (*entry) (void *),
				      void *cookie,
				      int imask,
				      struct xnthread *thread, char *name)
{
	register long gp __asm__ ("gp");

	struct tramp_stack {
		struct switch_stack sw;
		unsigned long r4; /* to hold tcb pointer arg */
		unsigned long ra; /* to xnarch_thread_trampoline() */
	} *childregs;

	childregs = (struct tramp_stack *)
		((unsigned long)tcb->stackbase +
		 tcb->stacksize - sizeof(*childregs));
	/*
	 * Stack space is guaranteed to be clean, so no need to zero
	 * it again.
	 */
	childregs->sw.gp = gp;	/* Inherit GP */
	childregs->sw.ra = (unsigned long)&rthal_thread_trampoline;
	childregs->ra = (unsigned long)&xnarch_thread_trampoline;
	childregs->r4 = (unsigned long)tcb;

	tcb->ts.ksp = (unsigned long)childregs;
	tcb->ts.kpsr = 0;	/* PIE=0, U=0, EH=0 */
	tcb->entry = entry;
	tcb->cookie = cookie;
	tcb->self = thread;
	tcb->imask = imask;
	tcb->name = name;
}

#define xnarch_fpu_init_p(task) (0)

static inline void xnarch_enable_fpu(struct xnarchtcb *current_tcb)
{
}

static inline void xnarch_init_fpu(struct xnarchtcb *tcb)
{
}

static inline void xnarch_save_fpu(struct xnarchtcb *tcb)
{
}

static inline void xnarch_restore_fpu(struct xnarchtcb *tcb)
{
}

static inline int xnarch_escalate(void)
{
	if (unlikely(rthal_current_domain == rthal_root_domain)) {
		rthal_trigger_irq(xnarch_escalation_virq);
		return 1;
	}

	return 0;
}

#endif /* !_XENO_ASM_NIOS2_BITS_POD_H */
