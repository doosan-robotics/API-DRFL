/*
 * Copyright (C) 2005 Philippe Gerum <rpm@xenomai.org>.
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

#ifndef _XENO_ASM_BLACKFIN_BITS_POD_H
#define _XENO_ASM_BLACKFIN_BITS_POD_H

#include <asm-generic/xenomai/bits/pod.h>

void xnpod_welcome_thread(struct xnthread *, int);

void xnpod_delete_thread(struct xnthread *);

#ifndef CONFIG_GENERIC_CLOCKEVENTS
#define xnarch_switch_htick_mode NULL
#define xnarch_next_htick_shot NULL
#endif /* CONFIG_GENERIC_CLOCKEVENTS */

/*
 * When GENERIC_CLOCKEVENTS are not available, the I-pipe frees the
 * Blackfin core timer for us, therefore we don't need any host tick
 * relay service since the regular Linux time source is still ticking
 * in parallel at the normal pace through TIMER0.
 */
static inline int xnarch_start_timer(void (*tick_handler)(void), int cpu)
{
	return rthal_timer_request(tick_handler,
				   xnarch_switch_htick_mode, xnarch_next_htick_shot,
				   cpu);
}

#define xnarch_stop_timer(cpu)	rthal_timer_release(cpu)

static inline void xnarch_leave_root(xnarchtcb_t * rootcb)
{
	/* Remember the preempted Linux task pointer. */
	rootcb->user_task = current;
#ifdef CONFIG_MPU
	rootcb->active_task = current;
#endif
	rootcb->tsp = &current->thread;
}

static inline void xnarch_enter_root(xnarchtcb_t * rootcb)
{
}

#ifdef CONFIG_MPU

static inline
struct task_struct *mpu_get_prev(struct xnarchtcb *tcb)
{
	return tcb->active_task;
}

static inline
void mpu_set_next(struct xnarchtcb *tcb,
		  struct task_struct *next)
{
	tcb->active_task = next;
}

static inline void mpu_switch(struct task_struct *prev,
			      struct task_struct *next)
{
	if (next && next != prev) {
		struct mm_struct *oldmm = prev->active_mm;
		wrap_switch_mm(oldmm, next->active_mm, next);
	}
}

#else /* !CONFIG_MPU */

static inline
struct task_struct *mpu_get_prev(struct xnarchtcb *tcb)
{
	return NULL;
}

static inline
void mpu_set_next(struct xnarchtcb *tcb,
		  struct task_struct *next)
{
}

static inline void mpu_switch(struct task_struct *prev,
			      struct task_struct *next)
{
}

#endif /* CONFIG_MPU */

static inline void xnarch_switch_to(xnarchtcb_t * out_tcb, xnarchtcb_t * in_tcb)
{
	struct task_struct *prev = mpu_get_prev(out_tcb);
	struct task_struct *next = in_tcb->user_task;

	if (likely(next != NULL)) {
		mpu_set_next(in_tcb, next);
		rthal_clear_foreign_stack(&rthal_domain);
	} else {
		mpu_set_next(in_tcb, prev);
		rthal_set_foreign_stack(&rthal_domain);
	}

	mpu_switch(prev, next);

	rthal_thread_switch(out_tcb->tsp, in_tcb->tsp);
}

asmlinkage static void xnarch_thread_trampoline(xnarchtcb_t * tcb)
{
	xnpod_welcome_thread(tcb->self, tcb->imask);
	tcb->entry(tcb->cookie);
	xnpod_delete_thread(tcb->self);
}

static inline void xnarch_init_thread(xnarchtcb_t * tcb,
				      void (*entry) (void *),
				      void *cookie,
				      int imask,
				      struct xnthread *thread, char *name)
{
	unsigned long ksp, *switchregs;

	ksp = (((unsigned long)tcb->stackbase + tcb->stacksize - 40) & ~0xf);
	switchregs = (unsigned long *)ksp;
	/*
	 * Stack space is guaranteed to be clear, so R7:4, P5:3, fp
	 * are already zero. We only need to set r0 and rets.
	 */
	switchregs[0] = (unsigned long)tcb;	/* r0 */
	switchregs[9] = (unsigned long)&xnarch_thread_trampoline; /* rets */

	tcb->ts.ksp = ksp;
	tcb->ts.pc = (unsigned long)&rthal_thread_trampoline;
	tcb->ts.usp = 0;

	tcb->entry = entry;
	tcb->cookie = cookie;
	tcb->self = thread;
	tcb->imask = imask;
	tcb->name = name;
}

#define xnarch_fpu_init_p(task) (0)

static inline void xnarch_enable_fpu(xnarchtcb_t * current_tcb)
{
}

static inline void xnarch_init_fpu(xnarchtcb_t * tcb)
{
}

static inline void xnarch_save_fpu(xnarchtcb_t * tcb)
{
}

static inline void xnarch_restore_fpu(xnarchtcb_t * tcb)
{
}

static inline int xnarch_escalate(void)
{
	extern int xnarch_escalation_virq;

	/* The following Blackfin-specific check is likely the most
	 * braindamage stuff we need to do for this arch, i.e. deferring
	 * Xenomai's rescheduling procedure whenever:

	 * 1. ILAT tells us that a deferred syscall (EVT15) is pending, so
	 * that we don't later execute this syscall over the wrong thread
	 * context. This could happen whenever a user-space task (plain or
	 * Xenomai) gets preempted by a high priority interrupt right
	 * after the deferred syscall event is raised (EVT15) but before
	 * the evt_system_call ISR could run. In case of deferred Xenomai
	 * rescheduling, the pending rescheduling opportunity will be
	 * checked at the beginning of Xenomai's do_hisyscall_event which
	 * intercepts any incoming syscall, and we know it will happen
	 * shortly after.
	 *
	 * 2. the context we will switch back to belongs to the Linux
	 * kernel code, so that we don't inadvertently cause the CPU to
	 * switch to user operating mode as a result of returning from an
	 * interrupt stack frame over the incoming thread through RTI. In
	 * the latter case, the preempted kernel code will be diverted
	 * shortly before resumption in order to run the rescheduling
	 * procedure (see __ipipe_irq_tail_hook).
	 */

	if (rthal_defer_switch_p()) {
		__ipipe_lock_root();
		return 1;
	}

	if (rthal_current_domain == rthal_root_domain) {
		rthal_trigger_irq(xnarch_escalation_virq);
		__ipipe_unlock_root();
		return 1;
	}

	__ipipe_unlock_root();

	return 0;
}

#endif /* !_XENO_ASM_BLACKFIN_BITS_POD_H */
