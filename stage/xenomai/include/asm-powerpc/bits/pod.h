/*
 * Copyright (C) 2001,2002,2003,2004 Philippe Gerum <rpm@xenomai.org>.
 *
 * 64-bit PowerPC adoption
 *   copyright (C) 2005 Taneli Vähäkangas and Heikki Lindholm
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

#ifndef _XENO_ASM_POWERPC_BITS_POD_H
#define _XENO_ASM_POWERPC_BITS_POD_H

#include <asm-generic/xenomai/bits/pod.h>

void xnpod_welcome_thread(struct xnthread *, int);

void xnpod_delete_thread(struct xnthread *);

#ifdef CONFIG_GENERIC_CLOCKEVENTS
#define xnarch_start_timer(tick_handler, cpu)	\
	rthal_timer_request(tick_handler, xnarch_switch_htick_mode, xnarch_next_htick_shot, cpu)
#else
#define xnarch_start_timer(tick_handler, cpu)	\
	({ int __tickval = rthal_timer_request(tick_handler, cpu) ?: \
			(1000000000UL/HZ); __tickval; })
#endif

#define xnarch_stop_timer(cpu)	rthal_timer_release(cpu)

#ifdef CONFIG_PPC64

/* from process.c/copy_thread */
unsigned long get_stack_vsid(unsigned long ksp)
{
	unsigned long vsid;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
	vsid = get_kernel_vsid(ksp);
	vsid <<= SLB_VSID_SHIFT;
	vsid |= SLB_VSID_KERNEL;
	if (cpu_has_feature(CPU_FTR_16M_PAGE))
		vsid |= SLB_VSID_L;
#else /* LINUX_VERSION_CODE >= 2.6.24 */
	unsigned long llp = mmu_psize_defs[mmu_linear_psize].sllp;

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)
	if (cpu_has_feature(CPU_FTR_1T_SEGMENT))
#else
	if (mmu_has_feature(MMU_FTR_1T_SEGMENT))
#endif
		vsid = get_kernel_vsid(ksp, MMU_SEGSIZE_1T)
			<< SLB_VSID_SHIFT_1T;
	else
		vsid = get_kernel_vsid(ksp, MMU_SEGSIZE_256M)
			<< SLB_VSID_SHIFT;
	vsid |= SLB_VSID_KERNEL | llp;
#endif /* LINUX_VERSION_CODE >= 2.6.24 */

	return vsid;
}

#endif /* CONFIG_PPC64 */

static inline void xnarch_leave_root(xnarchtcb_t * rootcb)
{
	struct task_struct *p = current;

	rthal_mute_pic();
	/* Remember the preempted Linux task pointer. */
	rootcb->user_task = rootcb->active_task = p;
	rootcb->tsp = &p->thread;
	rootcb->mm = rootcb->active_mm = rthal_get_active_mm();
#ifdef CONFIG_XENO_HW_UNLOCKED_SWITCH
	rootcb->tip = task_thread_info(p);
#endif
#ifdef CONFIG_XENO_HW_FPU
	rootcb->user_fpu_owner = rthal_get_fpu_owner(rootcb->user_task);
	/* So that xnarch_save_fpu() will operate on the right FPU area. */
	rootcb->fpup = rootcb->user_fpu_owner ?
		&rootcb->user_fpu_owner->thread : NULL;
#endif /* CONFIG_XENO_HW_FPU */
}

static inline void xnarch_enter_root(xnarchtcb_t * rootcb)
{
#ifdef CONFIG_XENO_HW_UNLOCKED_SWITCH
	if (!rootcb->mm)
		set_ti_thread_flag(rootcb->tip, TIF_MMSWITCH_INT);
#endif
	rthal_unmute_pic();
}

struct xnlock;

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

#ifdef __IPIPE_FEATURE_HARDENED_SWITCHMM
	if (next_mm && likely(prev_mm != next_mm))
		wrap_switch_mm(prev_mm, next_mm, next);
#else /* !__IPIPE_FEATURE_HARDENED_SWITCHMM */
	if (likely(prev_mm != next_mm)) {
#ifdef CONFIG_ALTIVEC
		asm volatile ("dssall;\n" :/*empty*/:);
#endif
#ifdef CONFIG_PPC64
		if (likely(next_mm)) {
			cpu_set(rthal_processor_id(), next_mm->cpu_vm_mask);
			if (wrap_mmu_has_slb())
				switch_slb(next, next_mm);
			else
				switch_stab(next, next_mm);
		}
	}
#else /* PPC32 */
		if (likely(next_mm != NULL)) {
			next->thread.pgdir = next_mm->pgd;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29)
			get_mmu_context(next_mm);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18)
			set_context(next_mm->context, next_mm->pgd);
#else /* !(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18))*/
			set_context(next_mm->context.id, next_mm->pgd);
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18) */
#else /* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,29) */
			switch_mmu_context(prev_mm, next_mm);
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,29) */
			current = prev;	/* Make sure r2 is valid. */
		}
	}
#endif	/* PPC32 */
#endif /* !__IPIPE_FEATURE_HARDENED_SWITCHMM */

#ifdef CONFIG_PPC64
	rthal_thread_switch(out_tcb->tsp, in_tcb->tsp, next == NULL);
#else
	rthal_thread_switch(out_tcb->tsp, in_tcb->tsp);
#endif
	barrier();
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
	struct pt_regs *childregs;
	unsigned long sp;

	/*
	 * Stack space is guaranteed to have been fully zeroed. We do
	 * this earlier in xnthread_init() which runs with interrupts
	 * on, to reduce latency.
	 */
	sp = (unsigned long)tcb->stackbase + tcb->stacksize;
	sp -= sizeof(struct pt_regs);
	childregs = (struct pt_regs *)sp;
	sp -= STACK_FRAME_OVERHEAD;

	tcb->ts.ksp = sp;
	tcb->entry = entry;
	tcb->cookie = cookie;
	tcb->self = thread;
	tcb->imask = imask;
	tcb->name = name;

#ifdef CONFIG_PPC64
	childregs->nip = ((unsigned long *)rthal_thread_trampoline)[0];
	childregs->gpr[2] = ((unsigned long *)rthal_thread_trampoline)[1];
	childregs->gpr[22] = (unsigned long)tcb;
	childregs->gpr[23] = ((unsigned long *)xnarch_thread_trampoline)[0];	/* lr = entry addr. */
	childregs->gpr[24] = ((unsigned long *)xnarch_thread_trampoline)[1];	/* r2 = TOC base. */
	if (wrap_mmu_has_slb())
		tcb->ts.ksp_vsid = get_stack_vsid(tcb->ts.ksp);
#else /* !CONFIG_PPC64 */
	childregs->nip = (unsigned long)rthal_thread_trampoline;
	childregs->gpr[22] = (unsigned long)tcb;
	childregs->gpr[23] = (unsigned long)xnarch_thread_trampoline;
#endif	/* !CONFIG_PPC64 */
}

/* No lazy FPU init on PPC. */
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

static void xnarch_save_fpu(xnarchtcb_t * tcb)
{
	if (tcb->fpup) {
		rthal_save_fpu(tcb->fpup);

		if (tcb->user_fpu_owner &&
		    tcb->user_fpu_owner->thread.regs)
			tcb->user_fpu_owner->thread.regs->msr &= ~(MSR_FP|MSR_FE0|MSR_FE1);
	}
}

static void xnarch_restore_fpu(xnarchtcb_t * tcb)
{
	struct thread_struct *ts;
	struct pt_regs *regs;

	if (tcb->fpup) {
		rthal_restore_fpu(tcb->fpup);
		/*
		 * Note: Only enable FP in MSR, if it was enabled when
		 * we saved the fpu state.
		 */
		if (tcb->user_fpu_owner) {
			ts = &tcb->user_fpu_owner->thread;
			regs = ts->regs;
			if (regs) {
				regs->msr &= ~(MSR_FE0|MSR_FE1);
				regs->msr |= (MSR_FP|ts->fpexc_mode);
			}
		}
	}
	/*
	 * FIXME: We restore FPU "as it was" when Xenomai preempted Linux,
	 * whereas we could be much lazier.
	 */
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

#endif /* !_XENO_ASM_POWERPC_BITS_POD_H */
