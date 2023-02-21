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

#ifndef _XENO_ASM_SH_SYSTEM_H
#define _XENO_ASM_SH_SYSTEM_H

#ifdef __KERNEL__

#include <linux/ptrace.h>
#include <asm-generic/xenomai/system.h>
#include <asm/system.h>
#include <asm/processor.h>

#define XNARCH_THREAD_STACKSZ   4096

#define xnarch_stack_size(tcb)  ((tcb)->stacksize)
#define xnarch_stack_base(tcb)	((tcb)->stackbase)
#define xnarch_stack_end(tcb)	((caddr_t)(tcb)->stackbase - (tcb)->stacksize)
#define xnarch_user_task(tcb)   ((tcb)->user_task)
#define xnarch_user_pid(tcb)    ((tcb)->user_task->pid)

struct xnthread;
struct task_struct;

typedef struct xnarchtcb {	/* Per-thread arch-dependent block */

	/* Shadowed user-space task */
	struct task_struct *user_task;
	/* Active user-space task */
	struct task_struct *active_task;
	/* Active thread struct (&ts or &user->thread). */
	struct thread_struct *tsp;
	struct mm_struct *mm;
	struct mm_struct *active_mm;
#ifdef XNARCH_HAVE_MAYDAY
	struct {
		unsigned long pc;
		unsigned long r3;
	} mayday;
#endif
	/* Thread context placeholder for kernel threads. */
	struct thread_struct ts;
#ifdef CONFIG_XENO_HW_FPU
	/* Pointer to the FPU backup container */
	struct thread_struct *fpup;
	struct task_struct *user_fpu_owner;
	/* Pointer to the FPU owner in userspace. */
#define xnarch_fpu_ptr(tcb)     ((tcb)->fpup)
#else				/* !CONFIG_XENO_HW_FPU */
#define xnarch_fpu_ptr(tcb)     NULL
#endif				/* CONFIG_XENO_HW_FPU */

	unsigned int stacksize;
	unsigned long *stackbase;

	struct xnthread *self;
	int imask;
	const char *name;
	void (*entry)(void *cookie);
	void *cookie;

} xnarchtcb_t;

typedef struct xnarch_fltinfo {

    unsigned int exception;
    struct pt_regs *regs;

} xnarch_fltinfo_t;

#define xnarch_fault_trap(fi)   ((fi)->exception)
#define xnarch_fault_code(fi)   (0)
#define xnarch_fault_pc(fi)     ((fi)->regs->pc)
#define xnarch_fault_fpu_p(fi)  ((fi)->exception == IPIPE_TRAP_FPUERR)
#define xnarch_fault_pf_p(fi)   ((fi)->exception == IPIPE_TRAP_PF)
#define xnarch_fault_bp_p(fi)   ((current->ptrace & PT_PTRACED) && \
				 (fi)->exception == IPIPE_TRAP_BP)
#define xnarch_fault_notify(fi) (!xnarch_fault_bp_p(fi))

#ifdef __cplusplus
extern "C" {
#endif

static inline void *xnarch_alloc_host_mem(u_long bytes)
{
	if (bytes > 128 * 1024)
		return vmalloc(bytes);

	return kmalloc(bytes, GFP_KERNEL);
}

static inline void xnarch_free_host_mem(void *chunk, u_long bytes)
{
	if (bytes > 128 * 1024)
		vfree(chunk);
	else
		kfree(chunk);
}

static inline void *xnarch_alloc_stack_mem(u_long bytes)
{
	return kmalloc(bytes, GFP_KERNEL);
}

static inline void xnarch_free_stack_mem(void *chunk, u_long bytes)
{
	kfree(chunk);
}

#ifdef __cplusplus
}
#endif

#else /* !__KERNEL__ */

#include <nucleus/system.h>
#include <bits/local_lim.h>

#endif /* __KERNEL__ */

#endif /* !_XENO_ASM_SH_SYSTEM_H */
