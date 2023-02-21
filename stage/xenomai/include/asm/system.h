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

#ifndef _XENO_ASM_X86_SYSTEM_H
#define _XENO_ASM_X86_SYSTEM_H
#define _XENO_ASM_X86_SYSTEM_H

#ifdef __KERNEL__

#include <linux/types.h>
#include <linux/ptrace.h>
#include <asm-generic/xenomai/system.h>

struct xnthread;
struct task_struct;

#ifdef CONFIG_X86_64
#define XNARCH_THREAD_STACKSZ	8192
#else
#define XNARCH_THREAD_STACKSZ   4096
#endif

typedef struct xnarchtcb {      /* Per-thread arch-dependent block */
	x86_fpustate i387 __attribute__ ((aligned (16)));
	x86_fpustate *fpup;
	unsigned long sp;
	unsigned long *spp;
	unsigned long ip;
	unsigned long *ipp;
	unsigned int stacksize;
	unsigned long *stackbase;
#ifdef CONFIG_CC_STACKPROTECTOR
	unsigned long canary;
#endif
	struct {
		unsigned long ip;
		unsigned long ax;
		unsigned long sp;
	} mayday;
	unsigned is_root: 1;
	unsigned ts_usedfpu: 1;
	unsigned cr0_ts: 1;
	struct task_struct *user_task;
	struct task_struct *active_task;
	struct xnthread *self;
	int imask;
	const char *name;
	void (*entry)(void *cookie);
	void *cookie;
} xnarchtcb_t;

typedef struct xnarch_fltinfo {
	unsigned vector;
	long errcode;
	struct pt_regs *regs;
} xnarch_fltinfo_t;

#define xnarch_stack_size(tcb)  ((tcb)->stacksize)
#define xnarch_stack_base(tcb)	((tcb)->stackbase)
#define xnarch_stack_end(tcb)	((caddr_t)(tcb)->stackbase - (tcb)->stacksize)
#define xnarch_fpu_ptr(tcb)     ((tcb)->fpup)
#define xnarch_user_task(tcb)   ((tcb)->user_task)
#define xnarch_user_pid(tcb)    ((tcb)->user_task->pid)
#define xnarch_fault_trap(fi)   ((fi)->vector)
#define xnarch_fault_code(fi)   ((fi)->errcode)
#define xnarch_fault_pc(fi)     ((fi)->regs->x86reg_ip)
#define xnarch_fault_fpu_p(fi)  ((fi)->vector == 7)
#define xnarch_fault_pf_p(fi)   ((fi)->vector == 14)
#define xnarch_fault_bp_p(fi)   ((current->ptrace & PT_PTRACED) &&	\
				 ((fi)->vector == 1 || (fi)->vector == 3))
#define xnarch_fault_notify(fi) (!xnarch_fault_bp_p(fi))

static inline void *xnarch_alloc_host_mem(unsigned long bytes)
{
	if (bytes > 128*1024)
		return vmalloc(bytes);

	return kmalloc(bytes,GFP_KERNEL);
}

static inline void xnarch_free_host_mem(void *chunk, unsigned long bytes)
{
	if (bytes > 128*1024)
		vfree(chunk);
	else
		kfree(chunk);
}

static inline void *xnarch_alloc_stack_mem(unsigned long bytes)
{
	return kmalloc(bytes, GFP_KERNEL);
}

static inline void xnarch_free_stack_mem(void *chunk, unsigned long bytes)
{
	kfree(chunk);
}

static inline int xnarch_shadow_p(xnarchtcb_t *tcb, struct task_struct *task)
{
	return tcb->spp == &task->thread.x86reg_sp;
}

#else /* !__KERNEL__ */

#include <nucleus/system.h>
#include <bits/local_lim.h>

#endif /* __KERNEL__ */

#endif /* !_XENO_ASM_X86_SYSTEM_H */
