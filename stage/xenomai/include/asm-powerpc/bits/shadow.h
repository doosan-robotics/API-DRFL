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

#ifndef _XENO_ASM_POWERPC_BITS_SHADOW_H
#define _XENO_ASM_POWERPC_BITS_SHADOW_H

#ifndef __KERNEL__
#error "Pure kernel header included from user-space!"
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
#include <asm/cache.h>
#else
#include <asm/cacheflush.h>
#endif

static inline void xnarch_init_shadow_tcb(xnarchtcb_t * tcb,
					  struct xnthread *thread,
					  const char *name)
{
	struct task_struct *task = current;

	tcb->user_task = task;
	tcb->active_task = NULL;
	tcb->tsp = &task->thread;
	tcb->mm = task->mm;
	tcb->active_mm = NULL;
#ifdef CONFIG_XENO_HW_UNLOCKED_SWITCH
	tcb->tip = task_thread_info(task);
#endif
#ifdef CONFIG_XENO_HW_FPU
	tcb->user_fpu_owner = task;
	tcb->fpup = &task->thread;
#endif /* CONFIG_XENO_HW_FPU */
	tcb->entry = NULL;
	tcb->cookie = NULL;
	tcb->self = thread;
	tcb->imask = 0;
	tcb->name = name;
}

static inline int xnarch_local_syscall(struct pt_regs *regs)
{
	return -ENOSYS;
}

#define xnarch_schedule_tail(prev) do { } while(0)

#ifdef XNARCH_HAVE_MAYDAY

static inline void xnarch_setup_mayday_page(void *page)
{
	/*
	 * We want this code to appear at the top of the MAYDAY page:
	 *
	 * 3c 00 0b 00 	lis     r0,mux_code@h
	 * 60 00 02 2b 	ori     r0,r0,mux_code@l
	 * 44 00 00 02 	sc
	 * 00 b0 0b 00  .long	0x00b00b00 <illegal insn>
	 *
	 * We don't mess with CCR here, so no need to save/restore it
	 * in handle/fixup code.
	 */
	u32 mux, insn[4];

	mux = __xn_mux_code(0, __xn_sys_mayday);
	insn[0] = 0x3c000000 | (mux >> 16);
	insn[1] = 0x60000000 | (mux & 0xffff);
	insn[2] = 0x44000002;
	insn[3] = 0x00b00b00;
	memcpy(page, insn, sizeof(insn));

	flush_dcache_range((unsigned long)page,
			   (unsigned long)page + sizeof(insn));
}

static inline void xnarch_call_mayday(struct task_struct *p)
{
	rthal_return_intercept(p);
}

static inline void xnarch_handle_mayday(struct xnarchtcb *tcb,
					struct pt_regs *regs,
					unsigned long tramp)
{
	tcb->mayday.nip = regs->nip;
	tcb->mayday.r0 = regs->gpr[0];
	regs->nip = tramp;
}

static inline void xnarch_fixup_mayday(struct xnarchtcb *tcb,
				       struct pt_regs *regs)
{
	regs->nip = tcb->mayday.nip;
	regs->gpr[0] = tcb->mayday.r0;
}

#endif /* XNARCH_HAVE_MAYDAY */

#endif /* !_XENO_ASM_POWERPC_BITS_SHADOW_H */
