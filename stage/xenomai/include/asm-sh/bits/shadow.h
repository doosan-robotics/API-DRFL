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

#ifndef _XENO_ASM_SH_BITS_SHADOW_H
#define _XENO_ASM_SH_BITS_SHADOW_H

#ifndef __KERNEL__
#error "Pure kernel header included from user-space!"
#endif

#include <asm/cacheflush.h>

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
	u16 insn[11];

	/*
	 * We want this code to appear at the top of the MAYDAY page:
	 *
	 * 0:	03 d3       	mov.l	12 <pc+0x12>,r3	! b022b
	 * 2:	09 00       	nop	
	 * 4:	10 c3       	trapa	#16
	 * 6:	0b 20       	or	r0,r0
	 * 8:	0b 20       	or	r0,r0
	 * a:	0b 20       	or	r0,r0
	 * c:	0b 20       	or	r0,r0
	 * e:	0b 20       	or	r0,r0
	 * 10:	3e c3       	trapa	#62
	 * 12:	2b 02       	.word 0x022b
	 * 14:	0b 00       	.word 0x000b
	 */
	insn[0] = 0xd303;
	insn[1] = 0x0009;
	insn[2] = 0xc310;
	insn[3] = 0x200b;
	insn[4] = 0x200b;
	insn[5] = 0x200b;
	insn[6] = 0x200b;
	insn[7] = 0x200b;
	insn[8] = 0xc33e;
	insn[9] = 0x022b;
	insn[10] = 0x000b;
	memcpy(page, insn, sizeof(insn));

	flush_dcache_page(vmalloc_to_page(page));
}

static inline void xnarch_call_mayday(struct task_struct *p)
{
	rthal_return_intercept(p);
}

static inline void xnarch_handle_mayday(struct xnarchtcb *tcb,
					struct pt_regs *regs,
					unsigned long tramp)
{
	tcb->mayday.pc = regs->pc;
	tcb->mayday.r3 = regs->regs[3];
	regs->pc = tramp;
}

static inline void xnarch_fixup_mayday(struct xnarchtcb *tcb,
				       struct pt_regs *regs)
{
	regs->pc = tcb->mayday.pc;
	regs->regs[3] = tcb->mayday.r3;
}

#endif /* XNARCH_HAVE_MAYDAY */

#endif /* !_XENO_ASM_SH_BITS_SHADOW_H */
