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

#ifndef _XENO_ASM_NIOS2_BITS_SHADOW_H
#define _XENO_ASM_NIOS2_BITS_SHADOW_H

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
#ifdef CONFIG_XENO_HW_UNLOCKED_SWITCH
	tcb->tip = task_thread_info(task);
#endif
	tcb->entry = NULL;
	tcb->cookie = NULL;
	tcb->self = thread;
	tcb->imask = 0;
	tcb->name = name;
}

static inline int xnarch_local_syscall(struct pt_regs *regs)
{
	unsigned long ptr, x, r;

	switch (__xn_reg_arg1(regs)) {
	case __xn_lsys_xchg:

		/* lsys_xchg(ptr,newval,&oldval) */
		ptr = __xn_reg_arg2(regs);
		x = __xn_reg_arg3(regs);
		r = xchg((unsigned long *)ptr, x);
		__xn_put_user(r, (unsigned long *)__xn_reg_arg4(regs));
		break;

	default:

		return -ENOSYS;
	}

	return 0;
}

#define xnarch_schedule_tail(prev) do { } while(0)

#ifdef XNARCH_HAVE_MAYDAY

static inline void xnarch_setup_mayday_page(void *page)
{
	/*
	 * We want this code to appear at the top of the MAYDAY page:
	 *
	 *	00c00334 	movhi	r3,#__xn_sys_mayday
	 *	18c08ac4 	addi	r3,r3,#__xn_sys_mux
	 *	00800004 	movi	r2,0
	 *	003b683a 	trap
	 *	003fff06 	br	.
	 */
	static const struct {
		u32 movhi_r3h;
		u32 addi_r3l;
		u32 movi_r2;
		u32 syscall;
		u32 bug;
	} code = {
		.movhi_r3h = 0x00c00334,
		.addi_r3l = 0x18c08ac4,
		.movi_r2 = 0x00800004,
		.syscall = 0x003b683a,
		.bug = 0x003fff06
	};

	memcpy(page, &code, sizeof(code));

	flush_dcache_range((unsigned long)page,
			   (unsigned long)page + sizeof(code));
}

static inline void xnarch_call_mayday(struct task_struct *p)
{
	rthal_return_intercept(p);
}

static inline void xnarch_handle_mayday(struct xnarchtcb *tcb,
					struct pt_regs *regs,
					unsigned long tramp)
{
	tcb->mayday.ea = regs->ea;
	tcb->mayday.r2 = regs->r2;
	tcb->mayday.r3 = regs->r3;
	regs->ea = tramp;
}

static inline void xnarch_fixup_mayday(struct xnarchtcb *tcb,
				       struct pt_regs *regs)
{
	regs->ea = tcb->mayday.ea;
	regs->r2 = tcb->mayday.r2;
	regs->r3 = tcb->mayday.r3;
}

#endif /* XNARCH_HAVE_MAYDAY */

#endif /* !_XENO_ASM_NIOS2_BITS_SHADOW_H */
