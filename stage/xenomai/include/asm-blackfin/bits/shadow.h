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

#ifndef _XENO_ASM_BLACKFIN_BITS_SHADOW_H
#define _XENO_ASM_BLACKFIN_BITS_SHADOW_H

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
#ifdef CONFIG_MPU
	tcb->active_task = NULL;
#endif
	tcb->tsp = &task->thread;
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
	 * 45 e1 0c 00 	  R5.H = 0xc
	 * 05 e1 2b 02 	  R5.L = 0x22b
	 * 05 32       	  P0 = R5
	 * a0 00       	  EXCPT 0x0
	 * cd ef          <bug opcode>
	 *
	 * We don't mess with ASTAT here, so no need to save/restore
	 * it in handle/fixup code.
	 */
	static const struct __attribute__ ((__packed__)) {
		struct __attribute__ ((__packed__)) {
			u16 op;
			u16 imm;
		} load_r5h;
		struct __attribute__ ((__packed__)) {
			u16 op;
			u16 imm;
		} load_r5l;
		u16 mov_p0;
		u16 syscall;
		u16 bug;
	} code = {
		.load_r5h = {
			.op = 0xe145,
			.imm = __xn_mux_code(0, __xn_sys_mayday) >> 16
		},
		.load_r5l = {
			.op = 0xe105,
			.imm = __xn_mux_code(0, __xn_sys_mayday) & 0xffff
		},
		.mov_p0 = 0x3205,
		.syscall = 0x00a0,
		.bug = BFIN_BUG_OPCODE,
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
	tcb->mayday.pc = regs->pc;
	tcb->mayday.p0 = regs->p0;
	tcb->mayday.r5 = regs->r5;
	regs->pc = tramp;	/* i.e. RETI */
}

static inline void xnarch_fixup_mayday(struct xnarchtcb *tcb,
				       struct pt_regs *regs)
{
	regs->pc = tcb->mayday.pc;
	regs->p0 = tcb->mayday.p0;
	regs->r5 = tcb->mayday.r5;
}

#endif /* XNARCH_HAVE_MAYDAY */

#endif /* !_XENO_ASM_BLACKFIN_BITS_SHADOW_H */
