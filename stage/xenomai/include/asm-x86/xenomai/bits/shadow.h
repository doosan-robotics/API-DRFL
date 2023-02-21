/*
 * Copyright (C) 2001-2007 Philippe Gerum <rpm@xenomai.org>.
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

#ifndef _XENO_ASM_X86_BITS_SHADOW_H
#define _XENO_ASM_X86_BITS_SHADOW_H
#define _XENO_ASM_X86_BITS_SHADOW_H

#ifndef __KERNEL__
#error "Pure kernel header included from user-space!"
#endif

static inline void xnarch_init_shadow_tcb(xnarchtcb_t * tcb,
					  struct xnthread *thread,
					  const char *name)
{
	struct task_struct *task = current;

	tcb->user_task = task;
	tcb->active_task = NULL;
	tcb->spp = &task->thread.x86reg_sp;
#ifdef CONFIG_X86_64
	tcb->ipp = &task->thread.rip; /* <!> raw naming intended. */
#else
	tcb->ipp = &task->thread.x86reg_ip;
#endif
	tcb->fpup = x86_fpustate_ptr(&task->thread);
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

static void xnarch_schedule_tail(struct task_struct *prev)
{
#ifdef CONFIG_X86_32
	wrap_switch_iobitmap(prev, rthal_processor_id());
#endif
}

#ifdef XNARCH_HAVE_MAYDAY

static inline void xnarch_setup_mayday_page(void *page)
{
#ifdef CONFIG_X86_64
	/*
	 * We want this code to appear at the top of the MAYDAY page:
	 *
	 * 	b8 2b 02 00 0c	     	mov    $<mux_code>,%eax
	 * 	0f 05		     	syscall
	 * 	0f 0b		     	ud2a
	 *
	 * We intentionally don't mess with EFLAGS here, so that we
	 * don't have to save/restore it in handle/fixup code.
	 */
	static const struct __attribute__ ((__packed__)) {
		struct __attribute__ ((__packed__)) {
			u8 op;
			u32 imm;
		} mov_eax;
		u16 syscall;
		u16 bug;
	} code = {
		.mov_eax = {
			.op = 0xb8,
			.imm = __xn_mux_code(0, __xn_sys_mayday)
		},
		.syscall = 0x050f,
		.bug = 0x0b0f,
	};

	memcpy(page, &code, sizeof(code));

#else /* CONFIG_X86_32 */
	/*
	 * We want this code to appear at the top of the MAYDAY page:
	 *
	 * 	b8 2b 02 00 0c	     	mov    $<mux_code>,%eax
	 * if HAVE_SEP
	 *      65 ff 15 10 00 00 00 	call   *%gs:0x10
	 * else
	 *      cd 80		     	int    $0x80
	 * endif
	 * 	0f 0b		     	ud2a
	 *
	 * We intentionally don't mess with EFLAGS here, so that we
	 * don't have to save/restore it in handle/fixup code.
	 *
	 * Also note that if SEP is present, we always assume NPTL on
	 * the user side.
	 */
	static const struct __attribute__ ((__packed__)) {
		struct __attribute__ ((__packed__)) {
			u8 op;
			u32 imm;
		} mov_eax;
		struct __attribute__ ((__packed__)) {
			u8 op[3];
			u32 moffs;
		} syscall;
		u16 bug;
	} code_sep = {
		.mov_eax = {
			.op = 0xb8,
			.imm = __xn_mux_code(0, __xn_sys_mayday)
		},
		.syscall = {
			.op = {
				0x65, 0xff, 0x15
			},
			.moffs = 0x10
		},
		.bug = 0x0b0f,
	};

	static const struct __attribute__ ((__packed__)) {
		struct __attribute__ ((__packed__)) {
			u8 op;
			u32 imm;
		} mov_eax;
		u16 syscall;
		u16 bug;
	} code_nosep = {
		.mov_eax = {
			.op = 0xb8,
			.imm = __xn_mux_code(0, __xn_sys_mayday)
		},
		.syscall = 0x80cd,
		.bug = 0x0b0f,
	};

	if (cpu_has_sep)
		memcpy(page, &code_sep, sizeof(code_sep));
	else
		memcpy(page, &code_nosep, sizeof(code_nosep));

#endif /* CONFIG_X86_32 */

	/* no cache flush required. */
}

static inline void xnarch_call_mayday(struct task_struct *p)
{
	rthal_return_intercept(p);
}

static inline void xnarch_handle_mayday(struct xnarchtcb *tcb,
					struct pt_regs *regs,
					unsigned long tramp)
{
	tcb->mayday.sp = regs->x86reg_sp;
	tcb->mayday.ip = regs->x86reg_ip;
	tcb->mayday.ax = regs->x86reg_ax;
	regs->x86reg_ip = tramp;
}

static inline void xnarch_fixup_mayday(struct xnarchtcb *tcb,
				       struct pt_regs *regs)
{
	regs->x86reg_ip = tcb->mayday.ip;
	regs->x86reg_ax = tcb->mayday.ax;
	regs->x86reg_sp = tcb->mayday.sp;
}

#endif /* XNARCH_HAVE_MAYDAY */

#endif /* !_XENO_ASM_X86_BITS_SHADOW_H */
