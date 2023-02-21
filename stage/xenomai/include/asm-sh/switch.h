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

#ifndef _XENO_ASM_SH_SWITCH_H
#define _XENO_ASM_SH_SWITCH_H

#ifndef __KERNEL__
#error "Pure kernel header included from user-space!"
#endif

#include <asm/system.h>

/*
 * Most of this code was lifted from the regular Linux task switching
 * code. A provision for handling Xenomai-originated kernel threads
 * (aka "hybrid scheduling" is added).
 */
#define xnarch_switch_threads(otcb, itcb, prev, next)		\
	({							\
	register u32 *__ts1 __asm__ ("r1");			\
	register u32 *__ts2 __asm__ ("r2");			\
	register u32 *__ts4 __asm__ ("r4");			\
	register u32 *__ts5 __asm__ ("r5");			\
	register u32 *__ts6 __asm__ ("r6");			\
	register u32 __ts7 __asm__ ("r7");			\
	struct task_struct *__last = prev;			\
	struct xnarchtcb *__ltcb = otcb;			\
								\
	if (otcb->tsp == &prev->thread &&			\
	    is_dsp_enabled(prev))				\
		__save_dsp(prev);				\
								\
	__ts1 = (u32 *)&otcb->tsp->sp;				\
	__ts2 = (u32 *)&otcb->tsp->pc;				\
	__ts4 = (u32 *)prev;					\
	__ts5 = (u32 *)next;					\
	__ts6 = (u32 *)&itcb->tsp->sp;				\
	__ts7 = itcb->tsp->pc;					\
								\
	__asm__ __volatile__ (					\
		".balign 4\n\t"					\
		"stc.l	gbr, @-r15\n\t"				\
		"sts.l	pr, @-r15\n\t"				\
		"mov.l	r8, @-r15\n\t"				\
		"mov.l	r9, @-r15\n\t"				\
		"mov.l	r10, @-r15\n\t"				\
		"mov.l	r11, @-r15\n\t"				\
		"mov.l	r12, @-r15\n\t"				\
		"mov.l	r13, @-r15\n\t"				\
		"mov.l	r14, @-r15\n\t"				\
		"mov.l	r15, @r1\t! save SP\n\t"		\
		"mov.l	@r6, r15\t! change to new stack\n\t"	\
		"mova	1f, %0\n\t"				\
		"mov.l	%0, @r2\t! save PC\n\t"			\
		"mov	#0, r8\n\t"				\
		"cmp/eq r5, r8\n\t"				\
		"bt/s   3f\n\t"					\
		" lds	r7, pr\t!  return to saved PC\n\t"	\
		"mov.l	2f, %0\n\t"				\
		"jmp	@%0\t! call __switch_to\n\t"		\
		" nop\t\n\t"					\
		"3:\n\t"					\
		"rts\n\t"					\
		".balign	4\n"				\
		"2:\n\t"					\
		".long	__switch_to\n"				\
		"1:\n\t"					\
		"mov.l	@r15+, r14\n\t"				\
		"mov.l	@r15+, r13\n\t"				\
		"mov.l	@r15+, r12\n\t"				\
		"mov.l	@r15+, r11\n\t"				\
		"mov.l	@r15+, r10\n\t"				\
		"mov.l	@r15+, r9\n\t"				\
		"mov.l	@r15+, r8\n\t"				\
		"lds.l	@r15+, pr\n\t"				\
		"ldc.l	@r15+, gbr\n\t"				\
		: "=z" (__last)					\
		: "r" (__ts1), "r" (__ts2), "r" (__ts4),	\
		  "r" (__ts5), "r" (__ts6), "r" (__ts7)		\
		: "r3", "t");					\
								\
	if (__ltcb->tsp == &__last->thread &&			\
	    is_dsp_enabled(__last))				\
		__restore_dsp(__last);				\
								\
	__last;							\
	})

#endif /* _XENO_ASM_SH_SWITCH_H */
