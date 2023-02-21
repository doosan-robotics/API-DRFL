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

#ifndef _XENO_ASM_SH_BITS_SCHED_H
#define _XENO_ASM_SH_BITS_SCHED_H

static inline void xnarch_init_root_tcb(xnarchtcb_t *tcb,
					struct xnthread *thread,
					const char *name)
{
	tcb->user_task = current;
	tcb->active_task = NULL;
	tcb->tsp = &tcb->ts;
	tcb->mm = current->mm;
	tcb->active_mm = NULL;
#ifdef CONFIG_XENO_HW_FPU
	tcb->user_fpu_owner = NULL;
	tcb->fpup = NULL;
#endif /* CONFIG_XENO_HW_FPU */
	tcb->entry = NULL;
	tcb->cookie = NULL;
	tcb->self = thread;
	tcb->imask = 0;
	tcb->name = name;
}

#endif /* !_XENO_ASM_SH_BITS_SCHED_H */
