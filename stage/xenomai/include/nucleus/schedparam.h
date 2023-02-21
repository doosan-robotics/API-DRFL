/*
 * Copyright (C) 2008 Philippe Gerum <rpm@xenomai.org>.
 *
 * Xenomai is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation; either version 2 of the License,
 * or (at your option) any later version.
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

#ifndef _XENO_NUCLEUS_SCHEDPARAM_H
#define _XENO_NUCLEUS_SCHEDPARAM_H

struct xnsched_idle_param {
	int prio;
};

struct xnsched_rt_param {
	int prio;
};

struct xnsched_tp_param {
	int prio;
	int ptid;	/* partition id. */
};

struct xnsched_sporadic_param {
	xntime_t init_budget;
	xntime_t repl_period;
	int max_repl;
	int low_prio;
	int normal_prio;
	int current_prio;
};

union xnsched_policy_param {
	struct xnsched_idle_param idle;
	struct xnsched_rt_param rt;
#ifdef CONFIG_XENO_OPT_SCHED_TP
	struct xnsched_tp_param tp;
#endif
#ifdef CONFIG_XENO_OPT_SCHED_SPORADIC
	struct xnsched_sporadic_param pss;
#endif
};

#endif /* !_XENO_NUCLEUS_SCHEDPARAM_H */
