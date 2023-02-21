/*!\file sched-tp.h
 * \brief Definitions for the TP scheduling class.
 * \author Philippe Gerum
 *
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

#ifndef _XENO_NUCLEUS_SCHED_TP_H
#define _XENO_NUCLEUS_SCHED_TP_H

#ifndef _XENO_NUCLEUS_SCHED_H
#error "please don't include nucleus/sched-tp.h directly"
#endif

#ifdef CONFIG_XENO_OPT_SCHED_TP

extern struct xnsched_class xnsched_class_tp;

struct xnsched_tp_window {
	xnticks_t w_offset;
	int w_part;
};

struct xnsched_tp_schedule {
	int pwin_nr;
	xnticks_t tf_duration;
	struct xnsched_tp_window pwins[0];
};

struct xnsched_tp {

	struct xnsched_tpslot {
		xnsched_queue_t runnable; /*!< Runnable thread queue. */
	} partitions[CONFIG_XENO_OPT_SCHED_TP_NRPART];

	struct xnsched_tpslot idle;	/* !< Idle slot for passive windows. */
	struct xnsched_tpslot *tps;	/* !< Active partition slot */
	struct xntimer tf_timer;	/* !< Time frame timer */
	struct xnsched_tp_schedule *gps; /* !< Global partition schedule */
	int wnext;			 /* !< Next partition window */
	xnticks_t tf_start;		 /* !< Start of next time frame */
	struct xnqueue threads;		 /* !< Assigned thread queue */
};

static inline int xnsched_tp_init_tcb(struct xnthread *thread)
{
	inith(&thread->tp_link);
	thread->tps = NULL;

	return 0;
}

struct xnsched_tp_schedule *
xnsched_tp_set_schedule(struct xnsched *sched,
			struct xnsched_tp_schedule *gps);

void xnsched_tp_start_schedule(struct xnsched *sched);

void xnsched_tp_stop_schedule(struct xnsched *sched);

int xnsched_tp_get_partition(struct xnsched *sched);

#endif /* !CONFIG_XENO_OPT_SCHED_TP */

#endif /* !_XENO_NUCLEUS_SCHED_TP_H */
