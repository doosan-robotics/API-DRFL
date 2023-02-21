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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Xenomai; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
 * 02111-1307, USA.
 */

#ifndef _XENO_NUCLEUS_SCHEDQUEUE_H
#define _XENO_NUCLEUS_SCHEDQUEUE_H

#include <nucleus/queue.h>

#ifdef CONFIG_XENO_OPT_SCALABLE_SCHED
/*
 * Multi-level priority queue, suitable for handling the runnable
 * thread queue of a scheduling class with O(1) property. We only
 * manage a descending queuing order, i.e. highest numbered priorities
 * come first.
 */
#define XNSCHED_MLQ_LEVELS  264

#if BITS_PER_LONG * BITS_PER_LONG < XNSCHED_MLQ_LEVELS
#error "Internal bitmap cannot hold so many priority levels"
#endif

#define __MLQ_LONGS ((XNSCHED_MLQ_LEVELS+BITS_PER_LONG-1)/BITS_PER_LONG)

struct xnsched_mlq {

	int loprio, hiprio, elems;
	unsigned long himap, lomap[__MLQ_LONGS];
	struct xnqueue queue[XNSCHED_MLQ_LEVELS];

};

#undef __MLQ_LONGS

void initmlq(struct xnsched_mlq *q, int loprio, int hiprio);

void addmlq(struct xnsched_mlq *q,
	    struct xnpholder *holder, int idx, int lifo);

void removemlq(struct xnsched_mlq *q, struct xnpholder *holder);

struct xnpholder *findmlqh(struct xnsched_mlq *q, int prio);

struct xnpholder *getheadmlq(struct xnsched_mlq *q);

struct xnpholder *getmlq(struct xnsched_mlq *q);

struct xnpholder *nextmlq(struct xnsched_mlq *q,
			  struct xnpholder *h);

static inline int countmlq(struct xnsched_mlq *q)
{
	return q->elems;
}

static inline int emptymlq_p(struct xnsched_mlq *q)
{
	return q->himap == 0;
}

static inline int indexmlq(struct xnsched_mlq *q, int prio)
{
	XENO_ASSERT(QUEUES,
		    prio >= q->loprio && prio <= q->hiprio,
		    xnpod_fatal("priority level %d is out of range ", prio));
	/*
	 * BIG FAT WARNING: We need to rescale the priority level to a
	 * 0-based range. We use ffnz() to scan the bitmap which MUST
	 * be based on a bit scan forward op. Therefore, the lower the
	 * index value, the higher the priority (since least
	 * significant bits will be found first when scanning the
	 * bitmaps).
	 */
	return q->hiprio - prio;
}

static inline int ffsmlq(struct xnsched_mlq *q)
{
	int hi = ffnz(q->himap);
	int lo = ffnz(q->lomap[hi]);
	return hi * BITS_PER_LONG + lo;	/* Result is undefined if none set. */
}

static inline void insertmlql(struct xnsched_mlq *q,
			      struct xnpholder *holder, int prio)
{
	addmlq(q, holder, indexmlq(q, prio), 1);
}

static inline void insertmlqf(struct xnsched_mlq *q,
			      struct xnpholder *holder, int prio)
{
	addmlq(q, holder, indexmlq(q, prio), 0);
}

static inline void appendmlq(struct xnsched_mlq *q, struct xnpholder *holder)
{
	addmlq(q, holder, indexmlq(q, q->hiprio), 0);
}

static inline void prependmlq(struct xnsched_mlq *q, struct xnpholder *holder)
{
	addmlq(q, holder, indexmlq(q, q->loprio), 1);
}

typedef struct xnsched_mlq xnsched_queue_t;

#define sched_initpq		initmlq
#define sched_emptypq_p		emptymlq_p
#define sched_insertpql		insertmlql
#define sched_insertpqf		insertmlqf
#define sched_appendpq		appendmlq
#define sched_prependpq		prependmlq
#define sched_removepq		removemlq
#define sched_getheadpq		getheadmlq
#define sched_nextpq		nextmlq
#define sched_getpq		getmlq
#define sched_findpqh		findmlqh

#else /* ! CONFIG_XENO_OPT_SCALABLE_SCHED */

typedef xnpqueue_t xnsched_queue_t;

#define sched_initpq(q, minp, maxp)	initpq(q)
#define sched_emptypq_p			emptypq_p
#define sched_insertpql			insertpql
#define sched_insertpqf			insertpqf
#define sched_appendpq			appendpq
#define sched_prependpq			prependpq
#define sched_removepq			removepq
#define sched_getheadpq			getheadpq
#define sched_nextpq			nextpq
#define sched_getpq			getpq
#define sched_findpqh			findpqh

#endif /* !CONFIG_XENO_OPT_SCALABLE_SCHED */

#endif /* !_XENO_NUCLEUS_SCHEDQUEUE_H */
