/*
 * Copyright (C) 2001,2002,2003 Philippe Gerum <rpm@xenomai.org>.
 * Copyright (C) 2005 Dmitry Adamushko <dmitry.adamushko@gmail.com>
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

#ifndef _XENO_NUCLEUS_QUEUE_H
#define _XENO_NUCLEUS_QUEUE_H

#include <nucleus/types.h>
#include <nucleus/assert.h>

/* Basic element holder */

typedef struct xnholder {

	struct xnholder *next;
	struct xnholder *last;

} xnholder_t;

static inline void inith(xnholder_t *holder)
{
	/* Holding queues are doubly-linked and circular */
	holder->last = holder;
	holder->next = holder;
}

static inline void ath(xnholder_t *head, xnholder_t *holder)
{
	/* Inserts the new element right after the heading one  */
	holder->last = head;
	holder->next = head->next;
	holder->next->last = holder;
	head->next = holder;
}

static inline void dth(xnholder_t *holder)
{
	holder->last->next = holder->next;
	holder->next->last = holder->last;
}

/* Basic element queue */

typedef struct xnqueue {

	xnholder_t head;
	int elems;
#if defined(__KERNEL__) && XENO_DEBUG(QUEUES)
	DECLARE_XNLOCK(lock);
#endif /* __KERNEL__ && XENO_DEBUG(QUEUES) */

} xnqueue_t;

#if XENO_DEBUG(QUEUES) && (defined(CONFIG_SMP) || XENO_DEBUG(XNLOCK))
#define XNQUEUE_INITIALIZER(q) { { &(q).head, &(q).head }, 0, XNARCH_LOCK_UNLOCKED }
#else /* !(XENO_DEBUG(QUEUES) */
#define XNQUEUE_INITIALIZER(q) { { &(q).head, &(q).head }, 0 }
#endif /* XENO_DEBUG(QUEUES) */

#define DEFINE_XNQUEUE(q) xnqueue_t q = XNQUEUE_INITIALIZER(q)

static inline void initq(xnqueue_t *qslot)
{
	inith(&qslot->head);
	qslot->elems = 0;
#if defined(__KERNEL__) && XENO_DEBUG(QUEUES)
	xnlock_init(&qslot->lock);
#endif /* __KERNEL__ && XENO_DEBUG(QUEUES) */
}

#if XENO_DEBUG(QUEUES)

#if defined(__KERNEL__) || defined(__XENO_SIM__)

#define XENO_DEBUG_CHECK_QUEUE(__qslot)					\
	do {								\
		xnholder_t *curr;					\
		spl_t s;						\
		int nelems = 0;						\
		xnlock_get_irqsave(&(__qslot)->lock,s);			\
		curr = (__qslot)->head.last;				\
		while (curr != &(__qslot)->head && nelems < (__qslot)->elems) \
			curr = curr->last, nelems++;			\
		if (curr != &(__qslot)->head || nelems != (__qslot)->elems) \
			xnpod_fatal("corrupted queue, qslot->elems=%d/%d, qslot=%p at %s:%d", \
				    nelems,				\
				    (__qslot)->elems,			\
				    __qslot,				\
				    __FILE__,__LINE__);			\
		xnlock_put_irqrestore(&(__qslot)->lock,s);		\
	} while(0)

#define XENO_DEBUG_INSERT_QUEUE(__qslot,__holder)			\
	do {								\
		xnholder_t *curr;					\
		spl_t s;						\
		xnlock_get_irqsave(&(__qslot)->lock,s);			\
		curr = (__qslot)->head.last;				\
		while (curr != &(__qslot)->head && (__holder) != curr)	\
			curr = curr->last;				\
		if (curr == (__holder))					\
			xnpod_fatal("inserting element twice, holder=%p, qslot=%p at %s:%d", \
				    __holder,				\
				    __qslot,				\
				    __FILE__,__LINE__);			\
		if ((__holder)->last == NULL)				\
			xnpod_fatal("holder=%p not initialized, qslot=%p", \
				    __holder,				\
				    __qslot);				\
		xnlock_put_irqrestore(&(__qslot)->lock,s);		\
	} while(0)

#define XENO_DEBUG_REMOVE_QUEUE(__qslot,__holder)			\
	do {								\
		xnholder_t *curr;					\
		spl_t s;						\
		xnlock_get_irqsave(&(__qslot)->lock,s);			\
		curr = (__qslot)->head.last;				\
		while (curr != &(__qslot)->head && (__holder) != curr)	\
			curr = curr->last;				\
		if (curr == &(__qslot)->head)				\
			xnpod_fatal("removing non-linked element, holder=%p, qslot=%p at %s:%d", \
				    __holder,				\
				    __qslot,				\
				    __FILE__,__LINE__);			\
		xnlock_put_irqrestore(&(__qslot)->lock,s);		\
	} while(0)

#else /* !(__KERNEL__ || __XENO_SIM__) */

/* Disable queue checks in user-space code which does not run as part
   of any virtual machine, e.g. skin call interface libs. */

#define XENO_DEBUG_CHECK_QUEUE(__qslot)
#define XENO_DEBUG_INSERT_QUEUE(__qslot,__holder)
#define XENO_DEBUG_REMOVE_QUEUE(__qslot,__holder)

#endif /* __KERNEL__ || __XENO_SIM__ */

/* Write the following as macros so that line numbering information
   keeps pointing at the real caller in diagnosis messages. */

#define insertq(__qslot,__head,__holder)			\
	({ XENO_DEBUG_CHECK_QUEUE(__qslot);			\
		XENO_DEBUG_INSERT_QUEUE(__qslot,__holder);	\
		ath((__head)->last,__holder);			\
		++(__qslot)->elems; })

#define prependq(__qslot,__holder)				\
	({ XENO_DEBUG_CHECK_QUEUE(__qslot);			\
		XENO_DEBUG_INSERT_QUEUE(__qslot,__holder);	\
		ath(&(__qslot)->head,__holder);			\
		++(__qslot)->elems; })

#define appendq(__qslot,__holder)				\
	({ XENO_DEBUG_CHECK_QUEUE(__qslot);			\
		XENO_DEBUG_INSERT_QUEUE(__qslot,__holder);	\
		ath((__qslot)->head.last,__holder);		\
		++(__qslot)->elems; })

#define removeq(__qslot,__holder)				\
	({ XENO_DEBUG_CHECK_QUEUE(__qslot);			\
		XENO_DEBUG_REMOVE_QUEUE(__qslot,__holder);	\
		dth(__holder);					\
		--(__qslot)->elems; })

#else /* !XENO_DEBUG(QUEUES) */

static inline void insertq(xnqueue_t *qslot,
			   xnholder_t *head, xnholder_t *holder)
{
	/* Insert the <holder> element before <head> */
	ath(head->last, holder);
	++qslot->elems;
}

static inline void prependq(xnqueue_t *qslot, xnholder_t *holder)
{
	/* Prepend the element to the queue */
	ath(&qslot->head, holder);
	++qslot->elems;
}

static inline void appendq(xnqueue_t *qslot, xnholder_t *holder)
{
	/* Append the element to the queue */
	ath(qslot->head.last, holder);
	++qslot->elems;
}

static inline void removeq(xnqueue_t *qslot, xnholder_t *holder)
{
	dth(holder);
	--qslot->elems;
}

#endif /* XENO_DEBUG(QUEUES) */

static inline xnholder_t *getheadq(xnqueue_t *qslot)
{
	xnholder_t *holder = qslot->head.next;
	return holder == &qslot->head ? NULL : holder;
}

static inline xnholder_t *getq(xnqueue_t *qslot)
{
	xnholder_t *holder = getheadq(qslot);
	if (holder)
		removeq(qslot, holder);
	return holder;
}

static inline xnholder_t *nextq(xnqueue_t *qslot, xnholder_t *holder)
{
	xnholder_t *nextholder = holder->next;
	return nextholder == &qslot->head ? NULL : nextholder;
}

static inline xnholder_t *popq(xnqueue_t *qslot, xnholder_t *holder)
{
	xnholder_t *nextholder = nextq(qslot, holder);
	removeq(qslot, holder);
	return nextholder;
}

static inline int countq(xnqueue_t *qslot)
{
	return qslot->elems;
}

static inline int emptyq_p(xnqueue_t *qslot)
{
	return qslot->head.next == &qslot->head;
}

static inline void moveq(xnqueue_t *dstq, xnqueue_t *srcq)
{
	xnholder_t *headsrc = srcq->head.next;
	xnholder_t *tailsrc = srcq->head.last;
	xnholder_t *headdst = &dstq->head;

	if (emptyq_p(srcq))
		return;

	/* srcq elements are moved to head of dstq (LIFO) */
	headsrc->last->next = tailsrc->next;
	tailsrc->next->last = headsrc->last;
	headsrc->last = headdst;
	tailsrc->next = headdst->next;
	headdst->next->last = tailsrc;
	headdst->next = headsrc;
	dstq->elems += srcq->elems;
	srcq->elems = 0;
}

/* Prioritized element holder */

typedef struct xnpholder {

	xnholder_t plink;
	int prio;

} xnpholder_t;

static inline void initph(xnpholder_t *holder)
{
	inith(&holder->plink);
	/* Priority is set upon queue insertion */
}

/* Prioritized element queue - we only manage a descending queuing
   order (highest numbered priorities are linked first). */

typedef struct xnpqueue { xnqueue_t pqueue; } xnpqueue_t;

static inline void initpq(xnpqueue_t *pqslot)
{
	initq(&pqslot->pqueue);
}

static inline void insertpq(xnpqueue_t *pqslot,
			    xnpholder_t *head, xnpholder_t *holder)
{
	/* Insert the <holder> element before <head> */
	insertq(&pqslot->pqueue, &head->plink, &holder->plink);
}

static inline void insertpqf(xnpqueue_t *pqslot, xnpholder_t *holder, int prio)
{
	/* Insert the element at the end of its priority group (FIFO) */

	xnholder_t *curr;

	for (curr = pqslot->pqueue.head.last;
	     curr != &pqslot->pqueue.head; curr = curr->last) {
		if (prio <= ((xnpholder_t *)curr)->prio)
			break;
	}

	holder->prio = prio;

	insertq(&pqslot->pqueue, curr->next, &holder->plink);
}

static inline void insertpql(xnpqueue_t *pqslot, xnpholder_t *holder, int prio)
{
	/* Insert the element at the front of its priority group (LIFO) */

	xnholder_t *curr;

	for (curr = pqslot->pqueue.head.next;
	     curr != &pqslot->pqueue.head; curr = curr->next) {
		if (prio >= ((xnpholder_t *)curr)->prio)
			break;
	}

	holder->prio = prio;

	insertq(&pqslot->pqueue, curr, &holder->plink);
}

static inline xnpholder_t *findpqh(xnpqueue_t *pqslot, int prio)
{
	/* Find the element heading a given priority group */

	xnholder_t *curr;

	for (curr = pqslot->pqueue.head.next;
	     curr != &pqslot->pqueue.head; curr = curr->next) {
		if (prio >= ((xnpholder_t *)curr)->prio)
			break;
	}

	if (curr && ((xnpholder_t *)curr)->prio == prio)
		return (xnpholder_t *)curr;

	return NULL;
}

static inline void insertpqfr(xnpqueue_t *pqslot, xnpholder_t *holder, int prio)
{
	/*
	 * Insert the element at the front of its priority group
	 * (FIFO) - Reverse queueing applied (lowest numbered
	 * priorities are put at front).
	 */
	xnholder_t *curr;

	for (curr = pqslot->pqueue.head.last;
	     curr != &pqslot->pqueue.head; curr = curr->last) {
		if (prio >= ((xnpholder_t *)curr)->prio)
			break;
	}

	holder->prio = prio;

	insertq(&pqslot->pqueue, curr->next, &holder->plink);
}

static inline void insertpqlr(xnpqueue_t *pqslot, xnpholder_t *holder, int prio)
{
	/*
	 * Insert the element at the front of its priority group
	 * (LIFO) - Reverse queueing applied (lowest numbered
	 * priorities are put at front).
	 */
	xnholder_t *curr;

	for (curr = pqslot->pqueue.head.next;
	     curr != &pqslot->pqueue.head; curr = curr->next) {
		if (prio <= ((xnpholder_t *)curr)->prio)
			break;
	}

	holder->prio = prio;

	insertq(&pqslot->pqueue, curr, &holder->plink);
}

static inline xnpholder_t *findpqhr(xnpqueue_t *pqslot, int prio)
{
	/*
	 * Find the element heading a given priority group - Reverse
	 * queueing assumed (lowest numbered priorities should be at
	 * front).
	 */
	xnholder_t *curr;

	for (curr = pqslot->pqueue.head.next;
	     curr != &pqslot->pqueue.head; curr = curr->next) {
		if (prio <= ((xnpholder_t *)curr)->prio)
			break;
	}

	if (curr && ((xnpholder_t *)curr)->prio == prio)
		return (xnpholder_t *)curr;

	return NULL;
}

static inline void appendpq(xnpqueue_t *pqslot, xnpholder_t *holder)
{
	holder->prio = 0;
	appendq(&pqslot->pqueue, &holder->plink);
}

static inline void prependpq(xnpqueue_t *pqslot, xnpholder_t *holder)
{
	holder->prio = 0;
	prependq(&pqslot->pqueue, &holder->plink);
}

static inline void removepq(xnpqueue_t *pqslot, xnpholder_t *holder)
{
	removeq(&pqslot->pqueue, &holder->plink);
}

static inline xnpholder_t *getheadpq(xnpqueue_t *pqslot)
{
	return (xnpholder_t *)getheadq(&pqslot->pqueue);
}

static inline xnpholder_t *nextpq(xnpqueue_t *pqslot, xnpholder_t *holder)
{
	return (xnpholder_t *)nextq(&pqslot->pqueue, &holder->plink);
}

static inline xnpholder_t *getpq(xnpqueue_t *pqslot)
{
	return (xnpholder_t *)getq(&pqslot->pqueue);
}

static inline xnpholder_t *poppq(xnpqueue_t *pqslot, xnpholder_t *holder)
{
	return (xnpholder_t *)popq(&pqslot->pqueue, &holder->plink);
}

static inline int countpq(xnpqueue_t *pqslot)
{
	return countq(&pqslot->pqueue);
}

static inline int emptypq_p(xnpqueue_t *pqslot)
{
	return emptyq_p(&pqslot->pqueue);
}

/* Generic prioritized element holder */

typedef struct xngholder {

	xnpholder_t glink;
	void *data;

} xngholder_t;

static inline void initgh(xngholder_t *holder, void *data)
{
	inith(&holder->glink.plink);
	holder->data = data;
}

/* Generic element queue */

typedef struct xngqueue {

	xnpqueue_t gqueue;
	xnqueue_t *freehq;
	void (*starvation) (xnqueue_t *);
	int threshold;

} xngqueue_t;

static inline void initgq(xngqueue_t *gqslot,
			  xnqueue_t *freehq,
			  void (*starvation) (xnqueue_t *),
			  int threshold)
{
	initpq(&gqslot->gqueue);
	gqslot->freehq = freehq;
	gqslot->starvation = starvation;
	gqslot->threshold = threshold;
}

static inline xngholder_t *allocgh(xngqueue_t *gqslot)
{
	if (countq(gqslot->freehq) < gqslot->threshold)
		gqslot->starvation(gqslot->freehq);

	return (xngholder_t *)getq(gqslot->freehq);
}

static inline void *removegh(xngqueue_t *gqslot, xngholder_t *holder)
{
	removepq(&gqslot->gqueue, &holder->glink);
	appendq(gqslot->freehq, &holder->glink.plink);
	return holder->data;
}

static inline void insertgqf(xngqueue_t *gqslot, void *data, int prio)
{
	xngholder_t *holder = allocgh(gqslot);
	holder->data = data;
	insertpqf(&gqslot->gqueue, &holder->glink, prio);
}

static inline void insertgql(xngqueue_t *gqslot, void *data, int prio)
{
	xngholder_t *holder = allocgh(gqslot);
	holder->data = data;
	insertpql(&gqslot->gqueue, &holder->glink, prio);
}

static inline void appendgq(xngqueue_t *gqslot, void *data)
{
	xngholder_t *holder = allocgh(gqslot);
	holder->data = data;
	appendpq(&gqslot->gqueue, &holder->glink);
}

static inline void prependgq(xngqueue_t *gqslot, void *data)
{
	xngholder_t *holder = allocgh(gqslot);
	holder->data = data;
	prependpq(&gqslot->gqueue, &holder->glink);
}

static inline xngholder_t *getheadgq(xngqueue_t *gqslot)
{
	return (xngholder_t *)getheadpq(&gqslot->gqueue);
}

static inline xngholder_t *nextgq(xngqueue_t *gqslot, xngholder_t *holder)
{
	return (xngholder_t *)nextpq(&gqslot->gqueue, &holder->glink);
}

static inline void *getgq(xngqueue_t *gqslot)
{
	xngholder_t *holder = getheadgq(gqslot);

	if (!holder)
		return NULL;

	appendq(gqslot->freehq, &getpq(&gqslot->gqueue)->plink);

	return holder->data;
}

static inline xngholder_t *popgq(xngqueue_t *gqslot, xngholder_t *holder)
{
	xngholder_t *nextholder = nextgq(gqslot, holder);
	removegh(gqslot, holder);
	return nextholder;
}

static inline xngholder_t *findgq(xngqueue_t *gqslot, void *data)
{
	xnholder_t *holder;

	for (holder = gqslot->gqueue.pqueue.head.next;
	     holder != &gqslot->gqueue.pqueue.head; holder = holder->next) {
		if (((xngholder_t *)holder)->data == data)
			return (xngholder_t *)holder;
	}

	return NULL;
}

static inline void *removegq(xngqueue_t *gqslot, void *data)
{
	xngholder_t *holder = findgq(gqslot, data);
	return holder ? removegh(gqslot, holder) : NULL;
}

static inline int countgq(xngqueue_t *gqslot)
{
	return countpq(&gqslot->gqueue);
}

static inline int emptygq_p(xngqueue_t *gqslot)
{
	return emptypq_p(&gqslot->gqueue);
}

#endif /* !_XENO_NUCLEUS_QUEUE_H */
