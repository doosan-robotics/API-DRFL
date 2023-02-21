/*
 * @note Copyright (C) 2001,2002,2003 Philippe Gerum <rpm@xenomai.org>.
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
 *
 * \ingroup synch
 */

#ifndef _XENO_NUCLEUS_SYNCH_H
#define _XENO_NUCLEUS_SYNCH_H

#include <nucleus/queue.h>

/* Creation flags */
#define XNSYNCH_FIFO    0x0
#define XNSYNCH_PRIO    0x1
#define XNSYNCH_NOPIP   0x0
#define XNSYNCH_PIP     0x2
#define XNSYNCH_DREORD  0x4
#define XNSYNCH_OWNER   0x8

#ifndef CONFIG_XENO_OPT_DEBUG_SYNCH_RELAX
#define CONFIG_XENO_OPT_DEBUG_SYNCH_RELAX 0
#endif /* CONFIG_XENO_OPT_DEBUG_SYNCH_RELAX */

#ifdef CONFIG_XENO_FASTSYNCH

/* Fast lock API */
static inline int xnsynch_fast_owner_check(xnarch_atomic_t *fastlock,
					   xnhandle_t ownerh)
{
	return (xnhandle_mask_spare(xnarch_atomic_get(fastlock)) == ownerh) ?
		0 : -EPERM;
}

static inline int xnsynch_fast_acquire(xnarch_atomic_t *fastlock,
				       xnhandle_t new_ownerh)
{
	xnhandle_t lock_state =
	    xnarch_atomic_cmpxchg(fastlock, XN_NO_HANDLE, new_ownerh);

	if (likely(lock_state == XN_NO_HANDLE))
		return 0;

	if (xnhandle_mask_spare(lock_state) == new_ownerh)
		return -EBUSY;

	return -EAGAIN;
}

static inline int xnsynch_fast_release(xnarch_atomic_t *fastlock,
				       xnhandle_t cur_ownerh)
{
	return (xnarch_atomic_cmpxchg(fastlock, cur_ownerh, XN_NO_HANDLE) ==
		cur_ownerh);
}

#else /* !CONFIG_XENO_FASTSYNCH */

static inline int xnsynch_fast_acquire(xnarch_atomic_t *fastlock,
				       xnhandle_t new_ownerh)
{
	return -ENOSYS;
}

static inline int xnsynch_fast_release(xnarch_atomic_t *fastlock,
				       xnhandle_t cur_ownerh)
{
	return -1;
}

#endif	/* !CONFIG_XENO_FASTSYNCH */

#if defined(__KERNEL__) || defined(__XENO_SIM__)

#define XNSYNCH_CLAIMED 0x10	/* Claimed by other thread(s) w/ PIP */

#define XNSYNCH_FLCLAIM XN_HANDLE_SPARE3 /* Corresponding bit in fast lock */

/* Spare flags usable by upper interfaces */
#define XNSYNCH_SPARE0  0x01000000
#define XNSYNCH_SPARE1  0x02000000
#define XNSYNCH_SPARE2  0x04000000
#define XNSYNCH_SPARE3  0x08000000
#define XNSYNCH_SPARE4  0x10000000
#define XNSYNCH_SPARE5  0x20000000
#define XNSYNCH_SPARE6  0x40000000
#define XNSYNCH_SPARE7  0x80000000

/* Statuses */
#define XNSYNCH_DONE    0	/* Resource available / operation complete */
#define XNSYNCH_WAIT    1	/* Calling thread blocked -- start rescheduling */
#define XNSYNCH_RESCHED 2	/* Force rescheduling */

struct xnthread;
struct xnsynch;
struct xnmutex;

typedef struct xnsynch {

    xnpholder_t link;	/* Link in claim queues */

#define link2synch(ln)		container_of(ln, struct xnsynch, link)

    xnflags_t status;	/* Status word */

    xnpqueue_t pendq;	/* Pending threads */

    struct xnthread *owner; /* Thread which owns the resource */

#ifdef CONFIG_XENO_FASTSYNCH
    xnarch_atomic_t *fastlock; /* Pointer to fast lock word */
#endif /* CONFIG_XENO_FASTSYNCH */

    void (*cleanup)(struct xnsynch *synch); /* Cleanup handler */

    XNARCH_DECL_DISPLAY_CONTEXT();

} xnsynch_t;

#define xnsynch_test_flags(synch,flags)	testbits((synch)->status,flags)
#define xnsynch_set_flags(synch,flags)	setbits((synch)->status,flags)
#define xnsynch_clear_flags(synch,flags)	clrbits((synch)->status,flags)
#define xnsynch_wait_queue(synch)		(&((synch)->pendq))
#define xnsynch_nsleepers(synch)		countpq(&((synch)->pendq))
#define xnsynch_pended_p(synch)		(!emptypq_p(&((synch)->pendq)))
#define xnsynch_owner(synch)		((synch)->owner)

#ifdef CONFIG_XENO_FASTSYNCH
#define xnsynch_fastlock(synch)		((synch)->fastlock)
#define xnsynch_fastlock_p(synch)	((synch)->fastlock != NULL)
#define xnsynch_owner_check(synch, thread) \
	xnsynch_fast_owner_check((synch)->fastlock, xnthread_handle(thread))
#else /* !CONFIG_XENO_FASTSYNCH */
#define xnsynch_fastlock(synch)		((xnarch_atomic_t *)NULL)
#define xnsynch_fastlock_p(synch)	0
#define xnsynch_owner_check(synch, thread) \
	((synch)->owner == thread ? 0 : -EPERM)
#endif /* !CONFIG_XENO_FASTSYNCH */

#define xnsynch_fast_is_claimed(fastlock) \
	xnhandle_test_spare(fastlock, XNSYNCH_FLCLAIM)
#define xnsynch_fast_set_claimed(fastlock, enable) \
	(((fastlock) & ~XNSYNCH_FLCLAIM) | ((enable) ? XNSYNCH_FLCLAIM : 0))
#define xnsynch_fast_mask_claimed(fastlock) ((fastlock) & ~XNSYNCH_FLCLAIM)

#ifdef __cplusplus
extern "C" {
#endif

#if XENO_DEBUG(SYNCH_RELAX)

void xnsynch_detect_relaxed_owner(struct xnsynch *synch,
				  struct xnthread *sleeper);

void xnsynch_detect_claimed_relax(struct xnthread *owner);

#else /* !XENO_DEBUG(SYNCH_RELAX) */

static inline void xnsynch_detect_relaxed_owner(struct xnsynch *synch,
				  struct xnthread *sleeper)
{
}

static inline void xnsynch_detect_claimed_relax(struct xnthread *owner)
{
}

#endif /* !XENO_DEBUG(SYNCH_RELAX) */

void xnsynch_init(struct xnsynch *synch, xnflags_t flags,
		  xnarch_atomic_t *fastlock);

#define xnsynch_destroy(synch)	xnsynch_flush(synch, XNRMID)

static inline void xnsynch_set_owner(struct xnsynch *synch,
				     struct xnthread *thread)
{
	synch->owner = thread;
}

static inline void xnsynch_register_cleanup(struct xnsynch *synch,
					    void (*handler)(struct xnsynch *))
{
	synch->cleanup = handler;
}

xnflags_t xnsynch_sleep_on(struct xnsynch *synch,
			   xnticks_t timeout,
			   xntmode_t timeout_mode);

struct xnthread *xnsynch_wakeup_one_sleeper(struct xnsynch *synch);

xnpholder_t *xnsynch_wakeup_this_sleeper(struct xnsynch *synch,
					 xnpholder_t *holder);

xnflags_t xnsynch_acquire(struct xnsynch *synch,
			  xnticks_t timeout,
			  xntmode_t timeout_mode);

struct xnthread *xnsynch_release(struct xnsynch *synch);

struct xnthread *xnsynch_peek_pendq(struct xnsynch *synch);

int xnsynch_flush(struct xnsynch *synch, xnflags_t reason);

void xnsynch_release_all_ownerships(struct xnthread *thread);

void xnsynch_requeue_sleeper(struct xnthread *thread);

void xnsynch_forget_sleeper(struct xnthread *thread);

#ifdef __cplusplus
}
#endif

#endif /* __KERNEL__ || __XENO_SIM__ */

#endif /* !_XENO_NUCLEUS_SYNCH_H_ */
