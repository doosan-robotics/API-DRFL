/*!\file pod.h
 * \brief Real-time pod interface header.
 * \author Philippe Gerum
 *
 * Copyright (C) 2001-2007 Philippe Gerum <rpm@xenomai.org>.
 * Copyright (C) 2004 The RTAI project <http://www.rtai.org>
 * Copyright (C) 2004 The HYADES project <http://www.hyades-itea.org>
 * Copyright (C) 2004 The Xenomai project <http://www.xenomai.org>
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
 * \ingroup pod
 */

#ifndef _XENO_NUCLEUS_POD_H
#define _XENO_NUCLEUS_POD_H

/*! \addtogroup pod
 *@{*/

#include <nucleus/sched.h>

/* Pod status flags */
#define XNFATAL  0x00000001	/* Fatal error in progress */
#define XNPEXEC  0x00000002	/* Pod is active (a skin is attached) */

/* These flags are available to the real-time interfaces */
#define XNPOD_SPARE0  0x01000000
#define XNPOD_SPARE1  0x02000000
#define XNPOD_SPARE2  0x04000000
#define XNPOD_SPARE3  0x08000000
#define XNPOD_SPARE4  0x10000000
#define XNPOD_SPARE5  0x20000000
#define XNPOD_SPARE6  0x40000000
#define XNPOD_SPARE7  0x80000000

#define XNPOD_NORMAL_EXIT  0x0
#define XNPOD_FATAL_EXIT   0x1

#define XNPOD_ALL_CPUS  XNARCH_CPU_MASK_ALL

#define XNPOD_FATAL_BUFSZ  16384

#define nkpod (&nkpod_struct)

struct xnsynch;

/*!
 * \brief Real-time pod descriptor.
 *
 * The source of all Xenomai magic.
 */

struct xnpod {

	xnflags_t status;	/*!< Status bitmask. */

	xnsched_t sched[XNARCH_NR_CPUS];	/*!< Per-cpu scheduler slots. */

	xnqueue_t threadq;	/*!< All existing threads. */
#ifdef CONFIG_XENO_OPT_VFILE
	struct xnvfile_rev_tag threadlist_tag;
#endif
	xnqueue_t tstartq,	/*!< Thread start hook queue. */
	 tswitchq,		/*!< Thread switch hook queue. */
	 tdeleteq;		/*!< Thread delete hook queue. */

	atomic_counter_t timerlck;	/*!< Timer lock depth.  */

	xntimer_t tslicer;	/*!< Time-slicing timer for aperiodic mode  */
	int tsliced;		/*!< Number of threads using the slicer */

	int refcnt;		/*!< Reference count.  */

#ifdef __XENO_SIM__
	void (*schedhook) (xnthread_t *thread, xnflags_t mask);	/*!< Internal scheduling hook. */
#endif	/* __XENO_SIM__ */
};

typedef struct xnpod xnpod_t;

DECLARE_EXTERN_XNLOCK(nklock);

extern u_long nklatency;

extern u_long nktimerlat;

extern xnarch_cpumask_t nkaffinity;

extern xnpod_t nkpod_struct;

#ifdef CONFIG_XENO_OPT_VFILE
int xnpod_init_proc(void);
void xnpod_cleanup_proc(void);
#else /* !CONFIG_XENO_OPT_VFILE */
static inline int xnpod_init_proc(void) { return 0; }
static inline void xnpod_cleanup_proc(void) {}
#endif /* !CONFIG_XENO_OPT_VFILE */

static inline int xnpod_mount(void)
{
	xnsched_register_classes();
	return xnpod_init_proc();
}

static inline void xnpod_umount(void)
{
	xnpod_cleanup_proc();
}

#ifdef __cplusplus
extern "C" {
#endif

int __xnpod_set_thread_schedparam(struct xnthread *thread,
				  struct xnsched_class *sched_class,
				  const union xnsched_policy_param *sched_param,
				  int propagate);

#ifdef CONFIG_XENO_HW_FPU
void xnpod_switch_fpu(xnsched_t *sched);
#endif /* CONFIG_XENO_HW_FPU */

void __xnpod_schedule(struct xnsched *sched);

	/* -- Beginning of the exported interface */

#define xnpod_sched_slot(cpu) \
    (&nkpod->sched[cpu])

#define xnpod_current_sched() \
    xnpod_sched_slot(xnarch_current_cpu())

#define xnpod_active_p() \
    testbits(nkpod->status, XNPEXEC)

#define xnpod_fatal_p() \
    testbits(nkpod->status, XNFATAL)

#define xnpod_interrupt_p() \
    testbits(xnpod_current_sched()->lflags, XNINIRQ)

#define xnpod_callout_p() \
    testbits(xnpod_current_sched()->status, XNKCOUT)

#define xnpod_asynch_p() \
	({								\
		xnsched_t *sched = xnpod_current_sched();		\
		testbits(sched->status | sched->lflags, XNKCOUT|XNINIRQ); \
	})

#define xnpod_current_thread() \
    (xnpod_current_sched()->curr)

#define xnpod_current_root() \
    (&xnpod_current_sched()->rootcb)

#ifdef CONFIG_XENO_OPT_PERVASIVE
#define xnpod_current_p(thread)						\
    ({ int __shadow_p = xnthread_test_state(thread, XNSHADOW);		\
       int __curr_p = __shadow_p ? xnshadow_thread(current) == thread	\
	   : thread == xnpod_current_thread();				\
       __curr_p;})
#else
#define xnpod_current_p(thread) \
    (xnpod_current_thread() == (thread))
#endif

#define xnpod_locked_p() \
    xnthread_test_state(xnpod_current_thread(), XNLOCK)

#define xnpod_unblockable_p() \
    (xnpod_asynch_p() || xnthread_test_state(xnpod_current_thread(), XNROOT))

#define xnpod_root_p() \
    xnthread_test_state(xnpod_current_thread(),XNROOT)

#define xnpod_shadow_p() \
    xnthread_test_state(xnpod_current_thread(),XNSHADOW)

#define xnpod_userspace_p() \
    xnthread_test_state(xnpod_current_thread(),XNROOT|XNSHADOW)

#define xnpod_primary_p() \
    (!(xnpod_asynch_p() || xnpod_root_p()))

#define xnpod_secondary_p()	xnpod_root_p()

#define xnpod_idle_p()		xnpod_root_p()

int xnpod_init(void);

int xnpod_enable_timesource(void);

void xnpod_disable_timesource(void);

void xnpod_shutdown(int xtype);

int xnpod_init_thread(struct xnthread *thread,
		      const struct xnthread_init_attr *attr,
		      struct xnsched_class *sched_class,
		      const union xnsched_policy_param *sched_param);

int xnpod_start_thread(xnthread_t *thread,
		       const struct xnthread_start_attr *attr);

void xnpod_stop_thread(xnthread_t *thread);

void xnpod_restart_thread(xnthread_t *thread);

void xnpod_delete_thread(xnthread_t *thread);

void xnpod_abort_thread(xnthread_t *thread);

xnflags_t xnpod_set_thread_mode(xnthread_t *thread,
				xnflags_t clrmask,
				xnflags_t setmask);

void xnpod_suspend_thread(xnthread_t *thread,
			  xnflags_t mask,
			  xnticks_t timeout,
			  xntmode_t timeout_mode,
			  struct xnsynch *wchan);

void xnpod_resume_thread(xnthread_t *thread,
			 xnflags_t mask);

int xnpod_unblock_thread(xnthread_t *thread);

int xnpod_set_thread_schedparam(struct xnthread *thread,
				struct xnsched_class *sched_class,
				const union xnsched_policy_param *sched_param);

int xnpod_migrate_thread(int cpu);

void xnpod_dispatch_signals(void);

static inline void xnpod_schedule(void)
{
	struct xnsched *sched;
	/*
	 * NOTE: Since __xnpod_schedule() won't run if an escalation
	 * to primary domain is needed, we won't use critical
	 * scheduler information before we actually run in primary
	 * mode; therefore we can first test the scheduler status then
	 * escalate.  Running in the primary domain means that no
	 * Linux-triggered CPU migration may occur from that point
	 * either. Finally, since migration is always a self-directed
	 * operation for Xenomai threads, we can safely read the
	 * scheduler state bits without holding the nklock.
	 *
	 * Said differently, if we race here because of a CPU
	 * migration, it must have been Linux-triggered because we run
	 * in secondary mode; in which case we will escalate to the
	 * primary domain, then unwind the current call frame without
	 * running the rescheduling procedure in
	 * __xnpod_schedule(). Therefore, the scheduler pointer will
	 * be either valid, or unused.
	 */
	sched = xnpod_current_sched();
	/*
	 * No immediate rescheduling is possible if an ISR or callout
	 * context is active, or if we are caught in the middle of a
	 * unlocked context switch.
	 */
#if XENO_DEBUG(NUCLEUS)
	if (testbits(sched->status | sched->lflags,
		     XNKCOUT|XNINIRQ|XNINSW|XNINLOCK))
		return;
#else /* !XENO_DEBUG(NUCLEUS) */
	if (testbits(sched->status | sched->lflags,
		     XNKCOUT|XNINIRQ|XNINSW|XNRESCHED|XNINLOCK) != XNRESCHED)
		return;
#endif /* !XENO_DEBUG(NUCLEUS) */

	__xnpod_schedule(sched);
}

void ___xnpod_lock_sched(xnsched_t *sched);

void ___xnpod_unlock_sched(xnsched_t *sched);

static inline void __xnpod_lock_sched(void)
{
	xnsched_t *sched;

	barrier();
	sched = xnpod_current_sched();
	___xnpod_lock_sched(sched);
}

static inline void __xnpod_unlock_sched(void)
{
	xnsched_t *sched;

	barrier();
	sched = xnpod_current_sched();
	___xnpod_unlock_sched(sched);
}

static inline void xnpod_lock_sched(void)
{
	xnsched_t *sched;
	spl_t s;

	xnlock_get_irqsave(&nklock, s);
	sched = xnpod_current_sched();
	___xnpod_lock_sched(sched);
	xnlock_put_irqrestore(&nklock, s);
}

static inline void xnpod_unlock_sched(void)
{
	xnsched_t *sched;
	spl_t s;

	xnlock_get_irqsave(&nklock, s);
	sched = xnpod_current_sched();
	___xnpod_unlock_sched(sched);
	xnlock_put_irqrestore(&nklock, s);
}

void xnpod_fire_callouts(xnqueue_t *hookq,
			 xnthread_t *thread);

static inline void xnpod_run_hooks(struct xnqueue *q,
				   struct xnthread *thread, const char *type)
{
	if (!emptyq_p(q)) {
		trace_mark(xn_nucleus, thread_callout,
			   "thread %p thread_name %s hook %s",
			   thread, xnthread_name(thread), type);
		xnpod_fire_callouts(q, thread);
	}
}

int xnpod_set_thread_periodic(xnthread_t *thread,
			      xnticks_t idate,
			      xnticks_t period);

int xnpod_wait_thread_period(unsigned long *overruns_r);

int xnpod_set_thread_tslice(struct xnthread *thread,
			    xnticks_t quantum);

static inline xntime_t xnpod_get_cpu_time(void)
{
	return xnarch_get_cpu_time();
}

int xnpod_add_hook(int type, void (*routine) (xnthread_t *));

int xnpod_remove_hook(int type, void (*routine) (xnthread_t *));

static inline void xnpod_yield(void)
{
	xnpod_resume_thread(xnpod_current_thread(), 0);
	xnpod_schedule();
}

static inline void xnpod_delay(xnticks_t timeout)
{
	xnpod_suspend_thread(xnpod_current_thread(), XNDELAY, timeout, XN_RELATIVE, NULL);
}

static inline void xnpod_suspend_self(void)
{
	xnpod_suspend_thread(xnpod_current_thread(), XNSUSP, XN_INFINITE, XN_RELATIVE, NULL);
}

static inline void xnpod_delete_self(void)
{
	xnpod_delete_thread(xnpod_current_thread());
}

#ifdef __cplusplus
}
#endif

/*@}*/

#endif /* !_XENO_NUCLEUS_POD_H */
