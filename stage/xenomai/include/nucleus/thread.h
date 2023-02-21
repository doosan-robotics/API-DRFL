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
 * \ingroup thread
 */

#ifndef _XENO_NUCLEUS_THREAD_H
#define _XENO_NUCLEUS_THREAD_H

#include <nucleus/types.h>

/*! @ingroup nucleus
  @defgroup nucleus_state_flags Thread state flags.
  @brief Bits reporting permanent or transient states of thread.
  @{
*/

/* State flags */

#define XNSUSP    0x00000001 /**< Suspended. */
#define XNPEND    0x00000002 /**< Sleep-wait for a resource. */
#define XNDELAY   0x00000004 /**< Delayed */
#define XNREADY   0x00000008 /**< Linked to the ready queue. */
#define XNDORMANT 0x00000010 /**< Not started yet or killed */
#define XNZOMBIE  0x00000020 /**< Zombie thread in deletion process */
#define XNRESTART 0x00000040 /**< Restarting thread */
#define XNSTARTED 0x00000080 /**< Thread has been started */
#define XNMAPPED  0x00000100 /**< Mapped to a regular Linux task (shadow only) */
#define XNRELAX   0x00000200 /**< Relaxed shadow thread (blocking bit) */
#define XNMIGRATE 0x00000400 /**< Thread is currently migrating to another CPU. */
#define XNHELD    0x00000800 /**< Thread is held to process emergency. */

#define XNBOOST   0x00001000 /**< Undergoes a PIP boost */
#define XNDEBUG   0x00002000 /**< Hit a debugger breakpoint (shadow only) */
#define XNLOCK    0x00004000 /**< Holds the scheduler lock (i.e. not preemptible) */
#define XNRRB     0x00008000 /**< Undergoes a round-robin scheduling */
#define XNASDI    0x00010000 /**< ASR are disabled */
#define XNDEFCAN  0x00020000 /**< Deferred cancelability mode (self-set only) */

/*
 * Some skins may depend on the following fields to live in the high
 * 16-bit word, in order to be combined with the emulated RTOS flags
 * which use the low one, so don't change them carelessly.
 */
#define XNTRAPSW  0x00040000 /**< Trap execution mode switches */
#define XNRPIOFF  0x00080000 /**< Stop priority coupling (shadow only) */
#define XNFPU     0x00100000 /**< Thread uses FPU */
#define XNSHADOW  0x00200000 /**< Shadow thread */
#define XNROOT    0x00400000 /**< Root thread (that is, Linux/IDLE) */
#define XNOTHER   0x00800000 /**< Non real-time shadow (prio=0) */

/*! @} */ /* Ends doxygen comment group: nucleus_state_flags */

/*
  Must follow the declaration order of the above bits. Status symbols
  are defined as follows:
  'S' -> Forcibly suspended.
  'w'/'W' -> Waiting for a resource, with or without timeout.
  'D' -> Delayed (without any other wait condition).
  'R' -> Runnable.
  'U' -> Unstarted or dormant.
  'X' -> Relaxed shadow.
  'H' -> Held in emergency.
  'b' -> Priority boost undergoing.
  'T' -> Ptraced and stopped.
  'l' -> Locks scheduler.
  'r' -> Undergoes round-robin.
  's' -> Interrupt shield enabled.
  't' -> Mode switches trapped.
  'o' -> Priority coupling off.
  'f' -> FPU enabled (for kernel threads).
*/
#define XNTHREAD_STATE_LABELS  "SWDRU....X.HbTlr..tof.."

#define XNTHREAD_BLOCK_BITS   (XNSUSP|XNPEND|XNDELAY|XNDORMANT|XNRELAX|XNMIGRATE|XNHELD)
#define XNTHREAD_MODE_BITS    (XNLOCK|XNRRB|XNASDI|XNTRAPSW|XNRPIOFF)

/* These state flags are available to the real-time interfaces */
#define XNTHREAD_STATE_SPARE0  0x10000000
#define XNTHREAD_STATE_SPARE1  0x20000000
#define XNTHREAD_STATE_SPARE2  0x40000000
#define XNTHREAD_STATE_SPARE3  0x80000000
#define XNTHREAD_STATE_SPARES  0xf0000000

/*! @ingroup nucleus
  @defgroup nucleus_info_flags Thread information flags.
  @brief Bits reporting events notified to the thread.
  @{
*/

/* Information flags */

#define XNTIMEO   0x00000001 /**< Woken up due to a timeout condition */
#define XNRMID    0x00000002 /**< Pending on a removed resource */
#define XNBREAK   0x00000004 /**< Forcibly awaken from a wait state */
#define XNKICKED  0x00000008 /**< Kicked upon Linux signal (shadow only) */
#define XNWAKEN   0x00000010 /**< Thread waken up upon resource availability */
#define XNROBBED  0x00000020 /**< Robbed from resource ownership */
#define XNATOMIC  0x00000040 /**< In atomic switch from secondary to primary mode */
#define XNAFFSET  0x00000080 /**< CPU affinity changed from primary mode */
#define XNPRIOSET 0x00000100 /**< Priority changed from primary mode */
#define XNABORT   0x00000200 /**< Thread is being aborted */
#define XNCANPND  0x00000400 /**< Cancellation request is pending */
#define XNAMOK    0x00000800 /**< Runaway, watchdog signal pending (shadow only) */
#define XNSWREP   0x00001000 /**< Mode switch already reported */

/* These information flags are available to the real-time interfaces */
#define XNTHREAD_INFO_SPARE0  0x10000000
#define XNTHREAD_INFO_SPARE1  0x20000000
#define XNTHREAD_INFO_SPARE2  0x40000000
#define XNTHREAD_INFO_SPARE3  0x80000000
#define XNTHREAD_INFO_SPARES  0xf0000000

/*! @} */ /* Ends doxygen comment group: nucleus_info_flags */

/*!
  @brief Structure containing thread information.
*/
typedef struct xnthread_info {

	unsigned long state; /**< Thread state, @see nucleus_state_flags */

	int bprio;  /**< Base priority. */
	int cprio; /**< Current priority. May change through Priority Inheritance.*/

	int cpu; /**< CPU the thread currently runs on. */
	unsigned long affinity; /**< Thread's CPU affinity. */

	unsigned long long relpoint; /**< Time of next release.*/

	unsigned long long exectime; /**< Execution time in primary mode in nanoseconds. */

	unsigned long modeswitches; /**< Number of primary->secondary mode switches. */
	unsigned long ctxswitches; /**< Number of context switches. */
	unsigned long pagefaults; /**< Number of triggered page faults. */

	char name[XNOBJECT_NAME_LEN];  /**< Symbolic name assigned at creation. */

} xnthread_info_t;

#if defined(__KERNEL__) || defined(__XENO_SIM__)

#include <nucleus/stat.h>
#include <nucleus/timer.h>
#include <nucleus/registry.h>
#include <nucleus/schedparam.h>

#ifdef __XENO_SIM__
/* Pseudo-status (must not conflict with other bits) */
#define XNRUNNING  XNTHREAD_STATE_SPARE0
#define XNDELETED  XNTHREAD_STATE_SPARE1
#endif /* __XENO_SIM__ */

#define XNTHREAD_INVALID_ASR  ((void (*)(xnsigmask_t))0)

struct xnthread;
struct xnsynch;
struct xnsched;
struct xnselector;
struct xnsched_class;
struct xnsched_tpslot;
union xnsched_policy_param;
struct xnbufd;

struct xnthread_operations {
	int (*get_denormalized_prio)(struct xnthread *, int coreprio);
	unsigned (*get_magic)(void);
};

struct xnthread_init_attr {
	struct xntbase *tbase;
	struct xnthread_operations *ops;
	xnflags_t flags;
	unsigned int stacksize;
	const char *name;
};

struct xnthread_start_attr {
	xnflags_t mode;
	int imask;
	xnarch_cpumask_t affinity;
	void (*entry)(void *cookie);
	void *cookie;
};

struct xnthread_wait_context {
	unsigned long oldstate;
};

typedef void (*xnasr_t)(xnsigmask_t sigs);

typedef struct xnthread {

	xnarchtcb_t tcb;		/* Architecture-dependent block -- Must be first */

	xnflags_t state;		/* Thread state flags */

	xnflags_t info;			/* Thread information flags */

	struct xnsched *sched;		/* Thread scheduler */

	struct xnsched_class *sched_class; /* Current scheduling class */

	struct xnsched_class *base_class; /* Base scheduling class */

#ifdef CONFIG_XENO_OPT_SCHED_TP
	struct xnsched_tpslot *tps;	/* Current partition slot for TP scheduling */
	struct xnholder tp_link;	/* Link in per-sched TP thread queue */
#endif
#ifdef CONFIG_XENO_OPT_SCHED_SPORADIC
	struct xnsched_sporadic_data *pss; /* Sporadic scheduling data. */
#endif

	unsigned idtag;			/* Unique ID tag */

	xnarch_cpumask_t affinity;	/* Processor affinity. */

	int bprio;			/* Base priority (before PIP boost) */

	int cprio;			/* Current priority */

	u_long schedlck;		/*!< Scheduler lock count. */

	xnpholder_t rlink;		/* Thread holder in ready queue */

	xnpholder_t plink;		/* Thread holder in synchronization queue(s) */

#ifdef CONFIG_XENO_OPT_PRIOCPL
	xnpholder_t xlink;		/* Thread holder in the RPI queue (shadow only) */

	struct xnsched *rpi;		/* Backlink pointer to the RPI slot (shadow only) */
#endif /* CONFIG_XENO_OPT_PRIOCPL */

	xnholder_t glink;		/* Thread holder in global queue */

#define link2thread(ln, fld)	container_of(ln, struct xnthread, fld)

	xnpqueue_t claimq;		/* Owned resources claimed by others (PIP) */

	struct xnsynch *wchan;		/* Resource the thread pends on */

	struct xnsynch *wwake;		/* Wait channel the thread was resumed from */

	int hrescnt;			/* Held resources count */

	xntimer_t rtimer;		/* Resource timer */

	xntimer_t ptimer;		/* Periodic timer */

	xnsigmask_t signals;		/* Pending core signals */

	xnticks_t rrperiod;		/* Allotted round-robin period (ticks) */

	xnticks_t rrcredit;		/* Remaining round-robin time credit (ticks) */

	union {
		struct {
			/*
			 * XXX: the buffer struct should disappear as
			 * soon as all IPCs are converted to use
			 * buffer descriptors instead (bufd).
			 */
			void *ptr;
			size_t size;
		} buffer;
		struct xnbufd *bufd;
		size_t size;
	} wait_u;

	/* Active wait context - Obsoletes wait_u. */
	struct xnthread_wait_context *wcontext;

	struct {
		xnstat_counter_t ssw;	/* Primary -> secondary mode switch count */
		xnstat_counter_t csw;	/* Context switches (includes secondary -> primary switches) */
		xnstat_counter_t pf;	/* Number of page faults */
		xnstat_exectime_t account; /* Execution time accounting entity */
		xnstat_exectime_t lastperiod; /* Interval marker for execution time reports */
	} stat;

#ifdef CONFIG_XENO_OPT_SELECT
	struct xnselector *selector;    /* For select. */
#endif /* CONFIG_XENO_OPT_SELECT */

	int errcode;			/* Local errno */

	xnasr_t asr;			/* Asynchronous service routine */

	xnflags_t asrmode;		/* Thread's mode for ASR */

	int asrimask;			/* Thread's interrupt mask for ASR */

	unsigned asrlevel;		/* ASR execution level (ASRs are reentrant) */

	int imask;			/* Initial interrupt mask */

	int imode;			/* Initial mode */

	struct xnsched_class *init_class; /* Initial scheduling class */

	union xnsched_policy_param init_schedparam; /* Initial scheduling parameters */

	struct {
		xnhandle_t handle;	/* Handle in registry */
		const char *waitkey;	/* Pended key */
	} registry;

	struct xnthread_operations *ops; /* Thread class operations. */

	char name[XNOBJECT_NAME_LEN]; /* Symbolic name of thread */

	void (*entry)(void *cookie); /* Thread entry routine */

	void *cookie;		/* Cookie to pass to the entry routine */

#ifdef CONFIG_XENO_OPT_PERVASIVE
	unsigned long *u_mode;	/* Thread mode variable shared with userland. */
#endif /* CONFIG_XENO_OPT_PERVASIVE */

    XNARCH_DECL_DISPLAY_CONTEXT();

} xnthread_t;

#define XNHOOK_THREAD_START  1
#define XNHOOK_THREAD_SWITCH 2
#define XNHOOK_THREAD_DELETE 3

typedef struct xnhook {
	xnholder_t link;
#define link2hook(ln)		container_of(ln, xnhook_t, link)
	void (*routine)(struct xnthread *thread);
} xnhook_t;

#define xnthread_name(thread)               ((thread)->name)
#define xnthread_clear_name(thread)        do { *(thread)->name = 0; } while(0)
#define xnthread_sched(thread)             ((thread)->sched)
#define xnthread_start_time(thread)        ((thread)->stime)
#define xnthread_state_flags(thread)       ((thread)->state)
#define xnthread_test_state(thread,flags)  testbits((thread)->state,flags)
#define xnthread_set_state(thread,flags)   __setbits((thread)->state,flags)
#define xnthread_clear_state(thread,flags) __clrbits((thread)->state,flags)
#define xnthread_test_info(thread,flags)   testbits((thread)->info,flags)
#define xnthread_set_info(thread,flags)    __setbits((thread)->info,flags)
#define xnthread_clear_info(thread,flags)  __clrbits((thread)->info,flags)
#define xnthread_lock_count(thread)        ((thread)->schedlck)
#define xnthread_init_schedparam(thread)   ((thread)->init_schedparam)
#define xnthread_base_priority(thread)     ((thread)->bprio)
#define xnthread_current_priority(thread)  ((thread)->cprio)
#define xnthread_init_class(thread)        ((thread)->init_class)
#define xnthread_base_class(thread)        ((thread)->base_class)
#define xnthread_sched_class(thread)       ((thread)->sched_class)
#define xnthread_time_slice(thread)        ((thread)->rrperiod)
#define xnthread_time_credit(thread)       ((thread)->rrcredit)
#define xnthread_archtcb(thread)           (&((thread)->tcb))
#define xnthread_asr_level(thread)         ((thread)->asrlevel)
#define xnthread_pending_signals(thread)  ((thread)->signals)
#define xnthread_timeout(thread)           xntimer_get_timeout(&(thread)->rtimer)
#define xnthread_stack_size(thread)        xnarch_stack_size(xnthread_archtcb(thread))
#define xnthread_stack_base(thread)        xnarch_stack_base(xnthread_archtcb(thread))
#define xnthread_stack_end(thread)         xnarch_stack_end(xnthread_archtcb(thread))
#define xnthread_handle(thread)            ((thread)->registry.handle)
#ifdef CONFIG_XENO_OPT_TIMING_PERIODIC
#define xnthread_time_base(thread)         ((thread)->rtimer.base)
#else /* !CONFIG_XENO_OPT_TIMING_PERIODIC */
#define xnthread_time_base(thread)         (&nktbase)
#endif /* !CONFIG_XENO_OPT_TIMING_PERIODIC */
#define xnthread_signaled_p(thread)        ((thread)->signals != 0)
#define xnthread_timed_p(thread)	      (!!testbits(xnthread_time_base(thread)->status, XNTBRUN))
#define xnthread_user_task(thread)         xnarch_user_task(xnthread_archtcb(thread))
#define xnthread_user_pid(thread) \
    (xnthread_test_state((thread),XNROOT) || !xnthread_user_task(thread) ? \
    0 : xnarch_user_pid(xnthread_archtcb(thread)))
#define xnthread_current_user_task(curr)				\
	({								\
		struct xnthread *__c = (curr);				\
		XENO_BUGON(NUCLEUS, __c != xnpod_current_thread());	\
		xnthread_test_state(__c, XNROOT|XNSHADOW) ? current : NULL; \
	})
#define xnthread_current_user_pid(curr)					\
	({								\
		struct task_struct *p = xnthread_current_user_task(curr); \
		p ? p->pid : 0;						\
	})
#define xnthread_affinity(thread)          ((thread)->affinity)
#define xnthread_affine_p(thread, cpu)     xnarch_cpu_isset(cpu, (thread)->affinity)
#define xnthread_get_exectime(thread)      xnstat_exectime_get_total(&(thread)->stat.account)
#define xnthread_get_lastswitch(thread)    xnstat_exectime_get_last_switch((thread)->sched)
#ifdef CONFIG_XENO_OPT_PERVASIVE
#define xnthread_inc_rescnt(thread)        ({ (thread)->hrescnt++; })
#define xnthread_dec_rescnt(thread)        ({ --(thread)->hrescnt; })
#define xnthread_get_rescnt(thread)        ((thread)->hrescnt)
#else /* !CONFIG_XENO_OPT_PERVASIVE */
#define xnthread_inc_rescnt(thread)        do { } while (0)
#define xnthread_dec_rescnt(thread)        do { } while (0)
#endif /* !CONFIG_XENO_OPT_PERVASIVE */
#if defined(CONFIG_XENO_OPT_WATCHDOG) || defined(CONFIG_XENO_SKIN_POSIX)
#define xnthread_amok_p(thread)            xnthread_test_info(thread, XNAMOK)
#define xnthread_clear_amok(thread)        xnthread_clear_info(thread, XNAMOK)
#else /* !CONFIG_XENO_OPT_WATCHDOG && !CONFIG_XENO_SKIN_POSIX */
#define xnthread_amok_p(thread)            ({ (void)(thread); 0; })
#define xnthread_clear_amok(thread)        do { (void)(thread); } while (0)
#endif /* !CONFIG_XENO_OPT_WATCHDOG && !CONFIG_XENO_SKIN_POSIX */

/* Class-level operations for threads. */
static inline int xnthread_get_denormalized_prio(struct xnthread *t, int coreprio)
{
	return t->ops && t->ops->get_denormalized_prio
		? t->ops->get_denormalized_prio(t, coreprio) : coreprio;
}

static inline unsigned xnthread_get_magic(struct xnthread *t)
{
	return t->ops ? t->ops->get_magic() : 0;
}

static inline
struct xnthread_wait_context *xnthread_get_wait_context(struct xnthread *thread)
{
	return thread->wcontext;
}

static inline
int xnthread_register(struct xnthread *thread, const char *name)
{
	return xnregistry_enter(name, thread, &xnthread_handle(thread), NULL);
}

static inline
struct xnthread *xnthread_lookup(xnhandle_t threadh)
{
	struct xnthread *thread = (struct xnthread *)xnregistry_lookup(threadh);
	return (thread && xnthread_handle(thread) == threadh) ? thread : NULL;
}

static inline int xnthread_try_grab(struct xnthread *curr,
				    struct xnsynch *synch)
{
	XENO_BUGON(NUCLEUS, xnsynch_fastlock_p(synch));

	if (xnsynch_owner(synch) != NULL)
		return 0;

	xnsynch_set_owner(synch, curr);

	if (xnthread_test_state(curr, XNOTHER))
		xnthread_inc_rescnt(curr);

	return 1;
}

#ifdef __cplusplus
extern "C" {
#endif

int xnthread_init(struct xnthread *thread,
		  const struct xnthread_init_attr *attr,
		  struct xnsched *sched,
		  struct xnsched_class *sched_class,
		  const union xnsched_policy_param *sched_param);

void xnthread_cleanup_tcb(struct xnthread *thread);

char *xnthread_format_status(xnflags_t status, char *buf, int size);

int *xnthread_get_errno_location(struct xnthread *thread);

xnticks_t xnthread_get_timeout(struct xnthread *thread, xnticks_t tsc_ns);

xnticks_t xnthread_get_period(struct xnthread *thread);

void xnthread_prepare_wait(struct xnthread_wait_context *wc);

void xnthread_finish_wait(struct xnthread_wait_context *wc,
			  void (*cleanup)(struct xnthread_wait_context *wc));

#ifdef __cplusplus
}
#endif

#endif /* __KERNEL__ || __XENO_SIM__ */

#endif /* !_XENO_NUCLEUS_THREAD_H */
