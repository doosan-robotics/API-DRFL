/**
 * @file
 * @note Copyright (C) 2006,2007 Philippe Gerum <rpm@xenomai.org>.
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
 * \ingroup timebase
 */

#ifndef _XENO_NUCLEUS_TIMEBASE_H
#define _XENO_NUCLEUS_TIMEBASE_H

/*! \addtogroup timebase
 *@{*/

#include <nucleus/queue.h>

#if defined(__KERNEL__) || defined(__XENO_SIM__)

#include <nucleus/vfile.h>

struct xntimer;

typedef struct xntbops {

	int (*start_timer)(struct xntimer *timer,
			   xnticks_t value,
			   xnticks_t interval,
			   xntmode_t mode);
	void (*stop_timer)(struct xntimer *timer);
	xnticks_t (*get_timer_date)(struct xntimer *timer);
	xnticks_t (*get_timer_timeout)(struct xntimer *timer);
	xnticks_t (*get_timer_interval)(struct xntimer *timer);
	xnticks_t (*get_timer_raw_expiry)(struct xntimer *timer);
	void (*move_timer)(struct xntimer *timer);

} xntbops_t;

#define XNTBRUN  0x00000001	/* Time base is running. */
#define XNTBSET  0x00000002	/* Time set in time base. */
#define XNTBLCK  0x00000004	/* Time base is locked. */
#define XNTBISO  0x00000008	/* Time base uses private wallclock offset */

typedef struct xntbase {

	struct xntbops *ops;	/*!< Time base operations. */

	xnticks_t jiffies;	/*!< Ticks elapsed since init (remains null if aperiodic). */

	void (*hook)(void);	/*!< Hook routine called upon tick. */

	xnticks_t wallclock_offset; /*!< (Wallclock time - epoch) in ticks. */

	u_long tickvalue;	/*!< Tick duration (ns, 1 if aperiodic). */

	u_long ticks2sec;	/*!< Number of ticks per second. */

	u_long status;		/*!< Status information. */

	const char *name;	/* !< Name of time base. */

	xnholder_t link;

#define link2tbase(ln)		container_of(ln, xntbase_t, link)

#ifdef CONFIG_XENO_OPT_STATS
	struct xnvfile_snapshot vfile;	/* !< Virtual file for access. */
	struct xnvfile_rev_tag revtag; /* !< Revision (for non-atomic list walks). */
	struct xnqueue timerq;	/* !< Timer holder in timebase. */
#endif /* CONFIG_XENO_OPT_STATS */

} xntbase_t;

#ifdef __cplusplus
extern "C" {
#endif

extern xntbase_t nktbase;

extern xnqueue_t nktimebaseq;

static inline u_long xntbase_get_ticks2sec(xntbase_t *base)
{
	return base->ticks2sec;
}

static inline u_long xntbase_get_tickval(xntbase_t *base)
{
	/* Returns the duration of a tick in nanoseconds */
	return base->tickvalue;
}

static inline xnticks_t xntbase_get_wallclock_offset(xntbase_t *base)
{
	return base->wallclock_offset;
}

static inline void xntbase_set_hook(xntbase_t *base, void (*hook)(void))
{
	base->hook = hook;
}

static inline int xntbase_timeset_p(xntbase_t *base)
{
	return !!testbits(base->status, XNTBSET);
}

static inline int xntbase_enabled_p(xntbase_t *base)
{
	return !!testbits(base->status, XNTBRUN);
}

static inline int xntbase_isolated_p(xntbase_t *base)
{
	return !!testbits(base->status, XNTBISO);
}

static inline const char *xntbase_name(xntbase_t *base)
{
	return base->name;
}

#ifdef CONFIG_XENO_OPT_TIMING_PERIODIC

static inline xntime_t xntbase_ticks2ns(xntbase_t *base, xnticks_t ticks)
{
	/* Convert a count of ticks in nanoseconds */
	return ticks * xntbase_get_tickval(base);
}

static inline xnticks_t xntbase_ns2ticks(xntbase_t *base, xntime_t t)
{
	return xnarch_ulldiv(t, xntbase_get_tickval(base), NULL);
}

static inline int xntbase_master_p(xntbase_t *base)
{
	return base == &nktbase;
}

static inline int xntbase_periodic_p(xntbase_t *base)
{
	return !xntbase_master_p(base);
}

static inline xnticks_t xntbase_get_jiffies(xntbase_t *base)
{
	return xntbase_periodic_p(base) ? base->jiffies : xnarch_get_cpu_time();
}

static inline xnticks_t xntbase_get_rawclock(xntbase_t *base)
{
	return xntbase_periodic_p(base) ? base->jiffies : xnarch_get_cpu_tsc();
}

int xntbase_alloc(const char *name,
		  u_long period,
		  u_long flags,
		  xntbase_t **basep);

void xntbase_free(xntbase_t *base);

int xntbase_update(xntbase_t *base,
		   u_long period);

int xntbase_switch(const char *name,
		   u_long period,
		   xntbase_t **basep);

void xntbase_start(xntbase_t *base);

void xntbase_stop(xntbase_t *base);

void xntbase_tick(xntbase_t *base);

xnticks_t xntbase_ns2ticks_ceil(xntbase_t *base, xntime_t t);

xnticks_t xntbase_convert(xntbase_t *srcbase,
			  xnticks_t ticks,
			  xntbase_t *dstbase);

#else /* !CONFIG_XENO_OPT_TIMING_PERIODIC */

void xntimer_tick_aperiodic(void);

static inline xntime_t xntbase_ticks2ns(xntbase_t *base, xnticks_t ticks)
{
	return ticks;
}

static inline xnticks_t xntbase_ns2ticks(xntbase_t *base, xntime_t t)
{
	return t;
}

static inline xnticks_t xntbase_ns2ticks_ceil(xntbase_t *base, xntime_t t)
{
	return t;
}

static inline int xntbase_master_p(xntbase_t *base)
{
	return 1;
}

static inline xnticks_t xntbase_convert(xntbase_t *srcbase, xnticks_t ticks, xntbase_t *dstbase)
{
	return ticks;
}

static inline int xntbase_periodic_p(xntbase_t *base)
{
	return 0;
}

static inline xnticks_t xntbase_get_jiffies(xntbase_t *base)
{
	return xnarch_get_cpu_time();
}

static inline xnticks_t xntbase_get_rawclock(xntbase_t *base)
{
	return xnarch_get_cpu_tsc();
}

static inline int xntbase_alloc(const char *name, u_long period, u_long flags, xntbase_t **basep)
{
	*basep = &nktbase;
	return 0;
}

static inline void xntbase_free(xntbase_t *base)
{
}

static inline int xntbase_update(xntbase_t *base, u_long period)
{
	return 0;
}

static inline int xntbase_switch(const char *name, u_long period, xntbase_t **basep)
{
	return period == XN_APERIODIC_TICK ? 0 : -ENODEV;
}

static inline void xntbase_start(xntbase_t *base)
{
}

static inline void xntbase_stop(xntbase_t *base)
{
}

static inline void xntbase_tick(xntbase_t *base)
{
	xntimer_tick_aperiodic();
}

#endif /* !CONFIG_XENO_OPT_TIMING_PERIODIC */

/*!
 * @fn xnticks_t xntbase_get_time(xntbase_t *base)
 * @brief Get the clock time for a given time base.
 *
 * This service returns the (external) clock time as maintained by the
 * specified time base. This value is adjusted with the wallclock
 * offset as defined by xntbase_adjust_time().
 *
 * @param base The address of the time base to query.
 *
 * @return The current time (in jiffies) if the specified time base
 * runs in periodic mode, or the machine time (converted to
 * nanoseconds) as maintained by the hardware if @a base refers to the
 * master time base.
 *
 * Environments:
 *
 * This service can be called from:
 *
 * - Kernel module initialization/cleanup code
 * - Interrupt service routine
 * - Kernel-based task
 * - User-space task
 *
 * Rescheduling: never.
 */

static inline xnticks_t xntbase_get_time(xntbase_t *base)
{
	/* Return an adjusted value of the monotonic time with the
	   translated system wallclock offset. */
	return xntbase_get_jiffies(base) + base->wallclock_offset;
}

void xntbase_adjust_time(xntbase_t *base, xnsticks_t delta);

#ifdef __cplusplus
}
#endif

#define xntbase_mount() \
do {						\
	inith(&nktbase.link);			\
	appendq(&nktimebaseq, &nktbase.link);	\
	xntbase_declare_proc(&nktbase);	\
} while (0)

#define xntbase_umount()			\
do {						\
	xntbase_remove_proc(&nktbase);		\
	removeq(&nktimebaseq, &nktbase.link);	\
} while (0)

void xntbase_init_proc(void);

void xntbase_cleanup_proc(void);

#ifdef CONFIG_XENO_OPT_STATS
void xntbase_declare_proc(xntbase_t *base);
void xntbase_remove_proc(xntbase_t *base);
#else /* !CONFIG_XENO_OPT_STATS */
static inline void xntbase_declare_proc(xntbase_t *base) { }
static inline void xntbase_remove_proc(xntbase_t *base) { }
#endif /* !CONFIG_XENO_OPT_STATS */

extern struct xnvfile_rev_tag tbaselist_tag;

#endif /* __KERNEL__ || __XENO_SIM__ */

/*@}*/

#endif /* !_XENO_NUCLEUS_TIMEBASE_H */
