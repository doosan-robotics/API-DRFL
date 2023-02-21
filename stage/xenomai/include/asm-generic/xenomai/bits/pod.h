/*
 * Copyright (C) 2001,2002,2003,2004,2005 Philippe Gerum <rpm@xenomai.org>.
 * Copyright (C) 2004,2005 Gilles Chanteperdrix <gilles.chanteperdrix@xenomai.org>.
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

#ifndef _XENO_ASM_GENERIC_BITS_POD_H
#define _XENO_ASM_GENERIC_BITS_POD_H

#ifndef __KERNEL__
#error "Pure kernel header included from user-space!"
#endif

#ifdef CONFIG_GENERIC_CLOCKEVENTS

#include <linux/tick.h>
#include <linux/ipipe_tickdev.h>

/*!
 * @internal
 * \fn void xnarch_next_htick_shot(unsigned long delay, struct clock_event_device *cdev)
 *
 * \brief Next tick setup emulation callback.
 *
 * Program the next shot for the host tick on the current CPU.
 * Emulation is done using a nucleus timer attached to the master
 * timebase.
 *
 * @param delay The time delta from the current date to the next tick,
 * expressed as a count of nanoseconds.
 *
 * @param cdev An pointer to the clock device which notifies us.
 *
 * Environment:
 *
 * This routine is a callback invoked from the kernel's clock event
 * handlers.
 *
 * @note Only Linux kernel releases which support clock event devices
 * (CONFIG_GENERIC_CLOCKEVENTS) would call this routine when the
 * latter are programmed in oneshot mode. Otherwise, periodic host
 * tick emulation is directly handled by the nucleus, and does not
 * involve any callback mechanism from the Linux kernel.
 *
 * Rescheduling: never.
 */

static int xnarch_next_htick_shot(unsigned long delay, struct clock_event_device *cdev)
{
	xnsched_t *sched;
	int ret;
	spl_t s;

	xnlock_get_irqsave(&nklock, s);
	sched = xnpod_current_sched();
	ret = xntimer_start(&sched->htimer, delay, XN_INFINITE, XN_RELATIVE);
	xnlock_put_irqrestore(&nklock, s);

	return ret ? -ETIME : 0;
}

/*!
 * @internal
 * \fn void xnarch_switch_htick_mode(enum clock_event_mode mode, struct clock_event_device *cdev)
 *
 * \brief Tick mode switch emulation callback.
 *
 * Changes the host tick mode for the tick device of the current CPU.
 *
 * @param mode The new mode to switch to. The possible values are:
 *
 * - CLOCK_EVT_MODE_ONESHOT, for a switch to oneshot mode.
 *
 * - CLOCK_EVT_MODE_PERIODIC, for a switch to periodic mode. The current
 * implementation for the generic clockevent layer Linux exhibits
 * should never downgrade from a oneshot to a periodic tick mode, so
 * this mode should not be encountered. This said, the associated code
 * is provided, basically for illustration purposes.
 *
 * - CLOCK_EVT_MODE_SHUTDOWN, indicates the removal of the current
 * tick device. Normally, the HAL code only interposes on tick devices
 * which should never be shut down, so this mode should not be
 * encountered.
 *
 * @param cdev An opaque pointer to the clock device which notifies us.
 *
 * Environment:
 *
 * This routine is a callback invoked from the kernel's clock event
 * handlers.
 *
 * @note Only Linux kernel releases which support clock event devices
 * (CONFIG_GENERIC_CLOCKEVENTS) would call this routine. Otherwise,
 * host tick mode is always periodic, and does not involve any
 * callback mechanism from the Linux kernel.
 *
 * Rescheduling: never.
 */

static void xnarch_switch_htick_mode(enum clock_event_mode mode, struct clock_event_device *cdev)
{
	xnsched_t *sched;
	xnticks_t tickval;
	spl_t s;

#ifndef __IPIPE_FEATURE_REQUEST_TICKDEV
	struct ipipe_tick_device *tdev = (struct ipipe_tick_device *)cdev;
	cdev = tdev->slave->evtdev;
#endif
	rthal_timer_notify_switch(mode, cdev);

	if (mode == CLOCK_EVT_MODE_ONESHOT)
		return;

	xnlock_get_irqsave(&nklock, s);

	sched = xnpod_current_sched();

	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		tickval = 1000000000UL / HZ;
		xntimer_start(&sched->htimer, tickval, tickval, XN_RELATIVE);
		break;

	case CLOCK_EVT_MODE_SHUTDOWN:
		xntimer_stop(&sched->htimer);
		break;

	default:
#if XENO_DEBUG(TIMERS)
		xnlogerr("host tick: invalid mode `%d'?\n", mode);
#endif
		;
	}

	xnlock_put_irqrestore(&nklock, s);
}

#endif /* CONFIG_GENERIC_CLOCKEVENTS */

#ifdef CONFIG_SMP

static inline int xnarch_hook_ipi (void (*handler)(void))
{
    return rthal_virtualize_irq(&rthal_domain,
				RTHAL_RESCHEDULE_IPI,
				(rthal_irq_handler_t) handler,
				NULL,
				NULL,
				IPIPE_HANDLE_MASK | IPIPE_WIRED_MASK);
}

static inline int xnarch_release_ipi (void)
{
    return rthal_virtualize_irq(&rthal_domain,
				RTHAL_RESCHEDULE_IPI,
				NULL,
				NULL,
				NULL,
				IPIPE_PASS_MASK);
}

static inline void xnarch_notify_halt(void)
{
    rthal_release_control();
}

#else /* !CONFIG_SMP */

static inline int xnarch_hook_ipi (void (*handler)(void))
{
    return 0;
}

static inline int xnarch_release_ipi (void)
{
    return 0;
}

#define xnarch_notify_halt()  rthal_release_control()

#endif /* CONFIG_SMP */

static inline void xnarch_notify_shutdown(void)
{
#if defined(CONFIG_SMP) && defined(MODULE)
	/* Make sure the shutdown sequence is kept on the same CPU
	   when running as a module. */
	set_cpus_allowed(current, cpumask_of_cpu(0));
#endif /* CONFIG_SMP && MODULE */
#ifdef CONFIG_XENO_OPT_PERVASIVE
    xnshadow_release_events();
#endif /* CONFIG_XENO_OPT_PERVASIVE */
    /* Wait for the currently processed events to drain. */
    set_current_state(TASK_UNINTERRUPTIBLE);
    schedule_timeout(50);
    xnarch_release_ipi();
}

static void xnarch_notify_ready (void)
{
    rthal_grab_control();
#ifdef CONFIG_XENO_OPT_PERVASIVE
    xnshadow_grab_events();
#endif /* CONFIG_XENO_OPT_PERVASIVE */
}

unsigned long long xnarch_get_host_time(void)
{
    struct timeval tv;
    do_gettimeofday(&tv);
    return tv.tv_sec * 1000000000ULL + tv.tv_usec * 1000;
}

EXPORT_SYMBOL_GPL(xnarch_get_host_time);

unsigned long long xnarch_get_cpu_time(void)
{
    return xnarch_tsc_to_ns(xnarch_get_cpu_tsc());
}

EXPORT_SYMBOL_GPL(xnarch_get_cpu_time);

#if defined(CONFIG_SMP) || XENO_DEBUG(XNLOCK)
void __xnlock_spin(xnlock_t *lock /*, */ XNLOCK_DBG_CONTEXT_ARGS)
{
	unsigned int spin_limit;
	int cpu = xnarch_current_cpu();

	xnlock_dbg_prepare_spin(&spin_limit);

	while (atomic_cmpxchg(&lock->owner, ~0, cpu) != ~0)
		do {
			cpu_relax();
			xnlock_dbg_spinning(lock, cpu, &spin_limit /*, */
					    XNLOCK_DBG_PASS_CONTEXT);
		} while(atomic_read(&lock->owner) != ~0);
}
EXPORT_SYMBOL_GPL(__xnlock_spin);
#endif /* CONFIG_SMP */

#endif /* !_XENO_ASM_GENERIC_BITS_POD_H */
