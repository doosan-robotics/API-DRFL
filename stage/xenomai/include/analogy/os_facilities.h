/**
 * @file
 * Analogy for Linux, Operation system facilities
 *
 * Copyright (C) 1997-2000 David A. Schleef <ds@schleef.org>
 * Copyright (C) 2008 Alexis Berlemont <alexis.berlemont@free.fr>
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
 * along with Xenomai; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __ANALOGY_OS_FACILITIES__
#define __ANALOGY_OS_FACILITIES__

#if defined(__KERNEL__) && !defined(DOXYGEN_CPP)

#include <linux/fs.h>
#include <linux/spinlock.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h>

#if !(defined(CONFIG_XENO_SKIN_RTDM) || \
      defined(CONFIG_XENO_SKIN_RTDM_MODULE))
#error Analogy needs RTDM enabled \
    (statically or as amodule) to compile properly
#endif /* !(CONFIG_XENO_SKIN_RTDM ||
	  CONFIG_XENO_SKIN_RTDM_MODULE */

#include <rtdm/rtdm_driver.h>

/* --- Kernel traces functions --- */

#define A4L_PROMPT "Analogy: "

#define RTDM_SUBCLASS_ANALOGY 0

#define __a4l_err(fmt, args...) \
	rtdm_printk(KERN_ERR A4L_PROMPT fmt, ##args)

#define __a4l_warn(fmt, args...) \
	rtdm_printk(KERN_WARNING A4L_PROMPT fmt, ##args)

#define __a4l_info(fmt, args...) \
	rtdm_printk(KERN_INFO A4L_PROMPT fmt, ##args)

#ifdef CONFIG_XENO_DRIVERS_ANALOGY_DEBUG

#define __a4l_dbg(level, debug, fmt, args...)			\
	do {							\
	if ((debug) >= (level))					\
		rtdm_printk(KERN_DEBUG A4L_PROMPT fmt, ##args); \
	} while (0)

#define core_dbg CONFIG_XENO_DRIVERS_ANALOGY_DEBUG_LEVEL
#define drv_dbg CONFIG_XENO_DRIVERS_ANALOGY_DRIVER_DEBUG_LEVEL

#else /* !CONFIG_XENO_DRIVERS_ANALOGY_DEBUG */

#define __a4l_dbg(level, debug, fmt, args...)

#endif /* CONFIG_XENO_DRIVERS_ANALOGY_DEBUG */

#define __a4l_dev_name(dev) \
	(dev->driver == NULL) ? "unattached dev" : dev->driver->board_name

#define a4l_err(dev, fmt, args...) \
	__a4l_err("%s: " fmt, __a4l_dev_name(dev), ##args)

#define a4l_warn(dev, fmt, args...) \
	__a4l_warn("%s: " fmt, __a4l_dev_name(dev), ##args)

#define a4l_info(dev, fmt, args...) \
	__a4l_info("%s: " fmt, __a4l_dev_name(dev), ##args)

#define a4l_dbg(level, debug, dev, fmt, args...)			\
	__a4l_dbg(level, debug, "%s: " fmt, __a4l_dev_name(dev), ##args)

/* --- Spinlock section --- */

typedef rtdm_lock_t a4l_lock_t;

#define A4L_LOCK_UNLOCKED RTDM_LOCK_UNLOCKED

#define a4l_lock_init(lock) rtdm_lock_init(lock)
#define a4l_lock(lock) rtdm_lock_get(lock)
#define a4l_unlock(lock) rtdm_lock_put(lock)
#define a4l_lock_irqsave(lock, context)	\
    rtdm_lock_get_irqsave(lock, context)
#define a4l_unlock_irqrestore(lock, context) \
    rtdm_lock_put_irqrestore(lock, context)

/* --- Task section --- */

#define A4L_TASK_LOWEST_PRIORITY RTDM_TASK_LOWEST_PRIORITY
#define A4L_TASK_HIGHEST_PRIORITY RTDM_TASK_HIGHEST_PRIORITY

typedef rtdm_task_t a4l_task_t;
typedef rtdm_task_proc_t a4l_task_proc_t;

#define a4l_task_init(tsk, name, proc, arg, priority) \
    rtdm_task_init(tsk, name, proc, arg, priority, 0)

#define a4l_task_destroy(tsk) rtdm_task_destroy(tsk)

#define a4l_task_sleep(delay) rtdm_task_sleep(delay)

/* --- Time section --- */

static inline void a4l_udelay(unsigned int us)
{
	rtdm_task_busy_sleep(((nanosecs_rel_t) us) * 1000);
}

static inline nanosecs_abs_t a4l_get_rawtime(void)
{
	return rtdm_clock_read();
}

/* Function which gives absolute time */
nanosecs_abs_t a4l_get_time(void);

/* Function for setting up the absolute time recovery */
void a4l_init_time(void);

/* --- IRQ section --- */

#define A4L_IRQ_SHARED RTDM_IRQTYPE_SHARED
#define A4L_IRQ_EDGE RTDM_IRQTYPE_EDGE
#define A4L_IRQ_DISABLED 0

typedef int (*a4l_irq_hdlr_t) (unsigned int irq, void *d);

struct a4l_irq_descriptor {
	/* These fields are useful to launch the IRQ trampoline;
	   that is the reason why a structure has been defined */
	a4l_irq_hdlr_t handler;
	unsigned int irq;
	void *cookie;
	rtdm_irq_t rtdm_desc;
};
typedef struct a4l_irq_descriptor a4l_irq_desc_t;

int __a4l_request_irq(a4l_irq_desc_t * dsc,
		      unsigned int irq,
		      a4l_irq_hdlr_t handler,
		      unsigned long flags, void *cookie);
int __a4l_free_irq(a4l_irq_desc_t * dsc);

/* --- Synchronization section --- */

#define __NRT_WAITER 1
#define __RT_WAITER 2
#define __EVT_PDING 3

struct a4l_sync {
	unsigned long status;
	rtdm_event_t rtdm_evt;
	rtdm_nrtsig_t nrt_sig;
	wait_queue_head_t wq;
};
typedef struct a4l_sync a4l_sync_t;

#define a4l_select_sync(snc, slr, type, fd) \
	rtdm_event_select_bind(&((snc)->rtdm_evt), slr, type, fd)

int a4l_init_sync(a4l_sync_t * snc);
void a4l_cleanup_sync(a4l_sync_t * snc);
void a4l_flush_sync(a4l_sync_t * snc);
int a4l_wait_sync(a4l_sync_t * snc, int rt);
int a4l_timedwait_sync(a4l_sync_t * snc,
		       int rt, unsigned long long ns_timeout);
void a4l_signal_sync(a4l_sync_t * snc);

/* --- Misc section --- */

#define a4l_test_rt() rtdm_in_rt_context()

#endif /* __KERNEL__ && !DOXYGEN_CPP */

#endif /* __ANALOGY_OS_FACILITIES__ */
