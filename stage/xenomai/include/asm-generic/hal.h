/**
 *   @ingroup hal
 *   @file
 *
 *   Generic Real-Time HAL.
 *   Copyright &copy; 2005 Philippe Gerum.
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, Inc., 675 Mass Ave, Cambridge MA 02139,
 *   USA; either version 2 of the License, or (at your option) any later
 *   version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

/**
 * @addtogroup hal
 *@{*/

#ifndef _XENO_ASM_GENERIC_HAL_H
#define _XENO_ASM_GENERIC_HAL_H

#ifndef __KERNEL__
#error "Pure kernel header included from user-space!"
#endif

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kallsyms.h>
#include <linux/init.h>
#include <asm/byteorder.h>
#include <asm/xenomai/wrappers.h>
#include <asm/xenomai/arith.h>

#define RTHAL_DOMAIN_ID		0x58454e4f

#define RTHAL_TIMER_FREQ	(rthal_tunables.timer_freq)
#define RTHAL_CLOCK_FREQ	(rthal_tunables.clock_freq)
#define RTHAL_CPU_FREQ		(rthal_tunables.cpu_freq)
#define RTHAL_NR_APCS		BITS_PER_LONG

#define RTHAL_EVENT_PROPAGATE   0
#define RTHAL_EVENT_STOP        1

#define RTHAL_NR_CPUS		IPIPE_NR_CPUS
#define RTHAL_NR_FAULTS		IPIPE_NR_FAULTS
#define RTHAL_NR_IRQS		IPIPE_NR_IRQS
#define RTHAL_VIRQ_BASE		IPIPE_VIRQ_BASE

#define rthal_virtual_irq_p(irq)	((irq) >= RTHAL_VIRQ_BASE && \
					(irq) < RTHAL_NR_IRQS)

#ifdef CONFIG_IPIPE_CORE
#define RTHAL_HRTIMER_IPI	IPIPE_HRTIMER_IPI
#define RTHAL_RESCHEDULE_IPI	IPIPE_RESCHEDULE_IPI
#define RTHAL_CRITICAL_IPI	IPIPE_CRITICAL_IPI
#define RTHAL_PERCPU_TIMER_IRQ(cpu) per_cpu(ipipe_percpu.hrtimer_irq, cpu)
#else /* !I-pipe core */
#define RTHAL_HRTIMER_IPI	IPIPE_SERVICE_IPI0
#define RTHAL_RESCHEDULE_IPI	IPIPE_SERVICE_IPI1
#define RTHAL_CRITICAL_IPI	IPIPE_CRITICAL_IPI
#define RTHAL_PERCPU_TIMER_IRQ(cpu) RTHAL_TIMER_IRQ
#endif /* !I-pipe core */

enum rthal_ktimer_mode { /* <!> Must follow enum clock_event_mode */
	KTIMER_MODE_UNUSED = 0,
	KTIMER_MODE_SHUTDOWN,
	KTIMER_MODE_PERIODIC,
	KTIMER_MODE_ONESHOT,
};

typedef struct ipipe_domain rthal_pipeline_stage_t;

#ifdef IPIPE_SPIN_LOCK_UNLOCKED
typedef ipipe_spinlock_t rthal_spinlock_t;
#define RTHAL_SPIN_LOCK_UNLOCKED IPIPE_SPIN_LOCK_UNLOCKED
#else /* !IPIPE_SPIN_LOCK_UNLOCKED */
#ifdef RAW_SPIN_LOCK_UNLOCKED
typedef raw_spinlock_t rthal_spinlock_t;
#define RTHAL_SPIN_LOCK_UNLOCKED RAW_SPIN_LOCK_UNLOCKED
#else /* !RAW_SPIN_LOCK_UNLOCKED */
typedef spinlock_t rthal_spinlock_t;
#define RTHAL_SPIN_LOCK_UNLOCKED SPIN_LOCK_UNLOCKED
#endif /* !RAW_SPIN_LOCK_UNLOCKED */
#endif /* !IPIPE_SPIN_LOCK_UNLOCKED */

#define rthal_irq_cookie(ipd,irq)	__ipipe_irq_cookie(ipd,irq)
#define rthal_irq_handler(ipd,irq)	__ipipe_irq_handler(ipd,irq)

#define rthal_cpudata_irq_hits(ipd,cpu,irq)	__ipipe_cpudata_irq_hits(ipd,cpu,irq)

#ifndef local_irq_save_hw_smp
#ifdef CONFIG_SMP
#define local_irq_save_hw_smp(flags)	local_irq_save_hw(flags)
#define local_irq_restore_hw_smp(flags)	local_irq_restore_hw(flags)
#else /* !CONFIG_SMP */
#define local_irq_save_hw_smp(flags)	do { (void)(flags); } while (0)
#define local_irq_restore_hw_smp(flags)	do { } while (0)
#endif /* !CONFIG_SMP */
#endif /* !local_irq_save_hw_smp */

/* I-pipe domain priorities and virtual interrupt mask handling. If
   the invariant pipeline head feature is enabled for Xenomai, use
   it. */
#define RTHAL_ROOT_PRIO			IPIPE_ROOT_PRIO
#ifdef CONFIG_XENO_OPT_PIPELINE_HEAD
#define RTHAL_XENO_PRIO			IPIPE_HEAD_PRIORITY
#define rthal_local_irq_disable()	ipipe_stall_pipeline_head()
#define rthal_local_irq_enable()	ipipe_unstall_pipeline_head()
#define rthal_local_irq_save(x)		((x) = ipipe_test_and_stall_pipeline_head() & 1)
#define rthal_local_irq_restore(x)	ipipe_restore_pipeline_head(x)
#else /* !CONFIG_XENO_OPT_PIPELINE_HEAD */
#define RTHAL_XENO_PRIO			(RTHAL_ROOT_PRIO + 100)
#define rthal_local_irq_disable()	ipipe_stall_pipeline_from(&rthal_domain)
#define rthal_local_irq_enable()	ipipe_unstall_pipeline_from(&rthal_domain)
#define rthal_local_irq_save(x)		((x) = ipipe_test_and_stall_pipeline_from(&rthal_domain) & 1)
#define rthal_local_irq_restore(x)	ipipe_restore_pipeline_from(&rthal_domain,(x))
#endif /* !CONFIG_XENO_OPT_PIPELINE_HEAD */
#define rthal_local_irq_flags(x)	((x) = ipipe_test_pipeline_from(&rthal_domain) & 1)
#define rthal_local_irq_test()		ipipe_test_pipeline_from(&rthal_domain)
#define rthal_local_irq_disabled()				\
({								\
	unsigned long __flags, __ret;				\
	local_irq_save_hw_smp(__flags);				\
	__ret = ipipe_test_pipeline_from(&rthal_domain);	\
	local_irq_restore_hw_smp(__flags);			\
	__ret;							\
})
#define rthal_stage_irq_enable(dom)	ipipe_unstall_pipeline_from(dom)
#define rthal_local_irq_save_hw(x)	local_irq_save_hw(x)
#define rthal_local_irq_restore_hw(x)	local_irq_restore_hw(x)
#define rthal_local_irq_enable_hw()	local_irq_enable_hw()
#define rthal_local_irq_disable_hw()	local_irq_disable_hw()
#define rthal_local_irq_flags_hw(x)	local_save_flags_hw(x)

#ifdef spin_lock_hw
#define rthal_spin_lock_init(lock)	spin_lock_init(lock)
#define rthal_spin_lock(lock)		spin_lock_hw(lock)
#define rthal_spin_unlock(lock)		spin_unlock_hw(lock)
#else /* !spin_lock_hw */
#define rthal_spin_lock_init(lock)	*(lock) = IPIPE_SPIN_LOCK_UNLOCKED
#define rthal_spin_lock(lock)		spin_lock(lock)
#define rthal_spin_unlock(lock)		spin_unlock(lock)
#endif /* !spin_lock_hw */

#define rthal_root_domain		ipipe_root_domain
#define rthal_current_domain		ipipe_current_domain

#define rthal_suspend_domain()		ipipe_suspend_domain()
#define rthal_grab_superlock(syncfn)	ipipe_critical_enter(syncfn)
#define rthal_release_superlock(x)	ipipe_critical_exit(x)
#define rthal_set_irq_affinity(irq,aff)	ipipe_set_irq_affinity(irq,aff)
#ifdef __IPIPE_FEATURE_FASTPEND_IRQ
/* We use the faster form assuming that hw IRQs are off at call site. */
#define rthal_schedule_irq_head(irq)	__ipipe_schedule_irq_head(irq)
#define rthal_schedule_irq_root(irq)	__ipipe_schedule_irq_root(irq)
#define rthal_propagate_irq(irq)	__ipipe_propagate_irq(irq)
#else /* !__IPIPE_FEATURE_FASTPEND_IRQ */
#define rthal_schedule_irq_head(irq)	ipipe_schedule_irq(irq)
#define rthal_schedule_irq_root(irq)	ipipe_schedule_irq(irq)
#define rthal_propagate_irq(irq)	ipipe_propagate_irq(irq)
#endif /* !__IPIPE_FEATURE_FASTPEND_IRQ */
#define rthal_virtualize_irq(dom,irq,isr,cookie,ackfn,mode) \
    ipipe_virtualize_irq(dom,irq,isr,cookie,ackfn,mode)
#define rthal_alloc_virq()		ipipe_alloc_virq()
#define rthal_free_virq(irq)		ipipe_free_virq(irq)
#define rthal_trigger_irq(irq)		ipipe_trigger_irq(irq)
#define rthal_get_sysinfo(ibuf)		ipipe_get_sysinfo(ibuf)
#define rthal_alloc_ptdkey()		ipipe_alloc_ptdkey()
#define rthal_free_ptdkey(key)		ipipe_free_ptdkey(key)
#define rthal_send_ipi(irq,cpus)	ipipe_send_ipi(irq,cpus)
#define rthal_lock_irq(dom,cpu,irq)	__ipipe_lock_irq(dom,cpu,irq)
#define rthal_unlock_irq(dom,irq)	__ipipe_unlock_irq(dom,irq)
#ifdef __IPIPE_FEATURE_PIC_MUTE
#define rthal_mute_pic()		ipipe_mute_pic()
#define rthal_unmute_pic()		ipipe_unmute_pic()
#else /* !__IPIPE_FEATURE_PIC_MUTE */
#define rthal_mute_pic()		do { } while(0)
#define rthal_unmute_pic()		do { } while(0)
#endif /* __IPIPE_FEATURE_PIC_MUTE */

#define rthal_processor_id()		ipipe_processor_id()

#define rthal_setsched_root(t,pol,prio)	ipipe_setscheduler_root(t,pol,prio)
#define rthal_reenter_root(t,pol,prio)	ipipe_reenter_root(t,pol,prio)
#define rthal_read_tsc(v)		ipipe_read_tsc(v)
#ifdef __IPIPE_FEATURE_ROOTPREEMPT_NOTIFIER
#define rthal_root_preempt_notify()	ipipe_root_preempt_notify()
#else /* !__IPIPE_FEATURE_ROOTPREEMPT_NOTIFIER */
#define rthal_root_preempt_notify()	do { } while (0)
#endif /* !__IPIPE_FEATURE_ROOTPREEMPT_NOTIFIER */

#define rthal_emergency_console()				\
	do {							\
		ipipe_stall_pipeline_from(ipipe_root_domain);	\
		ipipe_set_printk_sync(ipipe_current_domain);	\
	} while (0)

#ifdef __IPIPE_FEATURE_SYSINFO_V2

static inline unsigned long rthal_get_cpufreq(void)
{
	struct ipipe_sysinfo sysinfo;
	rthal_get_sysinfo(&sysinfo);
	return (unsigned long)sysinfo.sys_cpu_freq;
}

static inline unsigned long rthal_get_timerfreq(void)
{
	struct ipipe_sysinfo sysinfo;
	rthal_get_sysinfo(&sysinfo);
	return (unsigned long)sysinfo.sys_hrtimer_freq;
}

static inline unsigned long rthal_get_clockfreq(void)
{
	struct ipipe_sysinfo sysinfo;
	rthal_get_sysinfo(&sysinfo);
	return (unsigned long)sysinfo.sys_hrclock_freq;
}

#else /* !__IPIPE_FEATURE_SYSINFO_V2 */

static inline unsigned long rthal_get_cpufreq(void)
{
	struct ipipe_sysinfo sysinfo;
	rthal_get_sysinfo(&sysinfo);
	return (unsigned long)sysinfo.cpufreq;
}

static inline unsigned long rthal_get_timerfreq(void)
{
	struct ipipe_sysinfo sysinfo;
	rthal_get_sysinfo(&sysinfo);
	return (unsigned long)sysinfo.archdep.tmfreq;
}

static inline unsigned long rthal_get_clockfreq(void)
{
	return rthal_get_cpufreq();
}

#endif /* !__IPIPE_FEATURE_SYSINFO_V2 */

#define RTHAL_DECLARE_EVENT(hdlr)				       \
static int hdlr (unsigned event, struct ipipe_domain *ipd, void *data) \
{								       \
	return do_##hdlr(event,ipd,data);			       \
}

#define RTHAL_DECLARE_SCHEDULE_EVENT(hdlr)			       \
static int hdlr (unsigned event, struct ipipe_domain *ipd, void *data) \
{								       \
	struct task_struct *p = (struct task_struct *)data;	       \
	do_##hdlr(p);						       \
	return RTHAL_EVENT_PROPAGATE;				       \
}

#define RTHAL_DECLARE_SETSCHED_EVENT(hdlr)			       \
static int hdlr (unsigned event, struct ipipe_domain *ipd, void *data) \
{									\
	struct task_struct *p = (struct task_struct *)data;		\
	do_##hdlr(p, p->rt_priority);					\
	return RTHAL_EVENT_PROPAGATE;					\
}

#define RTHAL_DECLARE_SIGWAKE_EVENT(hdlr)			       \
static int hdlr (unsigned event, struct ipipe_domain *ipd, void *data) \
{								       \
	struct task_struct *p = (struct task_struct *)data;	       \
	do_##hdlr(p);						       \
	return RTHAL_EVENT_PROPAGATE;				       \
}

#define RTHAL_DECLARE_EXIT_EVENT(hdlr)				       \
static int hdlr (unsigned event, struct ipipe_domain *ipd, void *data) \
{								       \
	struct task_struct *p = (struct task_struct *)data;	       \
	do_##hdlr(p);						       \
	return RTHAL_EVENT_PROPAGATE;				       \
}

#define RTHAL_DECLARE_CLEANUP_EVENT(hdlr)			       \
static int hdlr (unsigned event, struct ipipe_domain *ipd, void *data) \
{								       \
	struct mm_struct *mm = (struct mm_struct *)data;	       \
	do_##hdlr(mm);						       \
	return RTHAL_EVENT_PROPAGATE;				       \
}

#define RTHAL_DECLARE_MAYDAY_EVENT(hdlr)			      \
static int hdlr(unsigned event, struct ipipe_domain *ipd, void *data) \
{								      \
	struct pt_regs *regs = data;				      \
	do_##hdlr(regs);					      \
	return RTHAL_EVENT_PROPAGATE;				      \
}

#define RTHAL_DECLARE_HOSTRT_EVENT(hdlr)			      \
static int hdlr(unsigned event, struct ipipe_domain *ipd, void *data) \
{								      \
	struct rthal_hostrt_data *hostrt = data;		      \
	do_##hdlr(hostrt);					      \
	return RTHAL_EVENT_PROPAGATE;				      \
}

#ifndef TASK_ATOMICSWITCH
#ifdef CONFIG_PREEMPT
/* We want this feature for preemptible kernels, or the behaviour when
   switching execution modes between Xenomai and Linux domains would
   be unreliable. */
#error "Adeos: atomic task switch support is missing; upgrading\n" \
       "     to a recent I-pipe version is required."
#endif /* CONFIG_PREEMPT */
/* I-pipe releases for 2.4 kernels don't have this task mode bit
   defined, so fake it. */
#define TASK_ATOMICSWITCH  0
#endif /* !TASK_ATOMICSWITCH */

static inline void set_task_nowakeup(struct task_struct *p)
{
	if (p->state & (TASK_INTERRUPTIBLE|TASK_UNINTERRUPTIBLE))
		set_task_state(p, p->state | TASK_NOWAKEUP);

}
static inline void clear_task_nowakeup(struct task_struct *p)
{
	set_task_state(p, p->state & ~TASK_NOWAKEUP);
}

#if defined(VM_PINNED) || defined(MMF_VM_PINNED)
#define rthal_disable_ondemand_mappings(tsk)   ipipe_disable_ondemand_mappings(tsk)
#else /* !(VM_PINNED || MMF_VM_PINNED) */
/* In case the I-pipe does not allow disabling ondemand mappings. */
#define rthal_disable_ondemand_mappings(tsk)   (0)
#endif	/* !(VM_PINNED || MMF_VM_PINNED) */

#define rthal_set_foreign_stack(ipd)	ipipe_set_foreign_stack(ipd)
#define rthal_clear_foreign_stack(ipd)	ipipe_clear_foreign_stack(ipd)

#ifdef __IPIPE_FEATURE_ENABLE_NOTIFIER
#define rthal_enable_notifier(p)	ipipe_enable_notifier(p)
#else
static inline void rthal_enable_notifier(struct task_struct *p)
{
	p->flags |= PF_EVNOTIFY;
}
#endif

#define rthal_catch_cleanup(hdlr)         \
    ipipe_catch_event(ipipe_root_domain,IPIPE_EVENT_CLEANUP,hdlr)
#define rthal_catch_taskexit(hdlr)	\
    ipipe_catch_event(ipipe_root_domain,IPIPE_EVENT_EXIT,hdlr)
#define rthal_catch_sigwake(hdlr)	\
    ipipe_catch_event(ipipe_root_domain,IPIPE_EVENT_SIGWAKE,hdlr)
#define rthal_catch_schedule(hdlr)	\
    ipipe_catch_event(ipipe_root_domain,IPIPE_EVENT_SCHEDULE,hdlr)
#define rthal_catch_setsched(hdlr)	\
    ipipe_catch_event(&rthal_domain,IPIPE_EVENT_SETSCHED,hdlr)
#define rthal_catch_losyscall(hdlr)	\
    ipipe_catch_event(ipipe_root_domain,IPIPE_EVENT_SYSCALL,hdlr)
#define rthal_catch_hisyscall(hdlr)	\
    ipipe_catch_event(&rthal_domain,IPIPE_EVENT_SYSCALL,hdlr)
#define rthal_catch_exception(ex,hdlr)	\
    ipipe_catch_event(&rthal_domain,ex|IPIPE_EVENT_SELF,hdlr)
#define rthal_catch_hostrt(hdlr)	\
    ipipe_catch_event(ipipe_root_domain, IPIPE_EVENT_HOSTRT, hdlr)

#ifdef IPIPE_EVENT_RETURN
#define RTHAL_HAVE_RETURN_EVENT
#define rthal_catch_return(hdlr)         \
    ipipe_catch_event(&rthal_domain,IPIPE_EVENT_RETURN,hdlr)
#define rthal_return_intercept(p)	ipipe_return_notify(p)
#else
#define rthal_catch_return(hdlr)	do { } while(0)
#endif

#define rthal_register_domain(_dom,_name,_id,_prio,_entry)	\
({								\
	struct ipipe_domain_attr attr;				\
	ipipe_init_attr(&attr);					\
	attr.name = _name;					\
	attr.entry = _entry;					\
	attr.domid = _id;					\
	attr.priority = _prio;					\
	ipipe_register_domain(_dom,&attr);			\
})

#define rthal_unregister_domain(dom)	ipipe_unregister_domain(dom)

#define RTHAL_DECLARE_DOMAIN(entry)		\
	void entry (void)			\
	{					\
		do_##entry();			\
	}

extern void rthal_domain_entry(void);

#define rthal_spin_lock_irq(lock)		\
	do {					\
		rthal_local_irq_disable();	\
		rthal_spin_lock(lock);		\
	} while(0)

#define rthal_spin_unlock_irq(lock)		\
	do {					\
		rthal_spin_unlock(lock);	\
		rthal_local_irq_enable();	\
	} while(0)

#define rthal_spin_lock_irqsave(lock,x)		\
	do {					\
		rthal_local_irq_save(x);	\
		rthal_spin_lock(lock);		\
	} while(0)

#define rthal_spin_unlock_irqrestore(lock,x)	\
	do {					\
		rthal_spin_unlock(lock);	\
		rthal_local_irq_restore(x);	\
	} while(0)

#define rthal_printk	printk

typedef ipipe_irq_handler_t rthal_irq_handler_t;
typedef ipipe_irq_ackfn_t   rthal_irq_ackfn_t;

struct rthal_calibration_data {
	unsigned long cpu_freq;
	unsigned long timer_freq;
	unsigned long clock_freq;
};

struct rthal_apc_desc {
	void (*handler)(void *cookie);
	void *cookie;
	const char *name;
	unsigned long hits[RTHAL_NR_CPUS];
};

typedef int (*rthal_trap_handler_t)(unsigned trapno,
				    rthal_pipeline_stage_t *stage,
				    void *data);

extern unsigned long rthal_cpufreq_arg;

extern unsigned long rthal_timerfreq_arg;

extern unsigned long rthal_clockfreq_arg;

extern rthal_pipeline_stage_t rthal_domain;

extern struct rthal_calibration_data rthal_tunables;

extern volatile int rthal_sync_op;

extern rthal_trap_handler_t rthal_trap_handler;

extern unsigned int rthal_realtime_faults[RTHAL_NR_CPUS][RTHAL_NR_FAULTS];

extern unsigned long rthal_apc_map;

extern struct rthal_apc_desc rthal_apc_table[RTHAL_NR_APCS];

extern unsigned long rthal_apc_pending[RTHAL_NR_CPUS];

extern unsigned int rthal_apc_virq;

extern unsigned long rthal_apc_pending[RTHAL_NR_CPUS];

extern unsigned int rthal_apc_virq;

extern int rthal_arch_init(void);

extern void rthal_arch_cleanup(void);

    /* Private interface -- Internal use only */

unsigned long rthal_critical_enter(void (*synch)(void));

void rthal_critical_exit(unsigned long flags);

    /* Public interface */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

int rthal_init(void);

void rthal_exit(void);

int rthal_irq_request(unsigned irq,
		      rthal_irq_handler_t handler,
		      rthal_irq_ackfn_t ackfn,
		      void *cookie);

int rthal_irq_release(unsigned irq);

int rthal_irq_enable(unsigned irq);

int rthal_irq_disable(unsigned irq);

int rthal_irq_end(unsigned irq);

int rthal_irq_host_request(unsigned irq,
			   rthal_irq_host_handler_t handler,
			   char *name,
			   void *dev_id);

int rthal_irq_host_release(unsigned irq,
			   void *dev_id);

static inline void rthal_irq_host_pend(unsigned irq)
{
	rthal_propagate_irq(irq);
}

int rthal_apc_alloc(const char *name,
		    void (*handler)(void *cookie),
		    void *cookie);

void rthal_apc_free(int apc);

static inline void __rthal_apc_schedule(int apc)
{
	int cpu = rthal_processor_id();
	if (!__test_and_set_bit(apc, &rthal_apc_pending[cpu]))
		rthal_schedule_irq_root(rthal_apc_virq);
}

static inline void rthal_apc_schedule(int apc)
{
	unsigned long flags;

	rthal_local_irq_save(flags);
	__rthal_apc_schedule(apc);
	rthal_local_irq_restore(flags);
}

rthal_trap_handler_t rthal_trap_catch(rthal_trap_handler_t handler);

unsigned long rthal_timer_calibrate(void);

#ifdef CONFIG_GENERIC_CLOCKEVENTS

enum clock_event_mode;
struct clock_event_device;

int rthal_timer_request(void (*tick_handler)(void),
			void (*mode_emul)(enum clock_event_mode mode, struct clock_event_device *cdev),
			int (*tick_emul) (unsigned long delay, struct clock_event_device *cdev),
			int cpu);

void rthal_timer_notify_switch(enum clock_event_mode mode,
			       struct clock_event_device *cdev);

#else
int rthal_timer_request(void (*tick_handler)(void),
			int cpu);
#endif

void rthal_timer_release(int cpu);

#ifdef CONFIG_SMP
extern cpumask_t rthal_supported_cpus;

static inline int rthal_cpu_supported(int cpu)
{
	return cpu_isset(cpu, rthal_supported_cpus);
}
#else  /* !CONFIG_SMP */
#define rthal_supported_cpus CPU_MASK_ALL

static inline int rthal_cpu_supported(int cpu)
{
	return 1;
}
#endif /* !CONFIG_SMP */

#ifdef CONFIG_IPIPE_TRACE

#include <linux/ipipe_trace.h>

static inline int rthal_trace_max_begin(unsigned long v)
{
	ipipe_trace_begin(v);
	return 0;
}

static inline int rthal_trace_max_end(unsigned long v)
{
	ipipe_trace_end(v);
	return 0;
}

static inline int rthal_trace_max_reset(void)
{
	ipipe_trace_max_reset();
	return 0;
}

static inline int rthal_trace_user_start(void)
{
	return ipipe_trace_frozen_reset();
}

static inline int rthal_trace_user_stop(unsigned long v)
{
	ipipe_trace_freeze(v);
	return 0;
}

static inline int rthal_trace_user_freeze(unsigned long v, int once)
{
	int err = 0;

	if (!once)
		err = ipipe_trace_frozen_reset();
	ipipe_trace_freeze(v);
	return err;
}

static inline int rthal_trace_special(unsigned char id, unsigned long v)
{
	ipipe_trace_special(id, v);
	return 0;
}

static inline int rthal_trace_special_u64(unsigned char id,
					  unsigned long long v)
{
	ipipe_trace_special(id, (unsigned long)(v >> 32));
	ipipe_trace_special(id, (unsigned long)(v & 0xFFFFFFFF));
	return 0;
}

static inline int rthal_trace_pid(pid_t pid, short prio)
{
	ipipe_trace_pid(pid, prio);
	return 0;
}

static inline int rthal_trace_tick(unsigned long delay_tsc)
{
#ifdef __IPIPE_FEATURE_TRACE_EVENT
	ipipe_trace_event(0, delay_tsc);
	return 0;
#else
	return -ENOSYS;
#endif
}

static inline int rthal_trace_panic_freeze(void)
{
	ipipe_trace_panic_freeze();
	return 0;
}

static inline int rthal_trace_panic_dump(void)
{
	ipipe_trace_panic_dump();
	return 0;
}

#else /* !CONFIG_IPIPE_TRACE */

#define rthal_trace_max_begin(v)		({int err = -ENOSYS; err; })
#define rthal_trace_max_end(v)			({int err = -ENOSYS; err; })
#define rthal_trace_max_reset(v)		({int err = -ENOSYS; err; })
#define rthal_trace_user_start()		({int err = -ENOSYS; err; })
#define rthal_trace_user_stop(v)		({int err = -ENOSYS; err; })
#define rthal_trace_user_freeze(v, once)	({int err = -ENOSYS; err; })
#define rthal_trace_special(id, v)		({int err = -ENOSYS; err; })
#define rthal_trace_special_u64(id, v)		({int err = -ENOSYS; err; })
#define rthal_trace_pid(pid, prio)		({int err = -ENOSYS; err; })
#define rthal_trace_tick(delay_tsc)		({int err = -ENOSYS; err; })
#define rthal_trace_panic_freeze()		({int err = -ENOSYS; err; })
#define rthal_trace_panic_dump()		({int err = -ENOSYS; err; })

#endif /* CONFIG_IPIPE_TRACE */

#define rthal_hostrt_data	ipipe_hostrt_data
#ifdef __cplusplus
}
#endif /* __cplusplus */

/*@}*/

#endif /* !_XENO_ASM_GENERIC_HAL_H */
