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

#ifndef _XENO_ASM_GENERIC_SYSTEM_H
#define _XENO_ASM_GENERIC_SYSTEM_H

#ifndef __KERNEL__
#error "Pure kernel header included from user-space!"
#endif

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <asm/uaccess.h>
#include <asm/param.h>
#include <asm/mmu_context.h>
#include <asm/ptrace.h>
#include <asm/xenomai/hal.h>
#include <asm/xenomai/atomic.h>
#include <asm-generic/xenomai/timeconv.h>
#include <nucleus/shadow.h>

/* debug support */
#include <nucleus/assert.h>

#ifndef CONFIG_XENO_OPT_DEBUG_XNLOCK
#define CONFIG_XENO_OPT_DEBUG_XNLOCK 0
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Time base export */
#define xnarch_declare_tbase(base)		do { } while(0)

/* Tracer interface */
#define xnarch_trace_max_begin(v)		rthal_trace_max_begin(v)
#define xnarch_trace_max_end(v)			rthal_trace_max_end(v)
#define xnarch_trace_max_reset()		rthal_trace_max_reset()
#define xnarch_trace_user_start()		rthal_trace_user_start()
#define xnarch_trace_user_stop(v)		rthal_trace_user_stop(v)
#define xnarch_trace_user_freeze(v, once)	rthal_trace_user_freeze(v, once)
#define xnarch_trace_special(id, v)		rthal_trace_special(id, v)
#define xnarch_trace_special_u64(id, v)		rthal_trace_special_u64(id, v)
#define xnarch_trace_pid(pid, prio)		rthal_trace_pid(pid, prio)
#define xnarch_trace_tick(delay_tsc)		rthal_trace_tick(delay_tsc)
#define xnarch_trace_panic_freeze()		rthal_trace_panic_freeze()
#define xnarch_trace_panic_dump()		rthal_trace_panic_dump()

#ifndef xnarch_fault_um
#define xnarch_fault_um(fi) user_mode(fi->regs)
#endif

#define module_param_value(parm) (parm)

typedef unsigned long spl_t;

#define splhigh(x)  rthal_local_irq_save(x)
#ifdef CONFIG_SMP
#define splexit(x)  rthal_local_irq_restore((x) & 1)
#else /* !CONFIG_SMP */
#define splexit(x)  rthal_local_irq_restore(x)
#endif /* !CONFIG_SMP */
#define splmax()    rthal_local_irq_disable()
#define splnone()   rthal_local_irq_enable()
#define spltest()   rthal_local_irq_test()
#define splget(x)   rthal_local_irq_flags(x)

static inline unsigned xnarch_current_cpu(void)
{
	return rthal_processor_id();
}

#ifndef xnarch_mb_before_unlock
#define xnarch_mb_before_unlock() xnarch_memory_barrier()
#endif

#ifndef xnarch_mb_after_unlock
#define xnarch_mb_after_unlock() xnarch_memory_barrier()
#endif

#if XENO_DEBUG(XNLOCK)

typedef struct {

	unsigned long long spin_time;
	unsigned long long lock_time;
	const char *file;
	const char *function;
	unsigned line;

} xnlockinfo_t;

typedef struct {

	atomic_t owner;
	const char *file;
	const char *function;
	unsigned line;
	int cpu;
	unsigned long long spin_time;
	unsigned long long lock_date;

} xnlock_t;

#define XNARCH_LOCK_UNLOCKED (xnlock_t) {	\
	{ ~0 },					\
	NULL,					\
	NULL,					\
	0,					\
	-1,					\
	0LL,					\
	0LL,					\
}

#define XNLOCK_DBG_CONTEXT		, __FILE__, __LINE__, __FUNCTION__
#define XNLOCK_DBG_CONTEXT_ARGS \
	, const char *file, int line, const char *function
#define XNLOCK_DBG_PASS_CONTEXT		, file, line, function
#define XNLOCK_DBG_MAX_SPINS		10000000

static inline void xnlock_dbg_prepare_acquire(unsigned long long *start)
{
	*start = rthal_rdtsc();
}

static inline void xnlock_dbg_prepare_spin(unsigned *spin_limit)
{
	*spin_limit = XNLOCK_DBG_MAX_SPINS;
}

static inline void
xnlock_dbg_spinning(xnlock_t *lock, int cpu, unsigned int *spin_limit,
		    const char *file, int line, const char *function)
{
	if (--*spin_limit == 0) {
		rthal_emergency_console();
		printk(KERN_ERR "Xenomai: stuck on nucleus lock %p\n"
				"	  waiter = %s:%u (%s(), CPU #%d)\n"
				"	  owner	 = %s:%u (%s(), CPU #%d)\n",
		       lock, file, line, function, cpu,
		       lock->file, lock->line, lock->function, lock->cpu);
		show_stack(NULL, NULL);
#ifndef CONFIG_SMP
		BUG();
#endif
	}
}

static inline void
xnlock_dbg_acquired(xnlock_t *lock, int cpu, unsigned long long *start,
		    const char *file, int line, const char *function)
{
	lock->lock_date = *start;
	lock->spin_time = rthal_rdtsc() - *start;
	lock->file = file;
	lock->function = function;
	lock->line = line;
	lock->cpu = cpu;
}

static inline int
xnlock_dbg_release(xnlock_t *lock,
		const char *file, int line, const char *function)
{
	extern xnlockinfo_t xnlock_stats[];
	unsigned long long lock_time = rthal_rdtsc() - lock->lock_date;
	int cpu = xnarch_current_cpu();
	xnlockinfo_t *stats = &xnlock_stats[cpu];

	if (unlikely(atomic_read(&lock->owner) != cpu)) {
		rthal_emergency_console();
		printk(KERN_ERR "Xenomai: %s:%u (%s()): unlocking unlocked nucleus lock %p"
				" on CPU #%d\n"
				"         owner  = %s:%u (%s(), CPU #%d)\n",
			file, line, function,
		       lock, cpu, lock->file, lock->line, lock->function,
		       lock->cpu);
		show_stack(NULL,NULL);
		return 1;
	}

	lock->cpu = -lock->cpu;	/* File that we released it. */

	if (lock_time > stats->lock_time) {
		stats->lock_time = lock_time;
		stats->spin_time = lock->spin_time;
		stats->file = lock->file;
		stats->function = lock->function;
		stats->line = lock->line;
	}
	return 0;
}

#else /* !XENO_DEBUG(XNLOCK) */

typedef struct { atomic_t owner; } xnlock_t;

#define XNARCH_LOCK_UNLOCKED		(xnlock_t) { { ~0 } }

#define XNLOCK_DBG_CONTEXT
#define XNLOCK_DBG_CONTEXT_ARGS
#define XNLOCK_DBG_PASS_CONTEXT

static inline void xnlock_dbg_prepare_acquire(unsigned long long *start) { }
static inline void xnlock_dbg_prepare_spin(unsigned *spin_limit)	 { }

static inline void
xnlock_dbg_spinning(xnlock_t *lock, int cpu, unsigned int *spin_limit)	 { }

static inline void
xnlock_dbg_acquired(xnlock_t *lock, int cpu, unsigned long long *start)	 { }

static inline int xnlock_dbg_release(xnlock_t *lock)
{
	return 0;
}

#endif /* !XENO_DEBUG(XNLOCK) */

#define XNARCH_NR_CPUS			RTHAL_NR_CPUS

#define XNARCH_NR_IRQS			RTHAL_NR_IRQS
#define XNARCH_TIMER_IRQ		RTHAL_TIMER_IRQ
#define XNARCH_PERCPU_TIMER_IRQ(cpu)	RTHAL_PERCPU_TIMER_IRQ(cpu)
#define XNARCH_TIMER_DEVICE		RTHAL_TIMER_DEVICE
#define XNARCH_CLOCK_DEVICE		RTHAL_CLOCK_DEVICE

#define XNARCH_PROMPT "Xenomai: "
#define xnarch_loginfo(fmt, args...)	printk(KERN_INFO XNARCH_PROMPT fmt, ##args)
#define xnarch_logwarn(fmt, args...)	printk(KERN_WARNING XNARCH_PROMPT fmt, ##args)
#define xnarch_logerr(fmt, args...)	printk(KERN_ERR XNARCH_PROMPT fmt, ##args)
#define xnarch_logerr_noprompt(fmt, args...)	printk(KERN_ERR fmt, ##args)
#define xnarch_printf(fmt, args...)	printk(KERN_INFO XNARCH_PROMPT fmt, ##args)

#ifndef RTHAL_SHARED_HEAP_FLAGS
#define XNARCH_SHARED_HEAP_FLAGS 0
#else /* !RTHAL_SHARED_HEAP_FLAGS */
#define XNARCH_SHARED_HEAP_FLAGS RTHAL_SHARED_HEAP_FLAGS
#endif /* !RTHAL_SHARED_HEAP_FLAGS */

typedef cpumask_t xnarch_cpumask_t;

#ifdef CONFIG_SMP
#define xnarch_cpu_online_map			(*cpu_online_mask)
#else
#define xnarch_cpu_online_map			cpumask_of_cpu(0)
#endif
#define xnarch_num_online_cpus()		num_online_cpus()
#define xnarch_cpu_set(cpu, mask)		cpu_set(cpu, (mask))
#define xnarch_cpu_clear(cpu, mask)		cpu_clear(cpu, (mask))
#define xnarch_cpus_clear(mask)			cpus_clear(mask)
#define xnarch_cpu_isset(cpu, mask)		cpu_isset(cpu, (mask))
#define xnarch_cpus_and(dst, src1, src2)	cpus_and((dst), (src1), (src2))
#define xnarch_cpus_equal(mask1, mask2)		cpus_equal((mask1), (mask2))
#define xnarch_cpus_empty(mask)			cpus_empty(mask)
#define xnarch_cpumask_of_cpu(cpu)		cpumask_of_cpu(cpu)
#define xnarch_cpu_test_and_set(cpu, mask)	cpu_test_and_set(cpu, (mask))
#define xnarch_first_cpu(mask)			first_cpu(mask)
#define XNARCH_CPU_MASK_ALL			CPU_MASK_ALL

#define xnarch_supported_cpus			rthal_supported_cpus
#define xnarch_cpu_supported(cpu)		rthal_cpu_supported(cpu)

struct xnheap;

typedef struct xnarch_heapcb {

	unsigned long numaps;	/* # of active user-space mappings. */
	int kmflags;		/* Kernel memory flags (0 if vmalloc()). */
	void *heapbase;		/* Shared heap memory base. */
	void (*release)(struct xnheap *heap); /* callback upon last unmap */

} xnarch_heapcb_t;

unsigned long long xnarch_get_host_time(void);

unsigned long long xnarch_get_cpu_time(void);

static inline unsigned long long xnarch_get_cpu_freq(void)
{
	return RTHAL_CPU_FREQ;
}

static inline unsigned long long xnarch_get_clock_freq(void)
{
	return RTHAL_CLOCK_FREQ;
}

#define xnarch_get_cpu_tsc			rthal_rdtsc

static inline void xnarch_begin_panic(void)
{
	xnarch_trace_panic_freeze();
	rthal_emergency_console();
}

#define xnarch_halt()					\
	do {						\
		show_stack(NULL,NULL);			\
		xnarch_trace_panic_dump();		\
		for (;;)				\
			cpu_relax();			\
	} while(0)

static inline int xnarch_setimask (int imask)
{
	spl_t s;

	splhigh(s);
	splexit(!!imask);
	return !!s;
}

static inline int xnarch_root_domain_p(void)
{
	return rthal_current_domain == rthal_root_domain;
}

#if defined(CONFIG_SMP) || XENO_DEBUG(XNLOCK)

#define xnlock_get(lock)		__xnlock_get(lock  XNLOCK_DBG_CONTEXT)
#define xnlock_get_irqsave(lock,x) \
	((x) = __xnlock_get_irqsave(lock  XNLOCK_DBG_CONTEXT))
#define xnlock_clear_irqoff(lock)	xnlock_put_irqrestore(lock, 1)
#define xnlock_clear_irqon(lock)	xnlock_put_irqrestore(lock, 0)

static inline void xnlock_init (xnlock_t *lock)
{
	*lock = XNARCH_LOCK_UNLOCKED;
}

#define DECLARE_XNLOCK(lock)		xnlock_t lock
#define DECLARE_EXTERN_XNLOCK(lock)	extern xnlock_t lock
#define DEFINE_XNLOCK(lock)		xnlock_t lock = XNARCH_LOCK_UNLOCKED
#define DEFINE_PRIVATE_XNLOCK(lock)	static DEFINE_XNLOCK(lock)

void __xnlock_spin(xnlock_t *lock /*, */ XNLOCK_DBG_CONTEXT_ARGS);

static inline int __xnlock_get(xnlock_t *lock /*, */ XNLOCK_DBG_CONTEXT_ARGS)
{
	unsigned long long start;
	int cpu = xnarch_current_cpu();

	if (atomic_read(&lock->owner) == cpu)
		return 1;

	xnlock_dbg_prepare_acquire(&start);

	if (unlikely(atomic_cmpxchg(&lock->owner, ~0, cpu) != ~0))
		__xnlock_spin(lock /*, */ XNLOCK_DBG_PASS_CONTEXT);

	xnlock_dbg_acquired(lock, cpu, &start /*, */ XNLOCK_DBG_PASS_CONTEXT);

	return 0;
}

static inline void __xnlock_put(xnlock_t *lock /*, */ XNLOCK_DBG_CONTEXT_ARGS)
{
	if (xnlock_dbg_release(lock /*, */ XNLOCK_DBG_PASS_CONTEXT))
		return;

	/*
	 * Make sure all data written inside the lock is visible to
	 * other CPUs before we release the lock.
	 */
	xnarch_mb_before_unlock();

	atomic_set(&lock->owner, ~0);

	xnarch_mb_after_unlock();
}
#define xnlock_put(lock) \
	__xnlock_put((lock) /*, */ XNLOCK_DBG_CONTEXT)

static inline spl_t
__xnlock_get_irqsave(xnlock_t *lock /*, */ XNLOCK_DBG_CONTEXT_ARGS)
{
	unsigned long flags;

	rthal_local_irq_save(flags);

	if (__xnlock_get(lock /*, */ XNLOCK_DBG_PASS_CONTEXT))
		flags |= 2;	/* Recursive acquisition */

	return flags;
}

static inline void
__xnlock_put_irqrestore(xnlock_t *lock, spl_t flags /*, */ XNLOCK_DBG_CONTEXT_ARGS)
{
	/* Only release the lock if we didn't take it recursively. */
	if (!(flags & 2))
		__xnlock_put(lock /*, */ XNLOCK_DBG_PASS_CONTEXT);

	rthal_local_irq_restore(flags & 1);
}
#define xnlock_put_irqrestore(lock, flags) \
	__xnlock_put_irqrestore((lock), (flags) /*, */ XNLOCK_DBG_CONTEXT)

static inline void xnarch_send_ipi(xnarch_cpumask_t cpumask)
{
#ifdef CONFIG_SMP
	rthal_send_ipi(RTHAL_RESCHEDULE_IPI, cpumask);
#endif /* !CONFIG_SMP */
}

static inline int xnlock_is_owner(xnlock_t *lock)
{
	return atomic_read(&lock->owner) == xnarch_current_cpu();
}

#else /* !(CONFIG_SMP || XENO_DEBUG(XNLOCK) */

#define xnlock_init(lock)		do { } while(0)
#define xnlock_get(lock)		do { } while(0)
#define xnlock_put(lock)		do { } while(0)
#define xnlock_get_irqsave(lock,x)	rthal_local_irq_save(x)
#define xnlock_put_irqrestore(lock,x)	rthal_local_irq_restore(x)
#define xnlock_clear_irqoff(lock)	rthal_local_irq_disable()
#define xnlock_clear_irqon(lock)	rthal_local_irq_enable()
#define xnlock_is_owner(lock)		1

#define DECLARE_XNLOCK(lock)
#define DECLARE_EXTERN_XNLOCK(lock)
#define DEFINE_XNLOCK(lock)
#define DEFINE_PRIVATE_XNLOCK(lock)

static inline void xnarch_send_ipi (xnarch_cpumask_t cpumask)
{
}

#endif /* !(CONFIG_SMP || XENO_DEBUG(XNLOCK)) */

#define xnlock_sync_irq(lock, x)			\
	do {						\
		xnlock_put_irqrestore(lock, x);		\
		xnlock_get_irqsave(lock, x);		\
	} while(0)

static inline int xnarch_remap_vm_page(struct vm_area_struct *vma,
				       unsigned long from,
				       unsigned long to)
{
	return wrap_remap_vm_page(vma, from, to);
}

static inline int xnarch_remap_io_page_range(struct file *filp,
					     struct vm_area_struct *vma,
					     unsigned long from,
					     phys_addr_t to,
					     unsigned long size,
					     pgprot_t prot)
{
	return wrap_remap_io_page_range(vma, from, to, size,
					wrap_phys_mem_prot(filp, (to) >> PAGE_SHIFT,
							   size, prot));
}

static inline int xnarch_remap_kmem_page_range(struct vm_area_struct *vma,
					       unsigned long from,
					       unsigned long to,
					       unsigned long size,
					       pgprot_t prot)
{
    return wrap_remap_kmem_page_range(vma,from,to,size,prot);
}

#define xnarch_finalize_no_switch(dead_tcb) do { } while(0)

#ifdef rthal_fault_range
#define xnarch_fault_range(vma) rthal_fault_range(vma)
#else /* !rthal_fault_range */
#define xnarch_fault_range(vma) do { } while (0)
#endif /*!rthal_fault_range */

#ifndef xnarch_hisyscall_entry
static inline void xnarch_hisyscall_entry(void)	{ }
#endif

#ifdef __cplusplus
}
#endif

/* Dashboard and graph control. */
#define XNARCH_DECL_DISPLAY_CONTEXT();
#define xnarch_init_display_context(obj)
#define xnarch_create_display(obj,name,tag)
#define xnarch_delete_display(obj)
#define xnarch_post_graph(obj,state)
#define xnarch_post_graph_if(obj,state,cond)

/* Synchronised realtime clock*/
#define xnarch_hostrt_data	rthal_hostrt_data

#endif /* !_XENO_ASM_GENERIC_SYSTEM_H */
