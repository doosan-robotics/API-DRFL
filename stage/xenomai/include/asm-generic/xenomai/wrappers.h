/*
 * Copyright (C) 2005 Philippe Gerum <rpm@xenomai.org>.
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
 * Linux wrappers.
 */

#ifndef _XENO_ASM_GENERIC_WRAPPERS_H

#ifndef __KERNEL__
#error "Pure kernel header included from user-space!"
#endif

#include <linux/ipipe.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/sched.h>
#if defined(CONFIG_XENO_OPT_HOSTRT) || defined(__IPIPE_FEATURE_REQUEST_TICKDEV)
#include <linux/ipipe_tickdev.h>
#endif /* CONFIG_XENO_OPT_HOSTRT || __IPIPE_FEATURE_REQUEST_TICKDEV */
#include <asm/io.h>

#ifndef CONFIG_IPIPE_CORE
#define IPIPE_CORE_APIREV  0
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,16,0)
#define smp_mb__before_atomic()  smp_mb()
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0)
#include <asm/system.h>
#endif /* kernel < 3.4.0 */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)

#include <linux/wrapper.h>
#include <linux/wait.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/linkage.h>
#include <linux/moduleparam.h>	/* Use the backport. */
#include <asm/atomic.h>

#if BITS_PER_LONG != 32
#error Upgrade to kernel 2.6!
#endif

/* Compiler */
#ifndef __attribute_const__
#define __attribute_const__	/* unimplemented */
#endif
#ifndef __restrict__
#define __restrict__		/* unimplemented */
#endif

#define module_param_named(name,var,type,mode)  module_param(var,type,mode)
#define _MODULE_PARM_STRING_charp "s"
#define compat_module_param_array(name, type, count, perm) \
	static inline void *__check_existence_##name(void) { return &name; } \
	MODULE_PARM(name, "1-" __MODULE_STRING(count) _MODULE_PARM_STRING_##type)

#define container_of(ptr, type, member) ({			\
	const typeof( ((type *)0)->member ) *__mptr = (ptr);	\
	(type *)( (char *)__mptr - offsetof(type,member) );})

/* VM */

#define wrap_remap_vm_page(vma,from,to) ({ \
    vma->vm_flags |= VM_RESERVED; \
    remap_page_range(from,page_to_phys(vmalloc_to_page((void *)to)),PAGE_SIZE,vma->vm_page_prot); \
})
#define wrap_remap_io_page_range(vma,from,to,size,prot) ({ \
    vma->vm_flags |= VM_RESERVED; \
    remap_page_range(from,to,size,prot); \
})
#define wrap_remap_kmem_page_range(vma,from,to,size,prot) ({ \
    vma->vm_flags |= VM_RESERVED; \
    remap_page_range(from,to,size,prot); \
})
#define wrap_switch_mm(prev,next,task)	\
    switch_mm(prev,next,task,(task)->processor)
#define wrap_enter_lazy_tlb(mm,task)	\
    enter_lazy_tlb(mm,task,(task)->processor)
#define pte_offset_kernel(pmd,addr)	pte_offset(pmd,addr)
#define __copy_to_user_inatomic		__copy_to_user
#define __copy_from_user_inatomic	__copy_from_user

/* Sched and process flags */
#define MAX_RT_PRIO 100
#define task_cpu(p) ((p)->processor)
#ifndef CONFIG_PREEMPT
#define preempt_disable()  do { } while(0)
#define preempt_enable()   do { } while(0)
#endif /* !CONFIG_PREEMPT */
#ifndef SCHED_NORMAL
#define SCHED_NORMAL SCHED_OTHER
#endif /* !SCHED_NORMAL */
#define PF_NOFREEZE 0

/* Signals */
#define wrap_get_sigpending(m,p) sigandsets(m, \
					    &(p)->pending.signal, \
					    &(p)->pending.signal)
/* Wait queues */
#define DEFINE_WAIT(w) DECLARE_WAITQUEUE(w, current)
#define is_sync_wait(wait)  (!(wait) || ((wait)->task))

static inline void prepare_to_wait(wait_queue_head_t *q,
				   wait_queue_t *wait,
				   int state)
{
	unsigned long flags;

	wait->flags &= ~WQ_FLAG_EXCLUSIVE;
	spin_lock_irqsave(&q->lock, flags);
	__add_wait_queue(q, wait);
	if (is_sync_wait(wait))
		set_current_state(state);
	spin_unlock_irqrestore(&q->lock, flags);
}

static inline void prepare_to_wait_exclusive(wait_queue_head_t *q,
					     wait_queue_t *wait,
					     int state)
{
	unsigned long flags;

	wait->flags |= WQ_FLAG_EXCLUSIVE;
	spin_lock_irqsave(&q->lock, flags);
	__add_wait_queue_tail(q, wait);
	if (is_sync_wait(wait))
		set_current_state(state);
	spin_unlock_irqrestore(&q->lock, flags);
}

static inline void finish_wait(wait_queue_head_t *q,
			       wait_queue_t *wait)
{
	unsigned long flags;

	__set_current_state(TASK_RUNNING);
	if (waitqueue_active(q)) {
		spin_lock_irqsave(&q->lock, flags);
		list_del_init(&wait->task_list);
		spin_unlock_irqrestore(&q->lock, flags);
	}
}

#ifndef wait_event_interruptible_timeout
#define __wait_event_interruptible_timeout(wq, condition, ret)		\
do {									\
	DEFINE_WAIT(__wait);						\
									\
	for (;;) {							\
		prepare_to_wait(&wq, &__wait, TASK_INTERRUPTIBLE);	\
		if (condition)						\
			break;						\
		if (!signal_pending(current)) {				\
			ret = schedule_timeout(ret);			\
			if (!ret)					\
				break;					\
			continue;					\
		}							\
		ret = -ERESTARTSYS;					\
		break;							\
	}								\
	finish_wait(&wq, &__wait);					\
} while (0)

#define wait_event_interruptible_timeout(wq, condition, timeout)	\
({									\
	long __ret = timeout;						\
	if (!(condition))						\
		__wait_event_interruptible_timeout(wq, condition, __ret); \
	__ret;								\
})
#endif

/* Workqueues. Some 2.4 ports already provide for a limited emulation
   of workqueue calls in linux/workqueue.h, except DECLARE_WORK(), so
   we define the latter here, and leave the rest in
   compat/linux/workqueue.h. */

#define __WORK_INITIALIZER(n,f,d) {				\
	.list	= { &(n).list, &(n).list },			\
	.sync = 0,						\
	.routine = (f),						\
	.data = (d),						\
}
#define DECLARE_WORK(n,f,d)		struct tq_struct n = __WORK_INITIALIZER(n, f, d)
#define DECLARE_WORK_NODATA(n, f)	DECLARE_WORK(n, f, NULL)
#define DECLARE_WORK_FUNC(f)		void f(void *cookie)
#define DECLARE_DELAYED_WORK_NODATA(n, f) DECLARE_WORK(n, f, NULL)

/*
 * WARNING: This is not identical to 2.6 schedule_delayed_work as it delayes
 * the caller to schedule the task after the specified delay. That's fine for
 * our current use cases, though.
 */
#define schedule_delayed_work(work, delay) do {			\
	if (delay) {						\
		set_current_state(TASK_UNINTERRUPTIBLE);	\
		schedule_timeout(delay);			\
	}							\
	schedule_task(work);					\
} while (0)

/* Msleep is unknown before 2.4.28 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,4,28)
#define msleep(x) do {				 \
	set_current_state(TASK_UNINTERRUPTIBLE); \
	schedule_timeout(((x)*HZ)/1000);         \
} while(0)
#endif

/* Shorthand for timeout setup */
#define schedule_timeout_interruptible(t) do {		\
		set_current_state(TASK_INTERRUPTIBLE);	\
		schedule_timeout(t);				\
} while(0)

#define DEFINE_SPINLOCK(x)	spinlock_t x = SPIN_LOCK_UNLOCKED

#ifndef NSEC_PER_MSEC
#define NSEC_PER_MSEC	1000000L
#endif

#ifndef USEC_PER_SEC
#define USEC_PER_SEC	1000000L
#endif

#define MAX_JIFFY_OFFSET ((~0UL >> 1)-1)

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,4,28)
static inline unsigned int jiffies_to_usecs(const unsigned long j)
{
#if HZ <= 1000 && !(1000 % HZ)
	return (1000000 / HZ) * j;
#elif HZ > 1000 && !(HZ % 1000)
	return (j*1000 + (HZ - 1000))/(HZ / 1000);
#else
	return (j * 1000000) / HZ;
#endif
}
#endif

static inline unsigned long usecs_to_jiffies(const unsigned int u)
{
	if (u > jiffies_to_usecs(MAX_JIFFY_OFFSET))
		return MAX_JIFFY_OFFSET;
#if HZ <= USEC_PER_SEC && !(USEC_PER_SEC % HZ)
	return (u + (USEC_PER_SEC / HZ) - 1) / (USEC_PER_SEC / HZ);
#elif HZ > USEC_PER_SEC && !(HZ % USEC_PER_SEC)
	return u * (HZ / USEC_PER_SEC);
#else
	return (u * HZ + USEC_PER_SEC - 1) / USEC_PER_SEC;
#endif
}

#ifdef MODULE
#define try_module_get(mod) try_inc_mod_count(mod)
#define module_put(mod) __MOD_DEC_USE_COUNT(mod)
#else /* !__MODULE__ */
#define try_module_get(mod) (1)
#define module_put(mod) do { } while (0)
#endif /* !__MODULE__ */

/* Types */
typedef enum __kernel_clockid_t {
    CLOCK_REALTIME  =0,
    CLOCK_MONOTONIC =1
} clockid_t;

typedef int timer_t;
typedef int mqd_t;

/* Decls */
struct task_struct;
void show_stack(struct task_struct *task,
		unsigned long *sp);

#define atomic_cmpxchg(v, old, new) ((int)cmpxchg(&((v)->counter), old, new))

#ifndef __deprecated
#define __deprecated  __attribute__((deprecated))
#endif

#ifndef BITOP_WORD
#define BITOP_WORD(nr)	((nr) / BITS_PER_LONG)
#endif

#define GFP_DMA32  GFP_DMA
#define __GFP_BITS_SHIFT 20
#define pgprot_noncached(p) (p)

typedef atomic_t atomic_long_t;

static inline long atomic_long_read(atomic_long_t *l)
{
	atomic_t *v = (atomic_t *)l;

	return (long)atomic_read(v);
}

static inline void atomic_long_set(atomic_long_t *l, long i)
{
	atomic_t *v = (atomic_t *)l;

	atomic_set(v, i);
}

static inline void atomic_long_inc(atomic_long_t *l)
{
	atomic_t *v = (atomic_t *)l;

	atomic_inc(v);
}

static inline void atomic_long_dec(atomic_long_t *l)
{
	atomic_t *v = (atomic_t *)l;

	atomic_dec(v);
}

static inline unsigned long hweight_long(unsigned long w)
{
	return hweight32(w);
}

#define find_first_bit(addr, size) find_next_bit((addr), (size), 0)
unsigned long find_next_bit(const unsigned long *addr,
			    unsigned long size, unsigned long offset);

#define mmiowb()	barrier()

#define wrap_f_inode(file)	((file)->f_dentry->d_inode)

static inline void *kzalloc(size_t size, int flags)
{
	void *ptr;

	ptr = kmalloc(size, flags);
	if (ptr == NULL)
		return NULL;

	memset(ptr, 0, size);

	return ptr;
}

#else /* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0) */

#define compat_module_param_array(name, type, count, perm) \
	module_param_array(name, type, NULL, perm)

/* VM */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,15) && defined(CONFIG_MMU)
#define wrap_remap_vm_page(vma,from,to) \
    vm_insert_page(vma,from,vmalloc_to_page((void *)to))

#define wrap_remap_io_page_range(vma,from,to,size,prot)  ({		\
    (vma)->vm_page_prot = pgprot_noncached((vma)->vm_page_prot);	\
    /* Sets VM_RESERVED | VM_IO | VM_PFNMAP on the vma. */		\
    remap_pfn_range(vma,from,(to) >> PAGE_SHIFT,size,pgprot_noncached(prot)); \
    })
#define wrap_remap_kmem_page_range(vma,from,to,size,prot)  ({		\
    /* Sets VM_RESERVED | VM_IO | VM_PFNMAP on the vma. */		\
    remap_pfn_range(vma,from,(to) >> PAGE_SHIFT,size,prot);		\
    })
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,10)
/* Actually, this is a best-effort since we don't have
 * vm_insert_page(), and has the unwanted side-effet of setting the
 * VM_IO flag on the vma, which prevents GDB inspection of the mmapped
 * memory. Anyway, this legacy would only hit setups using pre-2.6.11
 * kernel revisions. */
#define wrap_remap_vm_page(vma,from,to) \
    remap_pfn_range(vma,from,page_to_pfn(vmalloc_to_page((void *)to)),PAGE_SHIFT,vma->vm_page_prot)
#define wrap_remap_io_page_range(vma,from,to,size,prot)  ({		\
    (vma)->vm_page_prot = pgprot_noncached((vma)->vm_page_prot);	\
    /* Sets VM_RESERVED | VM_IO | VM_PFNMAP on the vma. */		\
    remap_pfn_range(vma,from,(to) >> PAGE_SHIFT,size,pgprot_noncached(prot));		\
    })
#define wrap_remap_kmem_page_range(vma,from,to,size,prot)  ({		\
    /* Sets VM_RESERVED | VM_IO | VM_PFNMAP on the vma. */		\
    remap_pfn_range(vma,from,(to) >> PAGE_SHIFT,size,prot);		\
    })
#else /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10) */
#define wrap_remap_vm_page(vma,from,to) ({ \
    vma->vm_flags |= VM_RESERVED; \
    remap_page_range(from,page_to_phys(vmalloc_to_page((void *)to)),PAGE_SIZE,vma->vm_page_prot); \
})
#define wrap_remap_io_page_range(vma,from,to,size,prot) ({	\
      vma->vm_flags |= VM_RESERVED;				\
      remap_page_range(vma,from,to,size,prot);			\
    })
#define wrap_remap_kmem_page_range(vma,from,to,size,prot) ({	\
      vma->vm_flags |= VM_RESERVED;				\
      remap_page_range(vma,from,to,size,prot);			\
    })
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,15) */

#ifndef __GFP_BITS_SHIFT
#define __GFP_BITS_SHIFT 20
#endif

#include <asm/pgtable.h>

#ifndef pgprot_noncached
#define pgprot_noncached(p) (p)
#endif /* !pgprot_noncached */

#if IPIPE_CORE_APIREV >= 2
#define wrap_switch_mm(prev, next, tsk) \
	ipipe_switch_mm_head(prev, next, tsk)
#elif IPIPE_CORE_APIREV == 1
#define wrap_switch_mm(prev, next, tsk) \
	__switch_mm(prev, next, tsk)
#elif defined(__IPIPE_FEATURE_HARDENED_SWITCHMM)
#define wrap_switch_mm(prev, next, tsk) \
	__switch_mm(prev, next, tsk)
#else
#define wrap_switch_mm(prev, next, tsk) \
	switch_mm(prev, next, tsk)
#endif

#define wrap_enter_lazy_tlb(mm,task)	\
    enter_lazy_tlb(mm,task)

/* Device registration */
#if  LINUX_VERSION_CODE > KERNEL_VERSION(2,6,25)
#define DECLARE_DEVCLASS(clname) struct class *clname
#if  LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
#define wrap_device_create(c,p,dt,dv,fmt,args...)	device_create(c,p,dt,fmt , ##args)
#else  /* >= 2.6.27 */
#define wrap_device_create(c,p,dt,dv,fmt,args...)	device_create(c,p,dt,dv,fmt , ##args)
#endif  /* >= 2.6.27 */
#define wrap_device_destroy	device_destroy
#define DECLARE_DEVHANDLE(devh) struct device *devh
#else /* <= 2.6.25 */
#define DECLARE_DEVHANDLE(devh) struct class_device *devh
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,13)
#define DECLARE_DEVCLASS(clname) struct class *clname
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,15) || defined(gfp_zone)
/*
 * Testing that gfp_zone() exists as a macro is a gross hack used to
 * discover DENX-originated 2.6.14 kernels, for which the prototype of
 * class_device_create() already conforms to the one found in 2.6.15
 * mainline.
 */
#define wrap_device_create class_device_create
#else /* < 2.6.15 */
#define wrap_device_create(c,p,dt,dv,fmt,args...) class_device_create(c,dt,dv,fmt , ##args)
#endif /* >= 2.6.15 */
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
#define DECLARE_DEVCLASS(clname) struct class_simple *clname
#define wrap_device_create(c,p,dt,dv,fmt,args...) class_simple_device_add(c,dt,dv,fmt , ##args)
#define class_create class_simple_create
#define class_device_destroy(a,b) class_simple_device_remove(b)
#define class_destroy class_simple_destroy
#endif  /* >= 2.6.13 */
#define wrap_device_destroy(a, b)	class_device_destroy(a, b)
#endif  /* >= 2.6.26 */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,15)
#define atomic_cmpxchg(v, old, new) ((int)cmpxchg(&((v)->counter), old, new))
#endif /* < 2.6.15 */

/* Signals */
#define wrap_get_sigpending(m,p) sigorsets(m, \
					   &(p)->pending.signal, \
					   &(p)->signal->shared_pending.signal)

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
#define DECLARE_WORK_NODATA(f, n)	DECLARE_WORK(f, n, NULL)
#define DECLARE_WORK_FUNC(f)		void f(void *cookie)
#define DECLARE_DELAYED_WORK_NODATA(n, f) DECLARE_DELAYED_WORK(n, f, NULL)
#else /* >= 2.6.20 */
#define DECLARE_WORK_NODATA(f, n)	DECLARE_WORK(f, n)
#define DECLARE_WORK_FUNC(f)		void f(struct work_struct *work)
#define DECLARE_DELAYED_WORK_NODATA(n, f) DECLARE_DELAYED_WORK(n, f)
#endif /* >= 2.6.20 */

#define wrap_f_inode(file)	((file)->f_path.dentry->d_inode)

#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0) */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18)
#define IRQF_SHARED			SA_SHIRQ
#endif /* < 2.6.18 */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,26)
#define clamp(val, min, max) ({			\
	typeof(val) __val = (val);		\
	typeof(min) __min = (min);		\
	typeof(max) __max = (max);		\
	(void) (&__val == &__min);		\
	(void) (&__val == &__max);		\
	__val = __val < __min ? __min: __val;	\
	__val > __max ? __max: __val; })
#endif /* < 2.6.25 */

#if defined(CONFIG_LTT) || \
    (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31) && defined(CONFIG_MARKERS))

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
#define trace_mark(channel, ev, fmt, args...)	\
	MARK(channel##_##ev, fmt , ##args)
#else /* >= 2.6.24 */
#include <linux/marker.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
#undef trace_mark
#define trace_mark(channel, ev, fmt, args...)	\
	__trace_mark(0, channel##_##ev, NULL, fmt, ## args)
#endif /* < 2.6.27 */
#endif /* >= 2.6.24 */

#else /* !LTTng markers */

#undef trace_mark
#define trace_mark(channel, ev, fmt, args...)	do { } while (0)

#endif /* !LTTng markers */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22)
#define KMALLOC_MAX_SIZE 131072
#if BITS_PER_LONG == 64
#define atomic_long_cmpxchg(l, old, new)	\
	atomic_cmpxchg((atomic64_t *)(l), (old), (new))
#define atomic_long_dec_and_test(l)		\
	atomic_dec_and_test((atomic64_t *)l)
#define atomic_long_inc_and_test(l)		\
	atomic_inc_and_test((atomic64_t *)l)
#else
#define atomic_long_cmpxchg(l, old, new)	\
	atomic_cmpxchg((atomic_t *)(l), (old), (new))
#define atomic_long_dec_and_test(l)		\
	atomic_dec_and_test((atomic_t *)l)
#define atomic_long_inc_and_test(l)		\
	atomic_inc_and_test((atomic_t *)l)
#endif
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)

#include <linux/semaphore.h>
#include <linux/pid.h>
#include <linux/rcupdate.h>

static inline void wrap_lock_tasklist(void)
{
	rcu_read_lock();
}

static inline void wrap_unlock_tasklist(void)
{
	rcu_read_unlock();
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31)

static inline struct task_struct *wrap_find_task_by_pid(pid_t nr)
{
	return pid_task(find_pid_ns(nr, &init_pid_ns), PIDTYPE_PID);
}

#else /* LINUX_VERSION_CODE < 2.6.31 */

#define wrap_find_task_by_pid(nr)	\
	find_task_by_pid_ns(nr, &init_pid_ns)

#endif /* LINUX_VERSION_CODE < 2.6.31 */

#define kill_proc(pid, sig, priv)	\
  kill_proc_info(sig, (priv) ? SEND_SIG_PRIV : SEND_SIG_NOINFO, pid)

#else /* LINUX_VERSION_CODE < 2.6.27 */

#include <linux/spinlock.h>
#include <asm/semaphore.h>

static inline void wrap_lock_tasklist(void)
{
	read_lock(&tasklist_lock);
}

static inline void wrap_unlock_tasklist(void)
{
	read_unlock(&tasklist_lock);
}

#define wrap_find_task_by_pid(nr)  find_task_by_pid(nr)

#endif /* LINUX_VERSION_CODE < 2.6.27 */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29)

#ifndef current_cap
#define current_cap()  ((current)->cap_effective)
#endif

static inline int wrap_raise_cap(int cap)
{
	cap_raise(current_cap(), cap);
	return 0;
}
#else /* LINUX_VERSION_CODE >= 2.6.29 */

#include <linux/cred.h>

static inline int wrap_raise_cap(int cap)
{
	struct cred *new;

	new = prepare_creds();
	if (new == NULL)
		return -ENOMEM;

	cap_raise(new->cap_effective, cap);

	return commit_creds(new);
}
#endif /* LINUX_VERSION_CODE >= 2.6.29 */

#ifdef CONFIG_XENO_OPT_VFILE
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,30)
#include <linux/module.h>
#include <linux/proc_fs.h>
static inline void wrap_proc_dir_entry_owner(struct proc_dir_entry *entry)
{
    entry->owner = THIS_MODULE;
}
#else  /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,30) */
#define wrap_proc_dir_entry_owner(entry) do { (void)entry; } while(0)
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,30) */
#endif /* CONFIG_XENO_OPT_VFILE */

#ifndef list_first_entry
#define list_first_entry(ptr, type, member) \
	list_entry((ptr)->next, type, member)
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
#define rthal_irq_descp(irq)	(irq_desc + (irq))
#else /* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32) */
#define rthal_irq_descp(irq)	irq_to_desc(irq)
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32) */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,33)
#define rthal_irqdesc_lock(irq, flags)					\
	spin_lock_irqsave(&rthal_irq_descp(irq)->lock, flags)
#define rthal_irqdesc_unlock(irq, flags)				\
	spin_unlock_irqrestore(&rthal_irq_descp(irq)->lock, flags)
#else /* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,33) */
#define rthal_irqdesc_lock(irq, flags)					\
	raw_spin_lock_irqsave(&rthal_irq_descp(irq)->lock, flags)
#define rthal_irqdesc_unlock(irq, flags)				\
	raw_spin_unlock_irqrestore(&rthal_irq_descp(irq)->lock, flags)
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,33) */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,11)
#define unlocked_ioctl ioctl
#define DECLARE_IOCTL_HANDLER(name, filp, cmd, arg)		\
	int name(struct inode *__inode__, struct file *filp,	\
	     unsigned int cmd, unsigned long arg)
#else
#define DECLARE_IOCTL_HANDLER(name, filp, cmd, arg)		\
	long name(struct file *filp, unsigned int cmd, unsigned long arg)
#endif

#ifndef DEFINE_SEMAPHORE
/* Legacy DECLARE_MUTEX vanished in 2.6.37 */
#define DEFINE_BINARY_SEMAPHORE(sem) DECLARE_MUTEX(sem)
#elif defined(CONFIG_PREEMPT_RT)
#define DEFINE_BINARY_SEMAPHORE(sem) DEFINE_SEMAPHORE(sem, 1)
#else
#define DEFINE_BINARY_SEMAPHORE(sem) DEFINE_SEMAPHORE(sem)
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,12,0)
#define CONFIG_GENERIC_HARDIRQS 1
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37) && defined(CONFIG_GENERIC_HARDIRQS)

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,39)
#define irq_desc_get_chip(desc)	get_irq_desc_chip(desc)
#endif

/*
 * The irq chip descriptor has been heavily revamped in
 * 2.6.37. Provide generic accessors to the chip handlers we need for
 * kernels implementing those changes.
 */
#define rthal_irq_chip_enable(irq)					\
	({								\
		struct irq_desc *desc = rthal_irq_descp(irq);		\
		struct irq_chip *chip = irq_desc_get_chip(desc);	\
		int __ret__ = 0;					\
		if (unlikely(chip->irq_unmask == NULL))			\
			__ret__ = -ENODEV;				\
		else							\
			chip->irq_unmask(&desc->irq_data);		\
		__ret__;						\
	})
#define rthal_irq_chip_disable(irq)					\
	({								\
		struct irq_desc *desc = rthal_irq_descp(irq);		\
		struct irq_chip *chip = irq_desc_get_chip(desc);	\
		int __ret__ = 0;					\
		if (unlikely(chip->irq_mask == NULL))			\
			__ret__ = -ENODEV;				\
		else							\
			chip->irq_mask(&desc->irq_data);		\
		__ret__;						\
	})
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0)

#include <linux/mm.h>
#include <linux/smp.h>

#ifndef cpu_online_map
#define cpu_online_mask (&cpu_online_map)
#endif /* !cpu_online_map */
#ifndef cpu_online
#ifdef CONFIG_SMP
#define cpu_online(cpu)	(cpu_online_map & (1UL << (cpu)))
#else /* !CONFIG_SMP */
#define cpu_online(cpu)	((cpu) == 0)
#endif /* !CONFIG_SMP */
#endif /* !cpu_online */

static inline
unsigned long vm_mmap(struct file *file, unsigned long addr,
	unsigned long len, unsigned long prot,
	unsigned long flag, unsigned long offset)
{
	struct mm_struct *mm = current->mm;
	unsigned long ret;

	down_write(&mm->mmap_sem);
	ret = do_mmap(file, addr, len, prot, flag, offset);
	up_write(&mm->mmap_sem);

	return ret;
}

static inline int vm_munmap(unsigned long start, size_t len)
{
	struct mm_struct *mm = current->mm;
	int ret;

	down_write(&mm->mmap_sem);
	ret = do_munmap(mm, start, len);
	up_write(&mm->mmap_sem);

	return ret;
}

#endif /* LINUX_VERSION_CODE < 3.4.0 */

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,5,0)
#define KGIDT_INIT(pid) (pid)
#endif /* LINUX < 3.8.0 */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,9,0)
#include <linux/sched/rt.h>
#endif /* LINUX >= 3.9.0 */

#include <linux/seq_file.h>
#ifndef SEQ_START_TOKEN
#define SEQ_START_TOKEN ((void *)1)
#endif
#ifndef SEQ_SKIP
#define SEQ_SKIP	0	/* not implemented. */
#endif

#if IPIPE_CORE_APIREV >= 2
#define wrap_select_timers(mask) ipipe_select_timers(mask)
#elif IPIPE_CORE_APIREV == 1
#define wrap_select_timers(mask) ipipe_timers_request()
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
#include <linux/proc_fs.h>

#define PDE_DATA(inode)	PDE(inode)->data

static inline void proc_remove(struct proc_dir_entry *pde)
{
	remove_proc_entry(pde->name, pde->parent);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,26)
static inline struct proc_dir_entry *
proc_create_data(const char *name, mode_t mode, struct proc_dir_entry *parent,
		 const struct file_operations *proc_fops, void *data)
{
	struct proc_dir_entry *pde = create_proc_entry(name, mode, parent);

	if (pde) {
		pde->proc_fops = (struct file_operations *)proc_fops;
		pde->data = data;
	}
	return pde;
}
#endif /* < 2.6.26 */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,25)
static inline struct proc_dir_entry *
proc_create(const char *name, mode_t mode, struct proc_dir_entry *parent,
	    const struct file_operations *proc_fops)
{
	struct proc_dir_entry *pde = create_proc_entry(name, mode, parent);

	if (pde)
		pde->proc_fops = (struct file_operations *)proc_fops;
	return pde;
}
#endif /* < 2.6.25 */
#endif /* < 3.10 */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,0,0)
#define cpu_set(cpu, mask)		cpumask_set_cpu(cpu, &(mask))
#define cpu_clear(cpu, mask)		cpumask_clear_cpu(cpu, &(mask))
#define cpus_clear(mask)		cpumask_clear(&(mask))
#define cpu_isset(cpu, mask)		cpumask_test_cpu(cpu, &(mask))
#define cpus_and(dst, src1, src2)	cpumask_and(&(dst), &(src1), &(src2))
#define cpus_equal(mask1, mask2)	cpumask_equal(&(mask1), &(mask2))
#define cpus_empty(mask)		cpumask_empty(&(mask))
#define cpu_test_and_set(cpu, mask)	cpumask_test_and_set_cpu(cpu, &(mask))
#define first_cpu(mask)			cpumask_first(&(mask))
#define cpumask_of_cpu(mask)		(*cpumask_of(mask))
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,19,0)
#define user_msghdr msghdr
#endif

#endif /* _XENO_ASM_GENERIC_WRAPPERS_H */
