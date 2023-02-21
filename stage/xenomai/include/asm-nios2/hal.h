/**
 *   @ingroup hal
 *   @file
 *
 *   Real-Time Hardware Abstraction Layer for the NIOS2 architecture.
 *
 *   Copyright &copy; 2009 Philippe Gerum.
 *
 *   Xenomai is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU General Public License as
 *   published by the Free Software Foundation, Inc., 675 Mass Ave,
 *   Cambridge MA 02139, USA; either version 2 of the License, or (at
 *   your option) any later version.
 *
 *   Xenomai is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *   General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with Xenomai; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
 *   02111-1307, USA.
 */

#ifndef _XENO_ASM_NIOS2_HAL_H
#define _XENO_ASM_NIOS2_HAL_H

#include <linux/irq.h>
#include <asm/system.h>
#include <asm/processor.h>
#include <asm/div64.h>

#define RTHAL_TIMER_DEVICE	"hrtimer"
#define RTHAL_TIMER_IRQ		__ipipe_hrtimer_irq
#define RTHAL_CLOCK_DEVICE	"hrclock"
#define RTHAL_CLOCK_MEMBASE     __ipipe_hrclock_membase

#include <asm-generic/xenomai/hal.h>	/* Read the generic bits. */

typedef unsigned long long rthal_time_t;

static inline __attribute_const__ unsigned long ffnz(unsigned long ul)
{
	return ffs(ul) - 1;
}

#define rthal_grab_control()     do { } while(0)
#define rthal_release_control()  do { } while(0)

static inline unsigned long long rthal_rdtsc(void)
{
	unsigned long long t;
	rthal_read_tsc(t);
	return t;
}

static inline void rthal_timer_program_shot(unsigned long delay)
{
	if (delay < 100)
		rthal_schedule_irq_head(RTHAL_TIMER_IRQ);
	else
		__ipipe_program_hrtimer(delay);
}

    /* Private interface -- Internal use only */

static inline struct mm_struct *rthal_get_active_mm(void)
{
	return current->active_mm;
}

#ifdef CONFIG_XENO_OPT_TIMING_PERIODIC
extern int rthal_periodic_p;
#else /* !CONFIG_XENO_OPT_TIMING_PERIODIC */
#define rthal_periodic_p  0
#endif /* CONFIG_XENO_OPT_TIMING_PERIODIC */

void rthal_thread_switch(struct thread_struct *prev,
			 struct thread_struct *next,
			 int kthreadp);

asmlinkage void rthal_thread_trampoline(void);

static const char *const rthal_fault_labels[] = {
	[0] = "Breakpoint",
	[1] = "Data or instruction access",
	[2] = "Unaligned access",
	[3] = "Illegal instruction",
	[4] = "Supervisor instruction",
	[5] = "Division error",
	[6] = NULL
};

#endif /* !_XENO_ASM_NIOS2_HAL_H */
