/**
 *   @ingroup hal
 *   @file
 *
 *   Real-Time Hardware Abstraction Layer for the SuperH architecture.
 *
 *   Copyright &copy; 2011 Philippe Gerum.
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

#ifndef _XENO_ASM_SH_HAL_H
#define _XENO_ASM_SH_HAL_H

#include <linux/irq.h>
#include <asm/system.h>
#include <asm/processor.h>

#define RTHAL_TIMER_DEVICE	"TMU0"
#define RTHAL_TIMER_IRQ		__ipipe_hrtimer_irq
#define RTHAL_CLOCK_DEVICE	"TMU1"
#define RTHAL_CLOCK_MEMBASE     __pa(__ipipe_tsc)

#include <asm-generic/xenomai/hal.h>	/* Read the generic bits. */

typedef unsigned long long rthal_time_t;

static inline __attribute_const__ unsigned long ffnz(unsigned long ul)
{
	return ffs(ul) - 1;
}

#define rthal_grab_control()     do { } while(0)
#define rthal_release_control()  do { } while(0)

#ifdef CONFIG_XENO_HW_FPU

#ifdef CONFIG_CPU_SH4
#define FPSCR_RCHG 0x00000000
#else
#error "unsupported SH architecture"
#endif

#define rthal_get_fpu_owner(cur)			\
	({						\
	unsigned long __sr;				\
	__asm__ __volatile__("stc	sr, %0\n\t"	\
			     : "=&r" (__sr)		\
			     : /* empty */);		\
	(__sr & SR_FD) ? NULL : cur;			\
	})

#define rthal_disable_fpu()	disable_fpu()

#define rthal_enable_fpu()	enable_fpu()

static inline void rthal_save_fpu(struct thread_struct *ts)
{
	unsigned long dummy;

	enable_fpu();
	asm volatile ("sts.l	fpul, @-%0\n\t"
		      "sts.l	fpscr, @-%0\n\t"
		      "lds	%2, fpscr\n\t"
		      "frchg\n\t"
		      "fmov.s	fr15, @-%0\n\t"
		      "fmov.s	fr14, @-%0\n\t"
		      "fmov.s	fr13, @-%0\n\t"
		      "fmov.s	fr12, @-%0\n\t"
		      "fmov.s	fr11, @-%0\n\t"
		      "fmov.s	fr10, @-%0\n\t"
		      "fmov.s	fr9, @-%0\n\t"
		      "fmov.s	fr8, @-%0\n\t"
		      "fmov.s	fr7, @-%0\n\t"
		      "fmov.s	fr6, @-%0\n\t"
		      "fmov.s	fr5, @-%0\n\t"
		      "fmov.s	fr4, @-%0\n\t"
		      "fmov.s	fr3, @-%0\n\t"
		      "fmov.s	fr2, @-%0\n\t"
		      "fmov.s	fr1, @-%0\n\t"
		      "fmov.s	fr0, @-%0\n\t"
		      "frchg\n\t"
		      "fmov.s	fr15, @-%0\n\t"
		      "fmov.s	fr14, @-%0\n\t"
		      "fmov.s	fr13, @-%0\n\t"
		      "fmov.s	fr12, @-%0\n\t"
		      "fmov.s	fr11, @-%0\n\t"
		      "fmov.s	fr10, @-%0\n\t"
		      "fmov.s	fr9, @-%0\n\t"
		      "fmov.s	fr8, @-%0\n\t"
		      "fmov.s	fr7, @-%0\n\t"
		      "fmov.s	fr6, @-%0\n\t"
		      "fmov.s	fr5, @-%0\n\t"
		      "fmov.s	fr4, @-%0\n\t"
		      "fmov.s	fr3, @-%0\n\t"
		      "fmov.s	fr2, @-%0\n\t"
		      "fmov.s	fr1, @-%0\n\t"
		      "fmov.s	fr0, @-%0\n\t"
		      "lds	%3, fpscr\n\t":"=r" (dummy)
		      :"0"((char *)(&ts->fpu.hard.status)),
		      "r"(FPSCR_RCHG), "r"(FPSCR_INIT)
		      :"memory");
}

static inline void rthal_restore_fpu(struct thread_struct *ts)
{
	unsigned long dummy;

	enable_fpu();
	asm volatile ("lds	%2, fpscr\n\t"
		      "fmov.s	@%0+, fr0\n\t"
		      "fmov.s	@%0+, fr1\n\t"
		      "fmov.s	@%0+, fr2\n\t"
		      "fmov.s	@%0+, fr3\n\t"
		      "fmov.s	@%0+, fr4\n\t"
		      "fmov.s	@%0+, fr5\n\t"
		      "fmov.s	@%0+, fr6\n\t"
		      "fmov.s	@%0+, fr7\n\t"
		      "fmov.s	@%0+, fr8\n\t"
		      "fmov.s	@%0+, fr9\n\t"
		      "fmov.s	@%0+, fr10\n\t"
		      "fmov.s	@%0+, fr11\n\t"
		      "fmov.s	@%0+, fr12\n\t"
		      "fmov.s	@%0+, fr13\n\t"
		      "fmov.s	@%0+, fr14\n\t"
		      "fmov.s	@%0+, fr15\n\t"
		      "frchg\n\t"
		      "fmov.s	@%0+, fr0\n\t"
		      "fmov.s	@%0+, fr1\n\t"
		      "fmov.s	@%0+, fr2\n\t"
		      "fmov.s	@%0+, fr3\n\t"
		      "fmov.s	@%0+, fr4\n\t"
		      "fmov.s	@%0+, fr5\n\t"
		      "fmov.s	@%0+, fr6\n\t"
		      "fmov.s	@%0+, fr7\n\t"
		      "fmov.s	@%0+, fr8\n\t"
		      "fmov.s	@%0+, fr9\n\t"
		      "fmov.s	@%0+, fr10\n\t"
		      "fmov.s	@%0+, fr11\n\t"
		      "fmov.s	@%0+, fr12\n\t"
		      "fmov.s	@%0+, fr13\n\t"
		      "fmov.s	@%0+, fr14\n\t"
		      "fmov.s	@%0+, fr15\n\t"
		      "frchg\n\t"
		      "lds.l	@%0+, fpscr\n\t"
		      "lds.l	@%0+, fpul\n\t"
		      :"=r" (dummy)
		      :"0"(&ts->fpu), "r"(FPSCR_RCHG)
		      :"memory");
}

static inline void rthal_init_fpu(struct thread_struct *ts)
{
	rthal_restore_fpu(ts);
}

#endif /* CONFIG_XENO_HW_FPU */

static inline unsigned long long rthal_rdtsc(void)
{
	unsigned long long t;
	rthal_read_tsc(t);
	return t;
}

static inline void rthal_timer_program_shot(unsigned long delay)
{
	if (delay < 10)
		rthal_schedule_irq_head(RTHAL_TIMER_IRQ);
	else
		__ipipe_program_hrtimer(delay);
}

    /* Private interface -- Internal use only */

static inline struct mm_struct *rthal_get_active_mm(void)
{
	return current->active_mm;
}

asmlinkage void rthal_thread_trampoline(void);

static const char *const rthal_fault_labels[] = {
	[0] = "Breakpoint",
	[1] = "Page fault",
	[2] = "Address error",
	[3] = "FPU error",
	[4] = "Exception error",
	[5] = NULL
};

#endif /* !_XENO_ASM_SH_HAL_H */
