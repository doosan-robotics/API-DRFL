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
 */

#ifndef _XENO_ASM_ARM_WRAPPERS_H
#define _XENO_ASM_ARM_WRAPPERS_H

#ifndef __KERNEL__
#error "Pure kernel header included from user-space!"
#endif

#include <linux/ipipe.h>
#include <linux/interrupt.h>
#include <asm-generic/xenomai/wrappers.h> /* Read the generic portion. */

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,16,0)
#include <asm/system.h>
#else
#include <asm/barrier.h>
#include <asm/compiler.h>
#include <asm/cmpxchg.h>
#include <asm/switch_to.h>
#include <asm/system_info.h>
#include <asm/system_misc.h>
#endif

#define wrap_phys_mem_prot(filp,pfn,size,prot)	(prot)

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0)
#define wrap_strncpy_from_user(dstP, srcP, n)	__strncpy_from_user(dstP, srcP, n)
#else
#define wrap_strncpy_from_user(dstP, srcP, n)	strncpy_from_user(dstP, srcP, n)
#endif

#define rthal_irq_desc_status(irq)	(rthal_irq_descp(irq)->status)

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
extern spinlock_t irq_controller_lock;

#define rthal_irq_desc_lock(irq) (&irq_controller_lock)
#define rthal_irq_chip_enable(irq)					\
	({								\
		int __err__ = 0;					\
		if (rthal_irq_descp(irq)->chip->unmask == NULL)		\
			__err__ = -ENODEV;				\
		else							\
			rthal_irq_descp(irq)->chip->unmask(irq);	\
		__err__;						\
	})
#define rthal_irq_chip_disable(irq)					\
	({								\
		int __err__ = 0;					\
		if (rthal_irq_descp(irq)->chip->mask == NULL)		\
			__err__ = -ENODEV;				\
		else							\
			rthal_irq_descp(irq)->chip->mask(irq);		\
		__err__;						\
	})
typedef irqreturn_t (*rthal_irq_host_handler_t)(int irq,
						void *dev_id,
						struct pt_regs *regs);
#define rthal_irq_chip_end(irq)	rthal_irq_chip_enable(irq)
#define rthal_mark_irq_disabled(irq) (rthal_irq_descp(irq)->disable_depth = 1)
#define rthal_mark_irq_enabled(irq) (rthal_irq_descp(irq)->disable_depth = 0)

extern void (*fp_init)(union fp_state *);
#else /* >= 2.6.19 */
#if !defined(CONFIG_GENERIC_HARDIRQS) \
	|| LINUX_VERSION_CODE < KERNEL_VERSION(2,6,37)
#define rthal_irq_chip_enable(irq)   ({ rthal_irq_descp(irq)->chip->unmask(irq); 0; })
#define rthal_irq_chip_disable(irq)  ({ rthal_irq_descp(irq)->chip->mask(irq); 0; })
#endif
#define rthal_irq_desc_lock(irq) (&rthal_irq_descp(irq)->lock)
#define rthal_irq_chip_end(irq)      ({ rthal_irq_descp(irq)->ipipe_end(irq, rthal_irq_descp(irq)); 0; })
typedef irq_handler_t rthal_irq_host_handler_t;
#define rthal_mark_irq_disabled(irq) do {              \
	    rthal_irq_descp(irq)->depth = 1;            \
	} while(0);
#define rthal_mark_irq_enabled(irq) do {                 \
	    rthal_irq_descp(irq)->depth = 0;             \
	} while(0);
static inline void fp_init(union fp_state *state)
{
    /* FIXME: This is insufficient. */
    memset(state, 0, sizeof(*state));
}

#endif

#if IPIPE_MAJOR_NUMBER == 1 && /* There is no version 0. */	\
	(IPIPE_MINOR_NUMBER < 5 || \
	 (IPIPE_MINOR_NUMBER == 5 && IPIPE_PATCH_NUMBER < 3))
#define __ipipe_mach_release_timer()  \
	__ipipe_mach_set_dec(__ipipe_mach_ticks_per_jiffy)
#endif /* IPIPE < 1.5-03 */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 23)
#define FPEXC_EN FPEXC_ENABLE
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 1, 0) && !defined(CONFIG_VFP_3_2_BACKPORT)
#define vfp_current_hw_state last_VFP_context
#endif /* Linux < 3.1 */

#if defined(CONFIG_SMP) && !defined(CONFIG_XENO_HW_UNLOCKED_SWITCH) && \
	LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0)
#error "Xenomai: ARM SMP systems require unlocked context switch prior to Linux 3.8"
#endif

#if !defined(CONFIG_IPIPE_CORE) && !defined(__IPIPE_FEATURE_UNMASKED_CONTEXT_SWITCH) && defined(TIF_MMSWITCH_INT)
/*
 * Legacy ARM patches might provide unmasked context switch support
 * without defining the common config option; force this support in.
 */
#define CONFIG_IPIPE_UNMASKED_CONTEXT_SWITCH	1
#define CONFIG_XENO_HW_UNLOCKED_SWITCH		1
#endif

#endif /* _XENO_ASM_ARM_WRAPPERS_H */
