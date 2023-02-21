/*
 * Copyright (C) 2011 Philippe Gerum <rpm@xenomai.org>.
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

#ifndef _XENO_ASM_SH_WRAPPERS_H
#define _XENO_ASM_SH_WRAPPERS_H

#ifndef __KERNEL__
#error "Pure kernel header included from user-space!"
#endif

#include <asm-generic/xenomai/wrappers.h> /* Read the generic portion. */
#include <linux/interrupt.h>
#include <asm/mmu.h>

#define wrap_phys_mem_prot(filp, pfn, size, prot)  \
	pgprot_noncached(prot)

#define wrap_strncpy_from_user(dstP, srcP, n)	strncpy_from_user(dstP, srcP, n)

#define rthal_irq_desc_status(irq)	(rthal_irq_descp(irq)->status)
#define rthal_irq_handlerp(irq)		rthal_irq_descp(irq)->chip
#define rthal_irq_chip_end(irq)		({ rthal_irq_descp(irq)->ipipe_end(irq, rthal_irq_descp(irq)); 0; })

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,37)
#define rthal_irq_chip_enable(irq)					\
	({								\
		int __err__ = 0;					\
		if (unlikely(rthal_irq_handlerp(irq)->unmask == NULL))	\
			__err__ = -ENODEV;				\
		else							\
			rthal_irq_handlerp(irq)->unmask(irq);		\
		__err__;						\
	})
#define rthal_irq_chip_disable(irq)					\
	({								\
		int __err__ = 0;					\
		if (rthal_irq_handlerp(irq)->mask == NULL)		\
			__err__ = -ENODEV;				\
		else							\
			rthal_irq_handlerp(irq)->mask(irq);		\
		__err__;						\
	})
#endif

typedef irq_handler_t rthal_irq_host_handler_t;

#endif /* _XENO_ASM_SH_WRAPPERS_H */
