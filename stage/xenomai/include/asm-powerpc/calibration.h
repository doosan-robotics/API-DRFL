/*
 * Copyright (C) 2001,2002,2003,2004,2005 Philippe Gerum <rpm@xenomai.org>.
 *
 * 64-bit PowerPC adoption
 *   copyright (C) 2005 Taneli Vähäkangas and Heikki Lindholm
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

#ifndef _XENO_ASM_POWERPC_CALIBRATION_H
#define _XENO_ASM_POWERPC_CALIBRATION_H

#ifndef _XENO_ASM_POWERPC_BITS_INIT_H
#error "please don't include asm/calibration.h directly"
#endif

static inline unsigned long xnarch_get_sched_latency(void)
{
#if CONFIG_XENO_OPT_TIMING_SCHEDLAT != 0
#define __sched_latency CONFIG_XENO_OPT_TIMING_SCHEDLAT
#else


#if defined(CONFIG_PPC_PASEMI)
#define __sched_latency 1000
#elif defined(CONFIG_WALNUT)
#define __sched_latency 11000
#elif defined(CONFIG_YOSEMITE)
#define __sched_latency 2000
#elif defined(CONFIG_BUBINGA)
#define __sched_latency 8000
#elif defined(CONFIG_SYCAMORE)
#define __sched_latency 8000
#elif defined(CONFIG_SEQUOIA)
#define __sched_latency 3000
#elif defined(CONFIG_LWMON5)
#define __sched_latency 2800
#elif defined(CONFIG_OCOTEA)
#define __sched_latency 2700
#elif defined(CONFIG_BAMBOO)
#define __sched_latency 4000
#elif defined(CONFIG_TAISHAN)
#define __sched_latency 1800
#elif defined(CONFIG_RAINIER)
#define __sched_latency 2300
#elif defined(CONFIG_YUCCA)
#define __sched_latency 2780
#elif defined(CONFIG_YELLOWSTONE)
#define __sched_latency 2700
#elif defined(CONFIG_YOSEMITE)
#define __sched_latency 2500
#elif defined(CONFIG_MPC8349_ITX)
#define __sched_latency 2500
#elif defined(CONFIG_MPC836x_MDS)
#define __sched_latency 2900
#elif defined(CONFIG_MPC5121_ADS)
#define __sched_latency 4000
#elif defined(CONFIG_MPC8272_ADS)
#define __sched_latency 5500
#elif defined(CONFIG_MPC85xx_RDB)
#define __sched_latency 2000
#elif defined(CONFIG_MVME7100)
#define __sched_latency 1500
#elif defined(CONFIG_TQM8548)
#define __sched_latency 500
#elif defined(CONFIG_TQM8560)
#define __sched_latency 1000
#elif defined(CONFIG_TQM8555)
#define __sched_latency 2000
#elif defined(CONFIG_KUP4K)
#define __sched_latency 22000
#elif defined(CONFIG_P1022_DS)
#define __sched_latency 3000
/*
 * Check for the most generic configs at the bottom of this list, so
 * that the most specific choices available are picked first.
 */
#elif defined(CONFIG_MPC85xx) || defined(CONFIG_PPC_85xx)
#define __sched_latency 1000
#elif defined(CONFIG_405GPR)
#define __sched_latency 9000
#elif defined(CONFIG_PPC_MPC52xx)
#define __sched_latency 4500
#elif defined(CONFIG_PPC_8xx)
#define __sched_latency 25000
#endif

#ifndef __sched_latency
/* Platform is unknown: pick a default value. */
#ifdef CONFIG_PPC64
#define __sched_latency 1000
#else
#define __sched_latency 4000
#endif
#endif

#endif /* CONFIG_XENO_OPT_TIMING_SCHEDLAT */

	return __sched_latency;
}

#undef __sched_latency

#endif /* !_XENO_ASM_POWERPC_CALIBRATION_H */
