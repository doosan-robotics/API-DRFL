/*
 * Copyright (C) 2008,2011 Gilles Chanteperdrix <gch@xenomai.org>.
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

#ifndef _XENO_NUCLEUS_SYS_PPD_H
#define _XENO_NUCLEUS_SYS_PPD_H

#include <nucleus/ppd.h>
#include <nucleus/heap.h>

struct xnsys_ppd {
	xnshadow_ppd_t ppd;
	xnheap_t sem_heap;
#ifdef XNARCH_HAVE_MAYDAY
	unsigned long mayday_addr;
#endif
	xnarch_atomic_t refcnt;
#define ppd2sys(addr) container_of(addr, struct xnsys_ppd, ppd)
};

extern struct xnsys_ppd __xnsys_global_ppd;

#ifdef CONFIG_XENO_OPT_PERVASIVE

static inline struct xnsys_ppd *xnsys_ppd_get(int global)
{
	xnshadow_ppd_t *ppd;

	if (global || !(ppd = xnshadow_ppd_get(0)))
		return &__xnsys_global_ppd;

	return ppd2sys(ppd);
}

#else /* !CONFIG_XENO_OPT_PERVASIVE */

static inline struct xnsys_ppd *xnsys_ppd_get(int global)
{
	return &__xnsys_global_ppd;
}

#endif /* !CONFIG_XENO_OPT_PERVASIVE */

#endif /* _XENO_NUCLEUS_SYS_PPD_H */
