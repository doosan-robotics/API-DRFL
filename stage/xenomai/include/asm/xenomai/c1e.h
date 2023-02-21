/*
 * Copyright (C) 2014 Gilles Chanteperdrix <gilles.chanteperdrix@xenomai.org>.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
#ifndef C1E_H
#define C1E_H

#include <linux/version.h>

#ifndef __KERNEL__
#error "Pure kernel header included from user-space!"
#endif

#ifndef _XENO_ASM_X86_HAL_H
#error "please don't include asm/c1e.h directly"
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0)

static inline void rthal_c1e_disable(void)
{
}

#else

void rthal_c1e_disable(void);

#endif

#endif /* C1E_H */
