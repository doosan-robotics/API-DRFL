/*
 * Copyright (C) 2011-2013 Gilles Chanteperdrix <gch@xenomai.org>.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA.
 */

#ifndef _XENO_POSIX_SELECT_H
#define _XENO_POSIX_SELECT_H

#if !(defined(__KERNEL__) || defined(__XENO_SIM__))

#pragma GCC system_header

#include_next <sys/select.h>

#ifdef __cplusplus
extern "C" {
#endif

extern int __real_select (int __nfds, fd_set *__restrict __readfds,
			  fd_set *__restrict __writefds,
			  fd_set *__restrict __exceptfds,
			  struct timeval *__restrict __timeout);

#ifdef __cplusplus
}
#endif

#endif /* !(__KERNEL__ || __XENO_SIM__) */

#endif /* _XENO_POSIX_SELECT_H */
