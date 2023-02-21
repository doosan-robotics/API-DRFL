/*
 * Copyright (C) 2010 Gilles Chanteperdrix <gch@xenomai.org>.
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

#ifndef STACKSIZE_H
#define STACKSIZE_H

#include <stdint.h>

#include <unistd.h>
#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

static inline unsigned xeno_stacksize(unsigned size)
{
	static const unsigned default_size = __WORDSIZE * 1024;
	static unsigned min_size;
	if (!min_size)
		min_size = PTHREAD_STACK_MIN + getpagesize();

	if (!size)
		size = default_size;
	if (size < min_size)
		size = min_size;

	return size;
}

void xeno_fault_stack(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STACKSIZE_H */
