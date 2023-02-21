/*
 * Copyright (C) 2009 Gilles Chanteperdrix <gilles.chanteperdrix@xenomai.org>.
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
#ifndef _XENO_ASM_GENERIC_CURRENT_H
#define _XENO_ASM_GENERIC_CURRENT_H

#include <pthread.h>
#include <nucleus/thread.h>

extern pthread_key_t xeno_current_mode_key;

xnhandle_t xeno_slow_get_current(void);
unsigned long xeno_slow_get_current_mode(void);
void xeno_current_warn_old(void);

#ifdef HAVE___THREAD
extern __thread __attribute__ ((tls_model ("initial-exec")))
xnhandle_t xeno_current;
extern __thread __attribute__ ((tls_model ("initial-exec")))
unsigned long *xeno_current_mode;

static inline xnhandle_t xeno_get_current(void)
{
	return xeno_current;
}

#define xeno_get_current_fast() xeno_get_current()

static inline unsigned long xeno_get_current_mode(void)
{
	return xeno_current_mode ? *xeno_current_mode : XNRELAX;
}

#else /* ! HAVE___THREAD */
extern pthread_key_t xeno_current_key;

xnhandle_t xeno_slow_get_current(void);

unsigned long xeno_slow_get_current_mode(void);

static inline xnhandle_t xeno_get_current(void)
{
	void *val = pthread_getspecific(xeno_current_key);

	return (xnhandle_t)val ?: xeno_slow_get_current();
}

/* syscall-free, but unreliable in TSD destructor context */
static inline xnhandle_t xeno_get_current_fast(void)
{
	void *val = pthread_getspecific(xeno_current_key);

	return (xnhandle_t)val ?: XN_NO_HANDLE;
}

static inline unsigned long xeno_get_current_mode(void)
{
	unsigned long *mode;

	mode = pthread_getspecific(xeno_current_mode_key);

	return mode ? *mode : XNRELAX;
}

#endif /* ! HAVE___THREAD */

void xeno_init_current_keys(void);

void xeno_set_current(void);

void xeno_set_current_mode(unsigned long offset);

#endif /* _XENO_ASM_GENERIC_CURRENT_H */
