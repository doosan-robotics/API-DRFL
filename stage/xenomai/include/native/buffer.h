/**
 * @file
 * This file is part of the Xenomai project.
 *
 * @note Copyright (C) 2008 Philippe Gerum <rpm@xenomai.org>
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

#ifndef _XENO_BUFFER_H
#define _XENO_BUFFER_H

#include <native/types.h>

/* Creation flags. */
#define B_PRIO   XNSYNCH_PRIO	/* Pend by task priority order. */
#define B_FIFO   XNSYNCH_FIFO	/* Pend by FIFO order. */

typedef struct rt_buffer_info {

	int iwaiters;
	int owaiters;
	size_t totalmem;
	size_t availmem;
	char name[XNOBJECT_NAME_LEN];

} RT_BUFFER_INFO;

typedef struct rt_buffer_placeholder {
	xnhandle_t opaque;
} RT_BUFFER_PLACEHOLDER;

#if (defined(__KERNEL__) || defined(__XENO_SIM__)) && !defined(DOXYGEN_CPP)

#include <nucleus/synch.h>
#include <nucleus/heap.h>
#include <native/ppd.h>

#define XENO_BUFFER_MAGIC 0x55550c0c

typedef struct rt_buffer {

	unsigned magic;   /* !< Magic code - must be first */

	xnsynch_t isynch_base;	/* !< Base synchronization object -- input side. */
	xnsynch_t osynch_base;	/* !< Base synchronization object -- output side. */
	xnhandle_t handle;	/* !< Handle in registry -- zero if unregistered. */
	char name[XNOBJECT_NAME_LEN]; /* !< Symbolic name. */

	int mode;		/* !< Creation mode. */
	off_t rdoff;		/* !< Read offset. */
	off_t wroff;		/* !< Write offset. */
	size_t fillsz;		/* !< Filled space. */

	u_long wrtoken;		/* !< Write token. */
	u_long rdtoken;		/* !< Read token. */

	size_t bufsz;		/* !< Buffer size. */
	caddr_t bufmem;		/* !< Buffer space. */

#ifdef CONFIG_XENO_OPT_PERVASIVE
	pid_t cpid;			/* !< Creator's pid. */
#endif /* CONFIG_XENO_OPT_PERVASIVE */
	xnholder_t rlink;		/* !< Link in resource queue. */
#define rlink2buffer(ln)	container_of(ln, RT_BUFFER, rlink)
	xnqueue_t *rqueue;		/* !< Backpointer to resource queue. */

} RT_BUFFER;

#ifdef __cplusplus
extern "C" {
#endif

#ifdef CONFIG_XENO_OPT_NATIVE_BUFFER

int __native_buffer_pkg_init(void);

void __native_buffer_pkg_cleanup(void);

static inline void __native_buffer_flush_rq(xnqueue_t *rq)
{
	xeno_flush_rq(RT_BUFFER, rq, buffer);
}

struct xnbufd;

ssize_t rt_buffer_read_inner(RT_BUFFER *bf, struct xnbufd *bufd,
			     xntmode_t timeout_mode, RTIME timeout);

ssize_t rt_buffer_write_inner(RT_BUFFER *bf, struct xnbufd *bufd,
			      xntmode_t timeout_mode, RTIME timeout);

#else /* !CONFIG_XENO_OPT_NATIVE_BUFFER */

#define __native_buffer_pkg_init()		({ 0; })
#define __native_buffer_pkg_cleanup()		do { } while(0)
#define __native_buffer_flush_rq(rq)		do { } while(0)

#endif /* !CONFIG_XENO_OPT_NATIVE_BUFFER */

#ifdef __cplusplus
}
#endif

#else /* !(__KERNEL__ || __XENO_SIM__) */

typedef RT_BUFFER_PLACEHOLDER RT_BUFFER;

#ifdef __cplusplus
extern "C" {
#endif

int rt_buffer_bind(RT_BUFFER *bf,
		   const char *name,
		   RTIME timeout);

static inline int rt_buffer_unbind(RT_BUFFER *bf)
{
	bf->opaque = XN_NO_HANDLE;
	return 0;
}

#ifdef __cplusplus
}
#endif

#endif /* __KERNEL__ || __XENO_SIM__ */

#ifdef __cplusplus
extern "C" {
#endif

/* Public interface. */

int rt_buffer_create(RT_BUFFER *bf,
		     const char *name,
		     size_t bufsz,
		     int mode);

int rt_buffer_delete(RT_BUFFER *bf);

ssize_t rt_buffer_write(RT_BUFFER *bf,
			const void *ptr, size_t size,
			RTIME timeout);

ssize_t rt_buffer_write_until(RT_BUFFER *bf,
			      const void *ptr, size_t size,
			      RTIME timeout);

ssize_t rt_buffer_read(RT_BUFFER *bf,
		       void *ptr, size_t size,
		       RTIME timeout);

ssize_t rt_buffer_read_until(RT_BUFFER *bf,
			     void *ptr, size_t size,
			     RTIME timeout);

int rt_buffer_clear(RT_BUFFER *bf);

int rt_buffer_inquire(RT_BUFFER *bf,
		      RT_BUFFER_INFO *info);

#ifdef __cplusplus
}
#endif

#endif /* !_XENO_BUFFER_H */
