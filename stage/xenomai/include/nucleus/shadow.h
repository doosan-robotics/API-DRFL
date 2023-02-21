/*
 * Copyright (C) 2001,2002,2003 Philippe Gerum <rpm@xenomai.org>.
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

#ifndef _XENO_NUCLEUS_SHADOW_H
#define _XENO_NUCLEUS_SHADOW_H

#include <asm/xenomai/atomic.h>

#ifdef CONFIG_XENO_OPT_PERVASIVE

#include <asm/xenomai/syscall.h>

#define XENOMAI_MUX_NR 16

/* Events sent to the interface callback */
#define XNSHADOW_CLIENT_ATTACH  0
#define XNSHADOW_CLIENT_DETACH  1

#ifdef __cplusplus
extern "C" {
#endif

struct xnthread;
struct xnmutex;
struct pt_regs;
struct timespec;
struct timeval;
struct xntbase;

struct xnskin_props {
	const char *name;
	unsigned magic;
	int nrcalls;
	void *(*eventcb)(int, void *);
	xnsysent_t *systab;
	struct xntbase **timebasep;
	struct module *module;
};

int xnshadow_mount(void);

void xnshadow_cleanup(void);

void xnshadow_grab_events(void);

void xnshadow_release_events(void);

int xnshadow_map(struct xnthread *curr,
		 xncompletion_t __user *u_completion,
		 unsigned long __user *u_mode_offset);

void xnshadow_unmap(struct xnthread *thread);

int xnshadow_harden(void);

void xnshadow_relax(int notify, int reason);

void xnshadow_renice(struct xnthread *thread);

void xnshadow_suspend(struct xnthread *thread);

int xnshadow_wait_barrier(struct pt_regs *regs);

void xnshadow_start(struct xnthread *thread);

void xnshadow_signal_completion(xncompletion_t __user *u_completion,
				int err);

void xnshadow_exit(void);

int xnshadow_register_interface(struct xnskin_props *props);

int xnshadow_unregister_interface(int muxid);

void xnshadow_reset_shield(void);

void xnshadow_send_sig(struct xnthread *thread,
		       int sig,
		       int arg,
		       int specific);

void xnshadow_rpi_check(void);

#ifdef RTHAL_HAVE_RETURN_EVENT
#define XNARCH_HAVE_MAYDAY  1
void xnshadow_call_mayday(struct xnthread *thread);
#else
static inline void xnshadow_call_mayday(struct xnthread *thread)
{
	/* no luck, I-pipe too old. Nobody hears you screaming... */
}
#endif

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_XENO_OPT_PERVASIVE */

#if defined(CONFIG_XENO_OPT_PERVASIVE) && defined(CONFIG_XENO_OPT_VFILE)
void xnshadow_init_proc(void);
void xnshadow_cleanup_proc(void);
#else
static inline void xnshadow_init_proc(void) { }
static inline void xnshadow_cleanup_proc(void) { }
#endif /* CONFIG_XENO_OPT_PERVASIVE && CONFIG_XENO_OPT_VFILE */

#endif /* !_XENO_NUCLEUS_SHADOW_H */
