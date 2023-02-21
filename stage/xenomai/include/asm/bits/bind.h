/*
 * Copyright (C) 2010-2013 Gilles Chanteperdrix <gch@xenomai.org>.
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

#ifndef _XENO_ASM_X86_BIND_H
#define _XENO_ASM_X86_BIND_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <asm-generic/xenomai/bind.h>
#include <asm/xenomai/features.h>

#ifdef __i386__
static inline void xeno_x86_features_check(struct xnfeatinfo *finfo)
{
#ifdef CONFIG_XENO_X86_SEP
	size_t n = confstr(_CS_GNU_LIBPTHREAD_VERSION, NULL, 0);
	if (n > 0) {
		char buf[n];

		confstr (_CS_GNU_LIBPTHREAD_VERSION, buf, n);

		if (strstr (buf, "NPTL"))
			return;
	}

	fprintf(stderr,
"Xenomai: --enable-x86-sep needs NPTL and Linux 2.6.x or higher,\n"
"which does not match your configuration. Please upgrade, or\n"
"rebuild the user-space support passing --disable-x86-sep.\n");
	exit(1);
#endif /* CONFIG_XENO_X86_SEP */
}
#define xeno_arch_features_check(finfo) xeno_x86_features_check(finfo)

#endif /* __i386__ */

#endif /* _XENO_ASM_X86_BIND_H */
