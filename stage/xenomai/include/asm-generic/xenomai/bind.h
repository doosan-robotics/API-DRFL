/*
 * Copyright (C) 2006 Philippe Gerum <rpm@xenomai.org>.
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

#ifndef _XENO_ASM_GENERIC_BITS_BIND_H
#define _XENO_ASM_GENERIC_BITS_BIND_H

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <nucleus/compiler.h>	/* For __constructor__ */

int 
xeno_bind_skin_opt(unsigned skin_magic, const char *skin, const char *module);

static inline int 
xeno_bind_skin(unsigned skin_magic, const char *skin, const char *module)
{
	int muxid = xeno_bind_skin_opt(skin_magic, skin, module);

	if (muxid == -1) {
		fprintf(stderr,
			"Xenomai: %s skin or CONFIG_XENO_OPT_PERVASIVE disabled.\n"
			"(modprobe %s?)\n", skin, module);
		exit(EXIT_FAILURE);
	}

	return muxid;
}

#endif /* _XENO_ASM_GENERIC_BITS_BIND_H */
