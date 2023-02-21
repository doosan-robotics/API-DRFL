/*
 * Copyright (C) 2010 Jan Kiszka <jan.kiszka@siemens.com>.
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

#ifndef _XENO_ASM_GENERIC_TIMECONV_H
#define _XENO_ASM_GENERIC_TIMECONV_H

#ifndef __KERNEL__
extern xnsysinfo_t sysinfo;

void xeno_init_timeconv(int muxid);
#endif

long long xnarch_tsc_to_ns(long long ticks);
long long xnarch_tsc_to_ns_rounded(long long ticks);
long long xnarch_ns_to_tsc(long long ns);

#endif /* !_XENO_ASM_GENERIC_TIMECONV_H */
