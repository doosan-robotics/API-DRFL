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

#ifndef SYSLOG_H
#define SYSLOG_H

#pragma GCC system_header

#include <stdarg.h>
#include_next <syslog.h>
#include <xeno_config.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void __real_syslog(int priority, const char *fmt, ...);

void __real_vsyslog(int priority, const char *fmt, va_list ap);

#ifdef CONFIG_XENO_FORTIFY
void __real___vsyslog_chk(int priority, int level, const char *fmt, va_list ap);
#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* SYSLOG_H */
