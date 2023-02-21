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

#ifndef __KERNEL__

#pragma GCC system_header

#include_next <stdio.h>

#ifndef _XENO_POSIX_STDIO_H
#define _XENO_POSIX_STDIO_H

#include <stddef.h>
#include <stdarg.h>
#include <xeno_config.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

int __real_vfprintf(FILE *stream, const char *fmt, va_list args);

#ifdef CONFIG_XENO_FORTIFY
int __real___vfprintf_chk(FILE *stream, int level, const char *fmt, va_list ap);
#endif

int __real_vprintf(const char *fmt, va_list args);

int __real_fprintf(FILE *stream, const char *fmt, ...);

int __real_printf(const char *fmt, ...);

int __real_puts(const char *s);

int __real_fputs(const char *s, FILE *stream);

int __real_fputc(int c, FILE *stream);

int __real_putchar(int c);

size_t __real_fwrite(const void *ptr, size_t sz, size_t nmemb, FILE *stream);

int __real_fclose(FILE *stream);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _XENO_POSIX_STDIO_H */

#endif /* !__KERNEL__ */
