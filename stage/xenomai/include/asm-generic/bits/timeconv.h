/*
 * Copyright (C) 2009 Philippe Gerum <rpm@xenomai.org>.
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

#ifndef _XENO_ASM_GENERIC_BITS_TIMECONV_H
#define _XENO_ASM_GENERIC_BITS_TIMECONV_H

#include <asm/xenomai/arith.h>

static unsigned long long clockfreq;

#ifdef XNARCH_HAVE_LLMULSHFT
static unsigned int tsc_scale, tsc_shift;
#ifdef XNARCH_HAVE_NODIV_LLIMD
static rthal_u32frac_t tsc_frac;
static rthal_u32frac_t bln_frac;
#endif
#endif

#ifdef XNARCH_HAVE_LLMULSHFT
long long xnarch_tsc_to_ns(long long ticks)
{
	return xnarch_llmulshft(ticks, tsc_scale, tsc_shift);
}

long long xnarch_tsc_to_ns_rounded(long long ticks)
{
	unsigned int shift = tsc_shift - 1;
	return (xnarch_llmulshft(ticks, tsc_scale, shift) + 1) / 2;
}

#ifdef XNARCH_HAVE_NODIV_LLIMD
long long xnarch_ns_to_tsc(long long ns)
{
	return xnarch_nodiv_llimd(ns, tsc_frac.frac, tsc_frac.integ);
}

unsigned long long xnarch_divrem_billion(unsigned long long value,
					 unsigned long *rem)
{
	unsigned long long q;
	unsigned r;

	q = xnarch_nodiv_ullimd(value, bln_frac.frac, bln_frac.integ);
	r = value - q * 1000000000;
	if (r >= 1000000000) {
		++q;
		r -= 1000000000;
	}
	*rem = r;
	return q;
}
#else /* !XNARCH_HAVE_NODIV_LLIMD */
long long xnarch_ns_to_tsc(long long ns)
{
	return xnarch_llimd(ns, 1 << tsc_shift, tsc_scale);
}
#endif /* !XNARCH_HAVE_NODIV_LLIMD */

#else  /* !XNARCH_HAVE_LLMULSHFT */

long long xnarch_tsc_to_ns(long long ticks)
{
	return xnarch_llimd(ticks, 1000000000, clockfreq);
}

long long xnarch_tsc_to_ns_rounded(long long ticks)
{
	return (xnarch_llimd(ticks, 1000000000, clockfreq/2) + 1) / 2;
}

long long xnarch_ns_to_tsc(long long ns)
{
	return xnarch_llimd(ns, clockfreq, 1000000000);
}
#endif /* !XNARCH_HAVE_LLMULSHFT */

#ifndef XNARCH_HAVE_NODIV_LLIMD
unsigned long long xnarch_divrem_billion(unsigned long long value,
					 unsigned long *rem)
{
	return xnarch_ulldiv(value, 1000000000, rem);

}
#endif /* !XNARCH_HAVE_NODIV_LLIMD */

static void xnarch_init_timeconv(unsigned long long freq)
{
	clockfreq = freq;
#ifdef XNARCH_HAVE_LLMULSHFT
	xnarch_init_llmulshft(1000000000, freq, &tsc_scale, &tsc_shift);
#ifdef XNARCH_HAVE_NODIV_LLIMD
	xnarch_init_u32frac(&tsc_frac, 1 << tsc_shift, tsc_scale);
	xnarch_init_u32frac(&bln_frac, 1, 1000000000);
#endif
#endif
}

#ifdef __KERNEL__
EXPORT_SYMBOL_GPL(xnarch_tsc_to_ns);
EXPORT_SYMBOL_GPL(xnarch_ns_to_tsc);
EXPORT_SYMBOL_GPL(xnarch_divrem_billion);
#endif /* __KERNEL__ */

#endif /* !_XENO_ASM_GENERIC_BITS_TIMECONV_H */
