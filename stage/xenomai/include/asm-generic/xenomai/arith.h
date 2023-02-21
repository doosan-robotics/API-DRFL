/**
 *   @ingroup hal
 *   @file
 *
 *   Generic arithmetic/conversion routines.
 *   Copyright &copy; 2005 Stelian Pop.
 *   Copyright &copy; 2005 Gilles Chanteperdrix.
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, Inc., 675 Mass Ave, Cambridge MA 02139,
 *   USA; either version 2 of the License, or (at your option) any later
 *   version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

/**
 * @addtogroup hal
 *@{*/

#ifndef _XENO_ASM_GENERIC_ARITH_H
#define _XENO_ASM_GENERIC_ARITH_H

#ifdef __KERNEL__
#include <asm/byteorder.h>
#include <asm/div64.h>

#ifdef __BIG_ENDIAN
#define endianstruct struct { unsigned _h; unsigned _l; } _s
#else /* __LITTLE_ENDIAN */
#define endianstruct struct { unsigned _l; unsigned _h; } _s
#endif

#else /* !__KERNEL__ */
#include <stddef.h>
#include <endian.h>

#if __BYTE_ORDER == __BIG_ENDIAN
#define endianstruct struct { unsigned _h; unsigned _l; } _s
#else /* __BYTE_ORDER == __LITTLE_ENDIAN */
#define endianstruct struct { unsigned _l; unsigned _h; } _s
#endif /* __BYTE_ORDER == __LITTLE_ENDIAN */

static inline unsigned __rthal_do_div(unsigned long long *a, unsigned d)
{
	unsigned r = *a % d;
	*a /= d;
	return r;
}

#define do_div(a, d) __rthal_do_div(&(a), (d))

#endif /* !__KERNEL__ */

#ifndef __rthal_u64tou32
#define __rthal_u64tou32(ull, h, l) ({          \
    union { unsigned long long _ull;            \
    endianstruct;                               \
    } _u;                                       \
    _u._ull = (ull);                            \
    (h) = _u._s._h;                             \
    (l) = _u._s._l;                             \
})
#endif /* !__rthal_u64tou32 */

#ifndef __rthal_u64fromu32
#define __rthal_u64fromu32(h, l) ({             \
    union { unsigned long long _ull;            \
    endianstruct;                               \
    } _u;                                       \
    _u._s._h = (h);                             \
    _u._s._l = (l);                             \
    _u._ull;                                    \
})
#endif /* !__rthal_u64fromu32 */

#ifndef rthal_ullmul
static inline __attribute__((__const__)) unsigned long long
__rthal_generic_ullmul(const unsigned m0, const unsigned m1)
{
    return (unsigned long long) m0 * m1;
}
#define rthal_ullmul(m0,m1) __rthal_generic_ullmul((m0),(m1))
#endif /* !rthal_ullmul */

#ifndef rthal_ulldiv
static inline unsigned long long __rthal_generic_ulldiv (unsigned long long ull,
							 const unsigned uld,
							 unsigned long *const rp)
{
    const unsigned r = do_div(ull, uld);

    if (rp)
	*rp = r;

    return ull;
}
#define rthal_ulldiv(ull,uld,rp) __rthal_generic_ulldiv((ull),(uld),(rp))
#endif /* !rthal_ulldiv */

#ifndef rthal_uldivrem
#define rthal_uldivrem(ull,ul,rp) ((unsigned) rthal_ulldiv((ull),(ul),(rp)))
#endif /* !rthal_uldivrem */

#ifndef rthal_divmod64
static inline unsigned long long
__rthal_generic_divmod64(unsigned long long a,
			 unsigned long long b,
			 unsigned long long *rem)
{
	unsigned long long q;
#if defined(__KERNEL__) && BITS_PER_LONG < 64
	if (b <= 0xffffffffULL) {
		unsigned long r;
		q = rthal_ulldiv(a, b, &r);
		if (rem)
			*rem = r;
	} else {
		extern unsigned long long
			__rthal_generic_full_divmod64(unsigned long long a,
						      unsigned long long b,
						      unsigned long long *rem);
		if (a < b) {
			if (rem)
				*rem = a;
			return 0;
		}

		return __rthal_generic_full_divmod64(a, b, rem);
	}
#else /* BITS_PER_LONG >= 64 */
	q = a / b;
	if (rem)
		*rem = a % b;
#endif /* BITS_PER_LONG < 64 */
	return q;
}
#define rthal_divmod64(a,b,rp) __rthal_generic_divmod64((a),(b),(rp))
#endif /* !rthal_divmod64 */

#ifndef rthal_imuldiv
static inline __attribute__((__const__)) int __rthal_generic_imuldiv(int i,
								     int mult,
								     int div)
{
    /* Returns (int)i = (unsigned long long)i*(unsigned)(mult)/(unsigned)div. */
    const unsigned long long ull = rthal_ullmul(i, mult);
    return rthal_uldivrem(ull, div, NULL);
}
#define rthal_imuldiv(i,m,d) __rthal_generic_imuldiv((i),(m),(d))
#endif /* !rthal_imuldiv */

#ifndef rthal_imuldiv_ceil
static inline __attribute__((__const__)) int __rthal_generic_imuldiv_ceil(int i,
									  int mult,
									  int div)
{
	/* Same as __rthal_generic_imuldiv, rounding up. */
	const unsigned long long ull = rthal_ullmul(i, mult);
	return rthal_uldivrem(ull + (unsigned)div - 1, div, NULL);
}
#define rthal_imuldiv_ceil(i,m,d) __rthal_generic_imuldiv_ceil((i),(m),(d))
#endif /* !rthal_imuldiv_ceil */

/* Division of an unsigned 96 bits ((h << 32) + l) by an unsigned 32 bits.
   Building block for llimd. Without const qualifiers, gcc reload registers
   after each call to uldivrem. */
static inline unsigned long long
__rthal_generic_div96by32 (const unsigned long long h,
			   const unsigned l,
			   const unsigned d,
			   unsigned long *const rp)
{
    unsigned long rh;
    const unsigned qh = rthal_uldivrem(h, d, &rh);
    const unsigned long long t = __rthal_u64fromu32(rh, l);
    const unsigned ql = rthal_uldivrem(t, d, rp);

    return __rthal_u64fromu32(qh, ql);
}

#ifndef rthal_llimd
static inline __attribute__((__const__))
unsigned long long __rthal_generic_ullimd (const unsigned long long op,
					   const unsigned m,
					   const unsigned d)
{
    unsigned oph, opl, tlh, tll;
    unsigned long long th, tl;

    __rthal_u64tou32(op, oph, opl);
    tl = rthal_ullmul(opl, m);
    __rthal_u64tou32(tl, tlh, tll);
    th = rthal_ullmul(oph, m);
    th += tlh;

    return __rthal_generic_div96by32(th, tll, d, NULL);
}

static inline __attribute__((__const__)) long long
__rthal_generic_llimd (long long op, unsigned m, unsigned d)
{
	long long ret;
	int sign = 0;

	if(op < 0LL) {
		sign = 1;
		op = -op;
	}
	ret = __rthal_generic_ullimd(op, m, d);

	return sign ? -ret : ret;
}
#define rthal_llimd(ll,m,d) __rthal_generic_llimd((ll),(m),(d))
#endif /* !rthal_llimd */

#ifndef __rthal_u96shift
#define __rthal_u96shift(h, m, l, s) ({		\
	unsigned _l = (l);			\
	unsigned _m = (m);			\
	unsigned _s = (s);			\
	_l >>= _s;				\
	_l |= (_m << (32 - _s));		\
	_m >>= _s;				\
	_m |= ((h) << (32 - _s));		\
	__rthal_u64fromu32(_m, _l);		\
})
#endif /* !__rthal_u96shift */

static inline long long rthal_llmi(int i, int j)
{
	/* Signed fast 32x32->64 multiplication */
	return (long long) i * j;
}

#ifndef rthal_llmulshft
/* Fast scaled-math-based replacement for long long multiply-divide */
static inline long long
__rthal_generic_llmulshft(const long long op,
			  const unsigned m,
			  const unsigned s)
{
	unsigned oph, opl, tlh, tll, thh, thl;
	unsigned long long th, tl;

	__rthal_u64tou32(op, oph, opl);
	tl = rthal_ullmul(opl, m);
	__rthal_u64tou32(tl, tlh, tll);
	th = rthal_llmi(oph, m);
	th += tlh;
	__rthal_u64tou32(th, thh, thl);

	return __rthal_u96shift(thh, thl, tll, s);
}
#define rthal_llmulshft(ll, m, s) __rthal_generic_llmulshft((ll), (m), (s))
#endif /* !rthal_llmulshft */

#ifdef XNARCH_HAVE_NODIV_LLIMD

/* Representation of a 32 bits fraction. */
typedef struct {
	unsigned long long frac;
	unsigned integ;
} rthal_u32frac_t;

static inline void xnarch_init_u32frac(rthal_u32frac_t *const f,
				       const unsigned m,
				       const unsigned d)
{
	/* Avoid clever compiler optimizations to occur when d is
	   known at compile-time. The performance of this function is
	   not critical since it is only called at init time. */
	volatile unsigned vol_d = d;
	f->integ = m / d;
	f->frac = __rthal_generic_div96by32
		(__rthal_u64fromu32(m % d, 0), 0, vol_d, NULL);
}

#ifndef rthal_nodiv_imuldiv
static inline __attribute__((__const__)) unsigned
rthal_generic_nodiv_imuldiv(unsigned op, const rthal_u32frac_t f)
{
	return (rthal_ullmul(op, f.frac >> 32) >> 32) + f.integ * op;
}
#define rthal_nodiv_imuldiv(op, f) rthal_generic_nodiv_imuldiv((op),(f))
#endif /* rthal_nodiv_imuldiv */

#ifndef rthal_nodiv_imuldiv_ceil
static inline __attribute__((__const__)) unsigned
rthal_generic_nodiv_imuldiv_ceil(unsigned op, const rthal_u32frac_t f)
{
	unsigned long long full = rthal_ullmul(op, f.frac >> 32) + ~0U;
	return (full >> 32) + f.integ * op;
}
#define rthal_nodiv_imuldiv_ceil(op, f) \
	rthal_generic_nodiv_imuldiv_ceil((op),(f))
#endif /* rthal_nodiv_imuldiv */

#ifndef rthal_nodiv_ullimd

#ifndef __rthal_add96and64
#error "__rthal_add96and64 must be implemented."
#endif

static inline __attribute__((__const__)) unsigned long long
__rthal_mul64by64_high(const unsigned long long op, const unsigned long long m)
{
    /* Compute high 64 bits of multiplication 64 bits x 64 bits. */
    register unsigned long long t0, t1, t2, t3;
    register unsigned oph, opl, mh, ml, t0h, t0l, t1h, t1l, t2h, t2l, t3h, t3l;

    __rthal_u64tou32(op, oph, opl);
    __rthal_u64tou32(m, mh, ml);
    t0 = rthal_ullmul(opl, ml);
    __rthal_u64tou32(t0, t0h, t0l);
    t3 = rthal_ullmul(oph, mh);
    __rthal_u64tou32(t3, t3h, t3l);
    __rthal_add96and64(t3h, t3l, t0h, 0, t0l >> 31);
    t1 = rthal_ullmul(oph, ml);
    __rthal_u64tou32(t1, t1h, t1l);
    __rthal_add96and64(t3h, t3l, t0h, t1h, t1l);
    t2 = rthal_ullmul(opl, mh);
    __rthal_u64tou32(t2, t2h, t2l);
    __rthal_add96and64(t3h, t3l, t0h, t2h, t2l);

    return __rthal_u64fromu32(t3h, t3l);
}

static inline unsigned long long
__rthal_generic_nodiv_ullimd(const unsigned long long op,
			     const unsigned long long frac,
			     unsigned integ)
{
	return __rthal_mul64by64_high(op, frac) + integ * op;
}
#define rthal_nodiv_ullimd(op, f, i)  __rthal_generic_nodiv_ullimd((op),(f), (i))
#endif /* !rthal_nodiv_ullimd */

#ifndef rthal_nodiv_llimd
static inline __attribute__((__const__)) long long
__rthal_generic_nodiv_llimd (long long op, unsigned long long frac, unsigned integ)
{
	long long ret;
	int sign = 0;

	if(op < 0LL) {
		sign = 1;
		op = -op;
	}
	ret = rthal_nodiv_ullimd(op, frac, integ);

	return sign ? -ret : ret;
}
#define rthal_nodiv_llimd(ll,frac,integ) __rthal_generic_nodiv_llimd((ll),(frac),(integ))
#endif /* !rthal_nodiv_llimd */

#endif /* XNARCH_HAVE_NODIV_LLIMD */

static inline void xnarch_init_llmulshft(const unsigned m_in,
					 const unsigned d_in,
					 unsigned *m_out,
					 unsigned *s_out)
{
	/* Avoid clever compiler optimizations to occur when d is
	   known at compile-time. The performance of this function is
	   not critical since it is only called at init time. */
	volatile unsigned vol_d = d_in;
	unsigned long long mult;

	*s_out = 31;
	while (1) {
		mult = ((unsigned long long)m_in) << *s_out;
		do_div(mult, vol_d);
		if (mult <= 0x7FFFFFFF)
			break;
		(*s_out)--;
	}
	*m_out = (unsigned)mult;
}

#define xnarch_ullmod(ull,uld,rem)   ({ xnarch_ulldiv(ull,uld,rem); (*rem); })
#define xnarch_uldiv(ull, d)         rthal_uldivrem(ull, d, NULL)
#define xnarch_ulmod(ull, d)         ({ u_long _rem;                    \
					rthal_uldivrem(ull,d,&_rem); _rem; })

#define xnarch_ullmul                rthal_ullmul
#define xnarch_uldivrem              rthal_uldivrem
#define xnarch_ulldiv                rthal_ulldiv
#define xnarch_divmod64              rthal_divmod64
#define xnarch_div64(a,b)            rthal_divmod64((a),(b),NULL)
#define xnarch_mod64(a,b)            ({ unsigned long long _rem; \
					rthal_divmod64((a),(b),&_rem); _rem; })
#define xnarch_imuldiv               rthal_imuldiv
#define xnarch_imuldiv_ceil          rthal_imuldiv_ceil
#define xnarch_llimd                 rthal_llimd
#define xnarch_nodiv_ullimd          rthal_nodiv_ullimd
#define xnarch_nodiv_llimd           rthal_nodiv_llimd
#define xnarch_llmulshft             rthal_llmulshft

unsigned long long xnarch_divrem_billion(unsigned long long value,
					 unsigned long *rem);

/*@}*/

#endif /* _XENO_ASM_GENERIC_ARITH_H */
