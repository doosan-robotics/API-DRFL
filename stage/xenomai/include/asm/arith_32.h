/**
 *   @ingroup hal
 *   @file
 *
 *   Arithmetic/conversion routines for x86.
 *
 *   Original RTAI/x86 HAL services from: \n
 *   Copyright &copy; 2000 Paolo Mantegazza, \n
 *   Copyright &copy; 2000 Steve Papacharalambous, \n
 *   Copyright &copy; 2000 Stuart Hughes, \n
 *   and others.
 *
 *   RTAI/x86 rewrite over Adeos: \n
 *   Copyright &copy; 2002,2003 Philippe Gerum.
 *   Major refactoring for Xenomai: \n
 *   Copyright &copy; 2004,2005 Philippe Gerum.
 *   Arithmetic/conversion routines: \n
 *   Copyright &copy; 2005 Gilles Chanteperdrix.
 *
 *   Xenomai is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU General Public License as
 *   published by the Free Software Foundation, Inc., 675 Mass Ave,
 *   Cambridge MA 02139, USA; either version 2 of the License, or (at
 *   your option) any later version.
 *
 *   Xenomai is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *   General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with Xenomai; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
 *   02111-1307, USA.
 */

#ifndef _XENO_ASM_X86_ARITH_32_H
#define _XENO_ASM_X86_ARITH_32_H
#define _XENO_ASM_X86_ARITH_H

#include <asm/xenomai/features.h>

#define __rthal_u64tou32(ull, h, l) ({          \
    unsigned long long _ull = (ull);            \
    (l) = _ull & 0xffffffff;                    \
    (h) = _ull >> 32;                           \
})

#define __rthal_u64fromu32(h, l) ({             \
    unsigned long long _ull;                    \
    asm ( "": "=A"(_ull) : "d"(h), "a"(l));     \
    _ull;                                       \
})

/* const helper for rthal_uldivrem, so that the compiler will eliminate
   multiple calls with same arguments, at no additionnal cost. */
static inline __attribute__((__const__)) unsigned long long
__rthal_uldivrem(const unsigned long long ull, const unsigned long d)
{
    unsigned long long ret;
    __asm__ ("divl %1" : "=A,A"(ret) : "r,?m"(d), "A,A"(ull));
    /* Exception if quotient does not fit on unsigned long. */
    return ret;
}

/* Fast long long division: when the quotient and remainder fit on 32 bits. */
static inline unsigned long __rthal_i386_uldivrem(unsigned long long ull,
						  const unsigned d,
						  unsigned long *const rp)
{
    unsigned long q, r;
    ull = __rthal_uldivrem(ull, d);
    __asm__ ( "": "=d"(r), "=a"(q) : "A"(ull));
    if(rp)
	*rp = r;
    return q;
}
#define rthal_uldivrem(ull, d, rp) __rthal_i386_uldivrem((ull),(d),(rp))

/* Division of an unsigned 96 bits ((h << 32) + l) by an unsigned 32 bits.
   Building block for ulldiv. */
static inline unsigned long long __rthal_div96by32 (const unsigned long long h,
						    const unsigned long l,
						    const unsigned long d,
						    unsigned long *const rp)
{
    u_long rh;
    const u_long qh = rthal_uldivrem(h, d, &rh);
    const unsigned long long t = __rthal_u64fromu32(rh, l);
    const u_long ql = rthal_uldivrem(t, d, rp);

    return __rthal_u64fromu32(qh, ql);
}

/* Slow long long division. Uses rthal_uldivrem, hence has the same property:
   the compiler removes redundant calls. */
static inline unsigned long long
__rthal_i386_ulldiv (const unsigned long long ull,
		     const unsigned d,
		     unsigned long *const rp)
{
    unsigned long h, l;
    __rthal_u64tou32(ull, h, l);
    return __rthal_div96by32(h, l, d, rp);
}
#define rthal_ulldiv(ull,d,rp) __rthal_i386_ulldiv((ull),(d),(rp))

/* Fast scaled-math-based replacement for long long multiply-divide */
#define rthal_llmulshft(ll, m, s)					\
({									\
	long long __ret;						\
	unsigned __lo, __hi;						\
									\
	__asm__ (							\
		/* HI = HIWORD(ll) * m */				\
		"mov  %%eax,%%ecx\n\t"					\
		"mov  %%edx,%%eax\n\t"					\
		"imull %[__m]\n\t"					\
		"mov  %%eax,%[__lo]\n\t"				\
		"mov  %%edx,%[__hi]\n\t"				\
									\
		/* LO = LOWORD(ll) * m */				\
		"mov  %%ecx,%%eax\n\t"					\
		"mull %[__m]\n\t"					\
									\
		/* ret = (HI << 32) + LO */				\
		"add  %[__lo],%%edx\n\t"				\
		"adc  $0,%[__hi]\n\t"					\
									\
		/* ret = ret >> s */					\
		"mov  %[__s],%%ecx\n\t"					\
		"shrd %%cl,%%edx,%%eax\n\t"				\
		"shrd %%cl,%[__hi],%%edx\n\t"				\
		: "=A" (__ret), [__lo] "=&r" (__lo), [__hi] "=&r" (__hi)	\
		: "A" (ll), [__m] "m" (m), [__s] "m" (s)		\
		: "ecx");						\
	__ret;								\
})

static inline __attribute__((const)) unsigned long long
__rthal_x86_nodiv_ullimd(const unsigned long long op,
			 const unsigned long long frac,
			 unsigned rhs_integ)
{
	register unsigned rl __asm__("ecx");
	register unsigned rm __asm__("esi");
	register unsigned rh __asm__("edi");
	unsigned fracl, frach, opl, oph;
	volatile unsigned integ = rhs_integ;
	register unsigned long long t;

	__rthal_u64tou32(op, oph, opl);
	__rthal_u64tou32(frac, frach, fracl);

	__asm__ ("mov %[oph], %%eax\n\t"
		 "mull %[frach]\n\t"
		 "mov %%eax, %[rm]\n\t"
		 "mov %%edx, %[rh]\n\t"
		 "mov %[opl], %%eax\n\t"
		 "mull %[fracl]\n\t"
		 "mov %%edx, %[rl]\n\t"
		 "shl $1, %%eax\n\t"
		 "adc $0, %[rl]\n\t"
		 "adc $0, %[rm]\n\t"
		 "adc $0, %[rh]\n\t"
		 "mov %[oph], %%eax\n\t"
		 "mull %[fracl]\n\t"
		 "add %%eax, %[rl]\n\t"
		 "adc %%edx, %[rm]\n\t"
		 "adc $0, %[rh]\n\t"
		 "mov %[opl], %%eax\n\t"
		 "mull %[frach]\n\t"
		 "add %%eax, %[rl]\n\t"
		 "adc %%edx, %[rm]\n\t"
		 "adc $0, %[rh]\n\t"
		 "mov %[opl], %%eax\n\t"
		 "mull %[integ]\n\t"
		 "add %[rm], %%eax\n\t"
		 "adc %%edx, %[rh]\n\t"
		 "mov %[oph], %%edx\n\t"
		 "imul %[integ], %%edx\n\t"
		 "add %[rh], %%edx\n\t"
		 : [rl]"=&c"(rl), [rm]"=&S"(rm), [rh]"=&D"(rh), "=&A"(t)
		 : [opl]"m"(opl), [oph]"m"(oph),
		   [fracl]"m"(fracl), [frach]"m"(frach), [integ]"m"(integ)
		 : "cc");

	return t;
}

#define rthal_nodiv_ullimd(op, frac, integ) \
	__rthal_x86_nodiv_ullimd((op), (frac), (integ))

#include <asm-generic/xenomai/arith.h>

#endif /* _XENO_ASM_X86_ARITH_32_H */
