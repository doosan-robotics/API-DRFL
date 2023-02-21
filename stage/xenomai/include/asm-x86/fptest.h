/*
 * Copyright (C) 2006-2013 Gilles Chanteperdrix <gch@xenomai.org>.
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

#ifndef _XENO_ASM_X86_FPTEST_H
#define _XENO_ASM_X86_FPTEST_H

#ifdef __KERNEL__
#include <linux/module.h>
#include <asm/i387.h>
#include <asm/processor.h>

#ifndef cpu_has_xmm2
#ifdef cpu_has_sse2
#define cpu_has_xmm2 cpu_has_sse2
#else
#define cpu_has_xmm2 0
#endif
#endif

#ifndef cpu_has_avx
#define cpu_has_avx 0
#endif

static inline int fp_kernel_supported(void)
{
	return 1;
}

static inline int fp_linux_begin(void)
{
#if defined(CONFIG_X86_USE_3DNOW) \
	|| defined(CONFIG_MD_RAID456) || defined(CONFIG_MD_RAID456_MODULE)
	/* Ther kernel uses x86 FPU, we can not also use it in our tests. */
	static int once = 0;
	if (!once) {
		once = 1;
		printk("%s:%d: Warning: Linux is compiled to use FPU in "
		       "kernel-space.\nFor this reason, switchtest can not "
		       "test using FPU in Linux kernel-space.\n",
		       __FILE__, __LINE__);
	}
	return -EBUSY;
#endif /* 3DNow or RAID 456 */
	kernel_fpu_begin();
	/* kernel_fpu_begin() does no re-initialize the fpu context, but
	   fp_regs_set() implicitly expects an initialized fpu context, so
	   initialize it here. */
	__asm__ __volatile__("fninit");
	return 0;
}

static inline void fp_linux_end(void)
{
	kernel_fpu_end();
}

static inline void fp_features_init(void)
{
}
#else /* !__KERNEL__ */
#include <stdio.h>
#define printk printf

#define FP_FEATURE_SSE2			0x01
#define FP_FEATURE_AVX			0x02

static unsigned long fp_features;

#define cpu_has_xmm2 (fp_features & FP_FEATURE_SSE2)
#define cpu_has_avx (fp_features & FP_FEATURE_AVX)

static void fp_features_init(void)
{
	char buffer[1024];
	FILE *f = fopen("/proc/cpuinfo", "r");
	if(!f)
		return;

	while(fgets(buffer, sizeof(buffer), f)) {
		if(strncmp(buffer, "flags", sizeof("flags") - 1))
			continue;

		if (strstr(buffer, "sse2"))
			fp_features |= FP_FEATURE_SSE2;
		if (strstr(buffer, "avx"))
			fp_features |= FP_FEATURE_AVX;
		break;
	}

	fclose(f);
}
#endif /* !__KERNEL__ */

static inline void fp_regs_set(unsigned val)
{
	uint64_t vec[8][4];
	unsigned i;

	for (i = 0; i < 8; i++) {
		__asm__ __volatile__("fildl %0": /* no output */ :"m"(val));
		val++;
	}
	for (i = 0; i < 8; i++) {
		vec[i][0] = val++;
		vec[i][2] = val++;
	}
	if (cpu_has_avx) {
		__asm__ __volatile__(
			"vmovupd %0,%%ymm0;"
			"vmovupd %4,%%ymm1;"
			"vmovupd %8,%%ymm2;"
			"vmovupd %12,%%ymm3;"
			: : "m" (vec[0][0]), "m" (vec[0][1]), "m" (vec[0][2]), "m" (vec[0][3]),
			  "m" (vec[1][0]), "m" (vec[1][1]), "m" (vec[1][2]), "m" (vec[1][3]),
			  "m" (vec[2][0]), "m" (vec[2][1]), "m" (vec[2][2]), "m" (vec[2][3]),
			  "m" (vec[3][0]), "m" (vec[3][1]), "m" (vec[3][2]), "m" (vec[3][3]));
		__asm__ __volatile__(
			"vmovupd %0,%%ymm4;"
			"vmovupd %4,%%ymm5;"
			"vmovupd %8,%%ymm6;"
			"vmovupd %12,%%ymm7;"
			: : "m" (vec[4][0]), "m" (vec[4][1]), "m" (vec[4][2]), "m" (vec[4][3]),
			  "m" (vec[5][0]), "m" (vec[5][1]), "m" (vec[5][2]), "m" (vec[5][3]),
			  "m" (vec[6][0]), "m" (vec[6][1]), "m" (vec[6][2]), "m" (vec[6][3]),
			  "m" (vec[7][0]), "m" (vec[7][1]), "m" (vec[7][2]), "m" (vec[7][3]));

	} else if (cpu_has_xmm2)
		__asm__ __volatile__(
			"movupd %0,%%xmm0;"
			"movupd %2,%%xmm1;"
			"movupd %4,%%xmm2;"
			"movupd %6,%%xmm3;"
			"movupd %8,%%xmm4;"
			"movupd %10,%%xmm5;"
			"movupd %12,%%xmm6;"
			"movupd %14,%%xmm7;"
			: : "m" (vec[0][0]), "m" (vec[0][1]),
			  "m" (vec[1][0]), "m" (vec[1][1]),
			  "m" (vec[2][0]), "m" (vec[2][1]),
			  "m" (vec[3][0]), "m" (vec[3][1]),
			  "m" (vec[4][0]), "m" (vec[4][1]),
			  "m" (vec[5][0]), "m" (vec[5][1]),
			  "m" (vec[6][0]), "m" (vec[6][1]),
			  "m" (vec[7][0]), "m" (vec[7][1]));
}

static inline unsigned fp_regs_check(unsigned val)
{
	unsigned i, result = val;
	unsigned val_offset;
	uint64_t vec[8][4];
	unsigned e[8];

	for (i = 0; i < 8; i++)
		__asm__ __volatile__("fistpl %0":"=m"(e[7 - i]));
	if (cpu_has_avx) {
		__asm__ __volatile__(
			"vmovupd %%ymm0,%0;"
			"vmovupd %%ymm1,%4;"
			"vmovupd %%ymm2,%8;"
			"vmovupd %%ymm3,%12;"
			: "=m" (vec[0][0]), "=m" (vec[0][1]), "=m" (vec[0][2]), "=m" (vec[0][3]),
			  "=m" (vec[1][0]), "=m" (vec[1][1]), "=m" (vec[1][2]), "=m" (vec[1][3]),
			  "=m" (vec[2][0]), "=m" (vec[2][1]), "=m" (vec[2][2]), "=m" (vec[2][3]),
			  "=m" (vec[3][0]), "=m" (vec[3][1]), "=m" (vec[3][2]), "=m" (vec[3][3]));
		__asm__ __volatile__(
			"vmovupd %%ymm4,%0;"
			"vmovupd %%ymm5,%4;"
			"vmovupd %%ymm6,%8;"
			"vmovupd %%ymm7,%12;"
			: "=m" (vec[4][0]), "=m" (vec[4][1]), "=m" (vec[4][2]), "=m" (vec[4][3]),
			  "=m" (vec[5][0]), "=m" (vec[5][1]), "=m" (vec[5][2]), "=m" (vec[5][3]),
			  "=m" (vec[6][0]), "=m" (vec[6][1]), "=m" (vec[6][2]), "=m" (vec[6][3]),
			  "=m" (vec[7][0]), "=m" (vec[7][1]), "=m" (vec[7][2]), "=m" (vec[7][3]));

	} else if (cpu_has_xmm2)
		__asm__ __volatile__(
			"movupd %%xmm0,%0;"
			"movupd %%xmm1,%2;"
			"movupd %%xmm2,%4;"
			"movupd %%xmm3,%6;"
			"movupd %%xmm4,%8;"
			"movupd %%xmm5,%10;"
			"movupd %%xmm6,%12;"
			"movupd %%xmm7,%14;"
			: "=m" (vec[0][0]), "=m" (vec[0][1]),
			  "=m" (vec[1][0]), "=m" (vec[1][1]),
			  "=m" (vec[2][0]), "=m" (vec[2][1]),
			  "=m" (vec[3][0]), "=m" (vec[3][1]),
			  "=m" (vec[4][0]), "=m" (vec[4][1]),
			  "=m" (vec[5][0]), "=m" (vec[5][1]),
			  "=m" (vec[6][0]), "=m" (vec[6][1]),
			  "=m" (vec[7][0]), "=m" (vec[7][1]));

	for (i = 0, val_offset = 0; i < 8; i++, val_offset++)
		if (e[i] != val + val_offset) {
			printk("r%d: %u != %u\n", i, e[i], val + val_offset);
			result = e[i] - val_offset;
		}

	if (cpu_has_avx) {
		for (i = 0; i < 8; i++) {
			int error = 0;
			if (vec[i][0] != val + val_offset) {
				result = vec[i][0] - val_offset;
				error = 1;
			}
			val_offset++;
			if (vec[i][2] != val + val_offset) {
				result = vec[i][2] - val_offset;
				error = 1;
			}
			if (error)
				printk("ymm%d: %llu/%llu != %u/%u\n",
				       i, (unsigned long long)vec[i][0],
				       (unsigned long long)vec[i][2],
				       val + val_offset - 1, val + val_offset);
			val_offset++;
		}
	} else if (cpu_has_xmm2) {
		for (i = 0; i < 8; i++, val_offset += 2)
			if (vec[i][0] != val + val_offset) {
				printk("xmm%d: %llu != %u\n",
				       i, (unsigned long long)vec[i][0], val + val_offset);
				result = vec[i][0] - val_offset;
			}
	}

	return result;
}

#endif /* _XENO_ASM_X86_FPTEST_H */
