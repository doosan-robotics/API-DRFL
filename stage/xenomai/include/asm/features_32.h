/*
 * Copyright (C) 2005 Philippe Gerum <rpm@xenomai.org>.
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

#ifndef _XENO_ASM_X86_FEATURES_32_H
#define _XENO_ASM_X86_FEATURES_32_H
#define _XENO_ASM_X86_FEATURES_H

#include <asm-generic/xenomai/features.h>

#ifdef __KERNEL__
#include <asm/xenomai/wrappers.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
/*
 * The kernel will deal dynamically with the actual SEP support
 * whether the CPU provides it or not; we just need to make sure that
 * VSYSCALL is supported by the current kernel here. The user-space
 * side extracts this information from the xeno_config.h header
 * included from asm-generic/xenomai/features.h.
 */
#define CONFIG_XENO_X86_SEP  1
#endif /* KERNEL_VERSION >= 2.6.0 */
#else /* !__KERNEL__ */
#define cpu_has_tsc 1
#define IPIPE_CORE_APIREV 0
#endif /* !__KERNEL__ */

#define __xn_feat_x86_sep 0x00000001
#define __xn_feat_x86_tsc 0x00000002

/* The ABI revision level we use on this arch. */
#define XENOMAI_ABI_REV   4UL

#if defined(CONFIG_X86_TSC) || IPIPE_CORE_APIREV >= 2
#define __xn_feat_x86_tsc_mask (cpu_has_tsc ? __xn_feat_x86_tsc : 0)
#define XNARCH_HAVE_NONPRIV_TSC  1
#else
#define __xn_feat_x86_tsc_mask   0
#endif

#ifdef CONFIG_XENO_X86_SEP
#define __xn_feat_x86_sep_mask  __xn_feat_x86_sep
#else
#define __xn_feat_x86_sep_mask   0
#endif

#define XENOMAI_FEAT_DEP  (__xn_feat_generic_mask| \
			   __xn_feat_x86_sep_mask| \
			   __xn_feat_x86_tsc_mask)

#define XENOMAI_FEAT_MAN  (__xn_feat_generic_man_mask| \
			   __xn_feat_x86_sep| \
			   __xn_feat_x86_tsc)

static inline int check_abi_revision(unsigned long abirev)
{
    return abirev == XENOMAI_ABI_REV;
}

static inline const char *get_feature_label (unsigned feature)
{
    switch (feature) {
    	case __xn_feat_x86_sep:
	    return "sep";
    	case __xn_feat_x86_tsc:
	    return "tsc";
    	default:
	    return get_generic_feature_label(feature);
    }
}

#define XNARCH_HAVE_LLMULSHFT    1
#define XNARCH_HAVE_NODIV_LLIMD  1

#endif /* !_XENO_ASM_X86_FEATURES_32_H */
