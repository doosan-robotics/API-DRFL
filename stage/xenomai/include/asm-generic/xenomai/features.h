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

#ifndef _XENO_ASM_GENERIC_FEATURES_H
#define _XENO_ASM_GENERIC_FEATURES_H

#ifdef __KERNEL__
#include <linux/version.h>
#ifndef xnfeatinfo_archdep
#define collect_arch_features(finfo) do { (void)(finfo); } while (0)
#endif /* !xnfeatinfo_archdep */
#else /* !__KERNEL__ */
#include <xeno_config.h>
#endif /* __KERNEL__ */

#define XNFEAT_STRING_LEN 64

typedef struct xnfeatinfo {
	unsigned long feat_all;	/* Available feature set. */
	char feat_all_s[XNFEAT_STRING_LEN];
	unsigned long feat_man;	/* Mandatory features (when requested). */
	char feat_man_s[XNFEAT_STRING_LEN];
	unsigned long feat_req;	/* Requested feature set. */
	char feat_req_s[XNFEAT_STRING_LEN];
	unsigned long feat_mis;	/* Missing features. */
	char feat_mis_s[XNFEAT_STRING_LEN];
#ifdef xnfeatinfo_archdep
	struct xnfeatinfo_archdep feat_arch; /* Arch-dep extension. */
#endif
	unsigned long feat_abirev; /* ABI revision level. */
} xnfeatinfo_t;

#define __xn_feat_smp         0x80000000
#define __xn_feat_nosmp       0x40000000
#define __xn_feat_fastsynch   0x20000000
#define __xn_feat_nofastsynch 0x10000000

#ifdef CONFIG_SMP
#define __xn_feat_smp_mask __xn_feat_smp
#else
#define __xn_feat_smp_mask __xn_feat_nosmp
#endif

#ifdef CONFIG_XENO_FASTSYNCH
#define __xn_feat_fastsynch_mask __xn_feat_fastsynch
#else
#define __xn_feat_fastsynch_mask __xn_feat_nofastsynch
#endif

/* List of generic features kernel or userland may support */
#define __xn_feat_generic_mask \
	(__xn_feat_smp_mask | __xn_feat_fastsynch_mask)

/* List of features both sides have to agree on:
   If userland supports it, the kernel has to provide it, too. */
#define __xn_feat_generic_man_mask \
	(__xn_feat_fastsynch | __xn_feat_nofastsynch | __xn_feat_nosmp)

static inline const char *get_generic_feature_label (unsigned feature)
{
	switch (feature) {
	case __xn_feat_smp:
		return "smp";
	case __xn_feat_nosmp:
		return "nosmp";
	case __xn_feat_fastsynch:
		return "fastsynch";
	case __xn_feat_nofastsynch:
		return "nofastsynch";
	default:
		return 0;
	}
}

#endif /* !_XENO_ASM_GENERIC_FEATURES_H */
