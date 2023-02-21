#ifndef _XENO_NUCLEUS_VDSO_H
#define _XENO_NUCLEUS_VDSO_H

/*!\file vdso.h
 * \brief Definitions for global semaphore heap shared objects
 * \author Wolfgang Mauerer
 *
 * Copyright (C) 2009 Wolfgang Mauerer <wolfgang.mauerer@siemens.com>.
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

#include <nucleus/types.h>
#include <nucleus/hostrt.h>

/*
 * Data shared between Xenomai kernel/userland and the Linux kernel/userland
 * on the global semaphore heap. The features element indicates which data are
 * shared. Notice that struct xnvdso may only grow, but never shrink.
 */
struct xnvdso {
	unsigned long long features;

	struct xnvdso_hostrt_data hostrt_data;
	/*
	 * Embed further domain specific structures that
	 * describe the shared data here
	 */
};

/*
 * For each shared feature, add a flag below. For now, the set is still
 * empty.
 */
/*
#define XNVDSO_FEAT_A	0x0000000000000001ULL
#define XNVDSO_FEAT_B	0x0000000000000002ULL
#define XNVDSO_FEAT_C	0x0000000000000004ULL
#define XNVDSO_FEATURES	(XNVDSO_FEAT_A | XNVDSO_FEAT_B | XVDSO_FEAT_C)
*/
#define XNVDSO_FEAT_HOST_REALTIME	0x0000000000000001ULL
#ifdef CONFIG_XENO_OPT_HOSTRT
#define XNVDSO_FEATURES XNVDSO_FEAT_HOST_REALTIME
#else
#define XNVDSO_FEATURES 0
#endif /* CONFIG_XENO_OPT_HOSTRT */

extern struct xnvdso *nkvdso;

static inline struct xnvdso_hostrt_data *get_hostrt_data(void)
{
	return &nkvdso->hostrt_data;
}

static inline int xnvdso_test_feature(unsigned long long feature)
{
	return testbits(nkvdso->features, feature);
}

extern void xnheap_init_vdso(void);
#endif /* _XENO_NUCLEUS_VDSO_H */
