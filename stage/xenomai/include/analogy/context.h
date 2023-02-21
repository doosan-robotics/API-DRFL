/**
 * @file
 * Analogy for Linux, context structure / macros declarations
 *
 * Copyright (C) 1997-2000 David A. Schleef <ds@schleef.org>
 * Copyright (C) 2008 Alexis Berlemont <alexis.berlemont@free.fr>
 *
 * Xenomai is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Xenomai is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Xenomai; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __ANALOGY_CONTEXT__
#define __ANALOGY_CONTEXT__

#if defined(__KERNEL__) && !defined(DOXYGEN_CPP)

#include <rtdm/rtdm_driver.h>

struct a4l_device;
struct a4l_buffer;

struct a4l_device_context {

	/* Needed to call rtdm_*_copy_from/to_user functions */
	rtdm_user_info_t *user_info;

	/* The adequate device pointer
	   (retrieved thanks to minor at open time) */
	struct a4l_device *dev;

	/* The buffer structure contains everything to transfer data
	   from asynchronous acquisition operations on a specific
	   subdevice */
	struct a4l_buffer *buffer;
};
typedef struct a4l_device_context a4l_cxt_t;

static inline int a4l_get_minor(a4l_cxt_t *cxt)
{
	/* Get a pointer on the container structure */
	struct rtdm_dev_context * rtdm_cxt = rtdm_private_to_context(cxt);
	/* Get the minor index */
	return rtdm_cxt->device->device_id;
}

#endif /* __KERNEL__ && !DOXYGEN_CPP */

#endif /* __ANALOGY_CONTEXT__ */
