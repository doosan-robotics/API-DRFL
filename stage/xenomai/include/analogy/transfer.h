/**
 * @file
 * Analogy for Linux, transfer related features
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

#ifndef __ANALOGY_TRANSFER__
#define __ANALOGY_TRANSFER__

#ifndef DOXYGEN_CPP

#include <analogy/context.h>
#include <analogy/subdevice.h>
#include <analogy/buffer.h>

/* Status flags / bits */
#define A4L_TSF_CLEAN 0

/* Fields init values */
#define A4L_IRQ_UNUSED (unsigned int)((unsigned short)(~0))
#define A4L_IDX_UNUSED (unsigned int)(~0)

/* TODO: IRQ handling must leave transfer for os_facilities */

/* IRQ types */
#define A4L_IRQ_SHARED RTDM_IRQTYPE_SHARED
#define A4L_IRQ_EDGE RTDM_IRQTYPE_EDGE
#define A4L_IRQ_DISABLED 0

/* Poll timeout values */
#define A4L_INFINITE 0
#define A4L_NONBLOCK (-1)

#ifdef __KERNEL__

struct a4l_device;
/* Analogy transfer descriptor */
struct a4l_transfer {

	/* Subdevices desc */
	unsigned int nb_subd;
	a4l_subd_t **subds;

	/* Buffer stuff: the default size */
	unsigned int default_bufsize;

	/* IRQ in use */
	/* TODO: irq_desc should vanish */
	a4l_irq_desc_t irq_desc;
};
typedef struct a4l_transfer a4l_trf_t;

/* --- Proc function --- */

int a4l_rdproc_transfer(struct seq_file *p, void *data);

/* --- Upper layer functions --- */

void a4l_presetup_transfer(a4l_cxt_t * cxt);
int a4l_setup_transfer(a4l_cxt_t * cxt);
int a4l_precleanup_transfer(a4l_cxt_t * cxt);
int a4l_cleanup_transfer(a4l_cxt_t * cxt);
int a4l_reserve_transfer(a4l_cxt_t * cxt, int idx_subd);
int a4l_init_transfer(a4l_cxt_t * cxt, a4l_cmd_t * cmd);
int a4l_cancel_transfer(a4l_cxt_t * cxt, int idx_subd);
int a4l_cancel_transfers(a4l_cxt_t * cxt);

ssize_t a4l_put(a4l_cxt_t * cxt, void *buf, size_t nbytes);
ssize_t a4l_get(a4l_cxt_t * cxt, void *buf, size_t nbytes);

int a4l_request_irq(struct a4l_device *dev,
		    unsigned int irq,
		    a4l_irq_hdlr_t handler,
		    unsigned long flags, void *cookie);
int a4l_free_irq(struct a4l_device *dev, unsigned int irq);
unsigned int a4l_get_irq(struct a4l_device *dev);

int a4l_ioctl_cancel(a4l_cxt_t * cxt, void *arg);

#endif /* __KERNEL__ */

#endif /* !DOXYGEN_CPP */

#endif /* __ANALOGY_TRANSFER__ */
