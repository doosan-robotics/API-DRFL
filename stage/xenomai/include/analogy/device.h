/**
 * @file
 * Analogy for Linux, device related features
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

#ifndef __ANALOGY_DEVICE__
#define __ANALOGY_DEVICE__

#ifndef DOXYGEN_CPP

#include <analogy/context.h>
#include <analogy/driver.h>
#include <analogy/transfer.h>

#ifdef __KERNEL__

#define A4L_NB_DEVICES 10

#define A4L_DEV_ATTACHED_NR 0

struct a4l_device {

	/* Spinlock for global device use */
	a4l_lock_t lock;

	/* Device specific flags */
	unsigned long flags;

	/* Driver assigned to this device thanks to attaching
	   procedure */
	a4l_drv_t *driver;

	/* Hidden description stuff */
	struct list_head subdvsq;

	/* Context-dependent stuff */
	a4l_trf_t transfer;

	/* Private data useful for drivers functioning */
	void *priv;
};
typedef struct a4l_device a4l_dev_t;

#endif /* __KERNEL__ */

/* DEVCFG ioctl argument structure */
struct a4l_link_desc {
	unsigned char bname_size;
	char *bname;
	unsigned int opts_size;
	void *opts;
};
typedef struct a4l_link_desc a4l_lnkdesc_t;

/* DEVINFO ioctl argument structure */
struct a4l_dev_info {
	char board_name[A4L_NAMELEN];
	int nb_subd;
	int idx_read_subd;
	int idx_write_subd;
};
typedef struct a4l_dev_info a4l_dvinfo_t;

#ifdef __KERNEL__

/* --- Devices tab related functions --- */
void a4l_init_devs(void);
int a4l_check_cleanup_devs(void);
int a4l_rdproc_devs(struct seq_file *p, void *data);

/* --- Context related function / macro --- */
void a4l_set_dev(a4l_cxt_t *cxt);
#define a4l_get_dev(x) ((x)->dev)

/* --- Upper layer functions --- */
int a4l_ioctl_devcfg(a4l_cxt_t * cxt, void *arg);
int a4l_ioctl_devinfo(a4l_cxt_t * cxt, void *arg);

#endif /* __KERNEL__ */

#endif /* !DOXYGEN_CPP */

#endif /* __ANALOGY_DEVICE__ */
