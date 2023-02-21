/**
 * @file
 * Analogy for Linux, driver related features
 *
 * @note Copyright (C) 1997-2000 David A. Schleef <ds@schleef.org>
 * @note Copyright (C) 2008 Alexis Berlemont <alexis.berlemont@free.fr>
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

#ifndef __ANALOGY_DRIVER__
#define __ANALOGY_DRIVER__

#ifdef __KERNEL__

#include <linux/list.h>

struct a4l_link_desc;
struct a4l_device;

/** Structure containing driver declaration data.
 *
 *  @see rt_task_inquire()
 */
/* Analogy driver descriptor */
struct a4l_driver {

	/* List stuff */
	struct list_head list;
			   /**< List stuff */

	/* Visible description stuff */
	struct module *owner;
			  /**< Pointer to module containing the code */
	unsigned int flags;
			/**< Type / status driver's flags */
	char *board_name;
		       /**< Board name */
	int privdata_size;
		       /**< Size of the driver's private data */

	/* Init/destroy procedures */
	int (*attach) (struct a4l_device *, struct a4l_link_desc *);
								      /**< Attach procedure */
	int (*detach) (struct a4l_device *);
				   /**< Detach procedure */

};
typedef struct a4l_driver a4l_drv_t;

#ifndef DOXYGEN_CPP

/* Driver list related functions */

int a4l_register_drv(a4l_drv_t * drv);
int a4l_unregister_drv(a4l_drv_t * drv);
int a4l_lct_drv(char *pin, a4l_drv_t ** pio);
#ifdef CONFIG_PROC_FS
int a4l_rdproc_drvs(struct seq_file *p, void *data);
#endif /* CONFIG_PROC_FS */

#endif /* !DOXYGEN_CPP */

#endif /* __KERNEL__ */

#endif /* __ANALOGY_DRIVER__ */
