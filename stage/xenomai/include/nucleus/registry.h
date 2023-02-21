/**
 * @file
 * This file is part of the Xenomai project.
 *
 * @note Copyright (C) 2004 Philippe Gerum <rpm@xenomai.org>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 * \ingroup registry
 */

#ifndef _XENO_NUCLEUS_REGISTRY_H
#define _XENO_NUCLEUS_REGISTRY_H

#include <nucleus/types.h>

#define XNOBJECT_SELF  XN_NO_HANDLE

#if defined(__KERNEL__) || defined(__XENO_SIM__)

#include <nucleus/synch.h>
#include <nucleus/vfile.h>

struct xnpnode;

typedef struct xnobject {
	void *objaddr;
	const char *key;	  /* !< Hash key. */
	struct xnsynch safesynch; /* !< Safe synchronization object. */
	u_long safelock;	  /* !< Safe lock count. */
	u_long cstamp;		  /* !< Creation stamp. */
#ifdef CONFIG_XENO_OPT_VFILE
	struct xnpnode *pnode;	/* !< v-file information class. */
	union {
		struct {
			struct xnvfile_rev_tag tag;
			struct xnvfile_snapshot file;
		} vfsnap; /* !< virtual snapshot file. */
		struct xnvfile_regular vfreg; /* !< virtual regular file */
		struct xnvfile_link link;     /* !< virtual link. */
	} vfile_u;
	struct xnvfile *vfilp;
#endif /* CONFIG_XENO_OPT_VFILE */
	struct xnobject *hnext;	/* !< Next in h-table */
	struct xnholder link;
} xnobject_t;

#define link2xnobj(ln)		container_of(ln, struct xnobject, link)

#ifdef __cplusplus
extern "C" {
#endif

int xnregistry_init(void);

void xnregistry_cleanup(void);

#ifdef CONFIG_XENO_OPT_VFILE

#define XNOBJECT_PNODE_RESERVED1 ((struct xnvfile *)1)
#define XNOBJECT_PNODE_RESERVED2 ((struct xnvfile *)2)

struct xnptree {
	const char *dirname;
	/* hidden */
	int entries;
	struct xnvfile_directory vdir;
};

#define DEFINE_XNPTREE(__var, __name)		\
	struct xnptree __var = {		\
		.dirname = __name,		\
		.entries = 0,			\
		.vdir = xnvfile_nodir,		\
	}

struct xnpnode_ops {
	int (*export)(struct xnobject *object, struct xnpnode *pnode);
	void (*unexport)(struct xnobject *object, struct xnpnode *pnode);
	void (*touch)(struct xnobject *object);
};

struct xnpnode {
	const char *dirname;
	struct xnptree *root;
	struct xnpnode_ops *ops;
	/* hidden */
	int entries;
	struct xnvfile_directory vdir;
};

struct xnpnode_snapshot {
	struct xnpnode node;
	struct xnvfile_snapshot_template vfile;
};

struct xnpnode_regular {
	struct xnpnode node;
	struct xnvfile_regular_template vfile;
};

struct xnpnode_link {
	struct xnpnode node;
	char *(*target)(void *obj);
};

#else /* !CONFIG_XENO_OPT_VFILE */

#define DEFINE_XNPTREE(__var, __name);

/* Placeholders. */

struct xnpnode {
	const char *dirname;
};

struct xnpnode_snapshot {
	struct xnpnode node;
};

struct xnpnode_regular {
	struct xnpnode node;
};

struct xnpnode_link {
	struct xnpnode node;
};

#endif /* !CONFIG_XENO_OPT_VFILE */

extern struct xnobject *registry_obj_slots;

/* Public interface. */

extern struct xnobject *registry_obj_slots;

static inline struct xnobject *xnregistry_validate(xnhandle_t handle)
{
	struct xnobject *object;
	/*
	 * Careful: a removed object which is still in flight to be
	 * unexported carries a NULL objaddr, so we have to check this
	 * as well.
	 */
	if (likely(handle && handle < CONFIG_XENO_OPT_REGISTRY_NRSLOTS)) {
		object = &registry_obj_slots[handle];
		return object->objaddr ? object : NULL;
	}

	return NULL;
}

static inline void *xnregistry_lookup(xnhandle_t handle)
{
	struct xnobject *object = xnregistry_validate(handle);
	return object ? object->objaddr : NULL;
}

int xnregistry_enter(const char *key,
		     void *objaddr,
		     xnhandle_t *phandle,
		     struct xnpnode *pnode);

int xnregistry_bind(const char *key,
		    xnticks_t timeout,
		    int timeout_mode,
		    xnhandle_t *phandle);

int xnregistry_remove(xnhandle_t handle);

int xnregistry_remove_safe(xnhandle_t handle,
			   xnticks_t timeout);

void *xnregistry_get(xnhandle_t handle);

void *xnregistry_fetch(xnhandle_t handle);

u_long xnregistry_put(xnhandle_t handle);

#ifdef __cplusplus
}
#endif

extern struct xnpnode_ops xnregistry_vfsnap_ops;

extern struct xnpnode_ops xnregistry_vlink_ops;

#endif /* __KERNEL__ || __XENO_SIM__ */

#endif /* !_XENO_NUCLEUS_REGISTRY_H */
