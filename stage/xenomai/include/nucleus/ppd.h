/**
 * @file
 * @note Copyright &copy; 2006 Gilles Chanteperdrix <gch@xenomai.org>
 * Per-process data.
 *
 * Xenomai is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, Inc., 675 Mass Ave, Cambridge MA 02139,
 * USA; either version 2 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
#ifndef _XENO_NUCLEUS_PPD_H
#define _XENO_NUCLEUS_PPD_H

#include <nucleus/queue.h>
#include <nucleus/shadow.h>

struct mm_struct;

typedef struct xnshadow_ppd_key {
    unsigned long muxid;
    struct mm_struct *mm;
} xnshadow_ppd_key_t;

typedef struct xnshadow_ppd_t {
    xnshadow_ppd_key_t key;
    xnholder_t link;
#define link2ppd(ln)	container_of(ln, xnshadow_ppd_t, link)
} xnshadow_ppd_t;

#define xnshadow_ppd_muxid(ppd) ((ppd)->key.muxid)

#define xnshadow_ppd_mm(ppd)    ((ppd)->key.mm)

/* Call with nklock locked irqs off. */
xnshadow_ppd_t *xnshadow_ppd_get(unsigned muxid);

#endif /* _XENO_NUCLEUS_PPD_H */
