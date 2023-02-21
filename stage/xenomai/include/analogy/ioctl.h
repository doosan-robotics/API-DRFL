/**
 * @file
 * Analogy for Linux, IOCTLs declarations
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

#ifndef __ANALOGY_IOCTL__
#define __ANALOGY_IOCTL__

#ifndef DOXYGEN_CPP

#ifdef __KERNEL__

#include <rtdm/rtdm_driver.h>

#define NB_IOCTL_FUNCTIONS 17

#endif /* __KERNEL__ */

#include <analogy/device.h>

#define CIO 'd'
#define A4L_DEVCFG _IOW(CIO,0,a4l_lnkdesc_t)
#define A4L_DEVINFO _IOR(CIO,1,a4l_dvinfo_t)
#define A4L_SUBDINFO _IOR(CIO,2,a4l_sbinfo_t)
#define A4L_CHANINFO _IOR(CIO,3,a4l_chinfo_arg_t)
#define A4L_RNGINFO _IOR(CIO,4,a4l_rnginfo_arg_t)
#define A4L_CMD _IOWR(CIO,5,a4l_cmd_t)
#define A4L_CANCEL _IOR(CIO,6,unsigned int)
#define A4L_INSNLIST _IOR(CIO,7,unsigned int)
#define A4L_INSN _IOR(CIO,8,unsigned int)
#define A4L_BUFCFG _IOR(CIO,9,a4l_bufcfg_t)
#define A4L_BUFINFO _IOWR(CIO,10,a4l_bufinfo_t)
#define A4L_POLL _IOR(CIO,11,unsigned int)
#define A4L_MMAP _IOWR(CIO,12,unsigned int)
#define A4L_NBCHANINFO _IOR(CIO,13,a4l_chinfo_arg_t)
#define A4L_NBRNGINFO _IOR(CIO,14,a4l_rnginfo_arg_t)

/* These IOCTLs are bound to be merged with A4L_BUFCFG and A4L_BUFINFO
   at the next major release */
#define A4L_BUFCFG2 _IOR(CIO,15,a4l_bufcfg_t)
#define A4L_BUFINFO2 _IOWR(CIO,16,a4l_bufcfg_t)

#endif /* !DOXYGEN_CPP */

#endif /* __ANALOGY_IOCTL__ */
