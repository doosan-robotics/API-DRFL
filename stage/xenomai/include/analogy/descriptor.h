/**
 * @file
 * Analogy for Linux, descriptor related features
 * @note Copyright (C) 1997-2000 David A. Schleef <ds@schleef.org>
 * @note Copyright (C) 2008 Alexis Berlemont <alexis.berlemont@free.fr>
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

#ifndef __ANALOGY_LIB_CORE__
#define __ANALOGY_LIB_CORE__

#include <analogy/subdevice.h>
#include <analogy/device.h>

/* --- Descriptor precompilation constants --- */

/* Constant used internally */
#define MAGIC_BSC_DESC 0x1234abcd
#define MAGIC_CPLX_DESC 0xabcd1234

/*!
  @addtogroup descriptor_sys
  @{
 */

/*!
 * @anchor ANALOGY_xxx_DESC   @name ANALOGY_xxx_DESC
 * @brief Constants used as argument so as to define the description
 * depth to recover
 * @{
 */

/**
 * BSC stands for basic descriptor (device data)
 */
#define A4L_BSC_DESC 0x0

/**
 * CPLX stands for complex descriptor (subdevice + channel + range
 * data)
 */
#define A4L_CPLX_DESC 0x1

	  /*! @} ANALOGY_xxx_DESC */

/* --- Descriptor structure --- */

/*!
 * @brief Structure containing device-information useful to users
 * @see a4l_get_desc()
 */

struct a4l_descriptor {
	char board_name[A4L_NAMELEN];
				     /**< Board name. */
	int nb_subd;
		 /**< Subdevices count. */
	int idx_read_subd;
		       /**< Input subdevice index. */
	int idx_write_subd;
			/**< Output subdevice index. */
	int fd;
	    /**< File descriptor. */
	unsigned int magic;
			/**< Opaque field. */
	int sbsize;
		/**< Data buffer size. */
	void *sbdata;
		 /**< Data buffer pointer. */
};
typedef struct a4l_descriptor a4l_desc_t;

	  /*! @} descriptor_sys */

#endif /* __ANALOGY_LIB_CORE__ */
