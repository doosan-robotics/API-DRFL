/**
 * @file
 * Analogy for Linux, subdevice related features
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

#ifndef __ANALOGY_SUBDEVICE__
#define __ANALOGY_SUBDEVICE__

#ifdef __KERNEL__
#include <linux/list.h>
#endif /* __KERNEL__ */

#include <analogy/types.h>
#include <analogy/context.h>
#include <analogy/instruction.h>
#include <analogy/command.h>
#include <analogy/channel_range.h>

/* --- Subdevice flags desc stuff --- */

/* TODO: replace ANALOGY_SUBD_AI with ANALOGY_SUBD_ANALOG
   and ANALOGY_SUBD_INPUT */

/* Subdevice types masks */
#define A4L_SUBD_MASK_READ 0x80000000
#define A4L_SUBD_MASK_WRITE 0x40000000
#define A4L_SUBD_MASK_SPECIAL 0x20000000

/*!
 * @addtogroup subdevice
 * @{
 */

/*!
 * @anchor ANALOGY_SUBD_xxx @name Subdevices types
 * @brief Flags to define the subdevice type
 * @{
 */

/**
 * Unused subdevice
 */
#define A4L_SUBD_UNUSED (A4L_SUBD_MASK_SPECIAL|0x1)
/**
 * Analog input subdevice
 */
#define A4L_SUBD_AI (A4L_SUBD_MASK_READ|0x2)
/**
 * Analog output subdevice
 */
#define A4L_SUBD_AO (A4L_SUBD_MASK_WRITE|0x4)
/**
 * Digital input subdevice
 */
#define A4L_SUBD_DI (A4L_SUBD_MASK_READ|0x8)
/**
 * Digital output subdevice
 */
#define A4L_SUBD_DO (A4L_SUBD_MASK_WRITE|0x10)
/**
 * Digital input/output subdevice
 */
#define A4L_SUBD_DIO (A4L_SUBD_MASK_SPECIAL|0x20)
/**
 * Counter subdevice
 */
#define A4L_SUBD_COUNTER (A4L_SUBD_MASK_SPECIAL|0x40)
/**
 * Timer subdevice
 */
#define A4L_SUBD_TIMER (A4L_SUBD_MASK_SPECIAL|0x80)
/**
 * Memory, EEPROM, DPRAM
 */
#define A4L_SUBD_MEMORY (A4L_SUBD_MASK_SPECIAL|0x100)
/**
 * Calibration subdevice  DACs
 */
#define A4L_SUBD_CALIB (A4L_SUBD_MASK_SPECIAL|0x200)
/**
 * Processor, DSP
 */
#define A4L_SUBD_PROC (A4L_SUBD_MASK_SPECIAL|0x400)
/**
 * Serial IO subdevice
 */
#define A4L_SUBD_SERIAL (A4L_SUBD_MASK_SPECIAL|0x800)
/**
 * Mask which gathers all the types
 */
#define A4L_SUBD_TYPES (A4L_SUBD_UNUSED |	 \
			   A4L_SUBD_AI |	 \
			   A4L_SUBD_AO |	 \
			   A4L_SUBD_DI |	 \
			   A4L_SUBD_DO |	 \
			   A4L_SUBD_DIO |	 \
			   A4L_SUBD_COUNTER | \
			   A4L_SUBD_TIMER |	 \
			   A4L_SUBD_MEMORY |	 \
			   A4L_SUBD_CALIB |	 \
			   A4L_SUBD_PROC |	 \
			   A4L_SUBD_SERIAL)

	  /*! @} ANALOGY_SUBD_xxx */

/*!
 * @anchor ANALOGY_SUBD_FT_xxx @name Subdevice features
 * @brief Flags to define the subdevice's capabilities
 * @{
 */

/* Subdevice capabilities */
/**
 * The subdevice can handle command (i.e it can perform asynchronous
 * acquisition)
 */
#define A4L_SUBD_CMD 0x1000
/**
 * The subdevice support mmap operations (technically, any driver can
 * do it; however, the developer might want that his driver must be
 * accessed through read / write
 */
#define A4L_SUBD_MMAP 0x8000

	  /*! @} ANALOGY_SUBD_FT_xxx */

/*!
 * @anchor ANALOGY_SUBD_ST_xxx @name Subdevice status
 * @brief Flags to define the subdevice's status
 * @{
 */

/* Subdevice status flag(s) */
/**
 * The subdevice is busy, a synchronous or an asynchronous acquisition
 * is occuring
 */
#define A4L_SUBD_BUSY_NR 0
#define A4L_SUBD_BUSY (1 << A4L_SUBD_BUSY_NR)

/**
 * The subdevice is about to be cleaned in the middle of the detach
 * procedure
 */
#define A4L_SUBD_CLEAN_NR 1
#define A4L_SUBD_CLEAN (1 << A4L_SUBD_CLEAN_NR)


	  /*! @} ANALOGY_SUBD_ST_xxx */

#ifdef __KERNEL__

/* --- Subdevice descriptor structure --- */

struct a4l_device;
struct a4l_buffer;

/*!
 * @brief Structure describing the subdevice
 * @see a4l_add_subd()
 */

struct a4l_subdevice {

	struct list_head list;
			   /**< List stuff */

	struct a4l_device *dev;
			       /**< Containing device */

	unsigned int idx;
		      /**< Subdevice index */

	struct a4l_buffer *buf;
			       /**< Linked buffer */

	/* Subdevice's status (busy, linked?) */
	unsigned long status;
			     /**< Subdevice's status */

	/* Descriptors stuff */
	unsigned long flags;
			 /**< Type flags */
	a4l_chdesc_t *chan_desc;
				/**< Tab of channels descriptors pointers */
	a4l_rngdesc_t *rng_desc;
				/**< Tab of ranges descriptors pointers */
	a4l_cmd_t *cmd_mask;
			    /**< Command capabilities mask */

	/* Functions stuff */
	int (*insn_read) (struct a4l_subdevice *, a4l_kinsn_t *);
							/**< Callback for the instruction "read" */
	int (*insn_write) (struct a4l_subdevice *, a4l_kinsn_t *);
							 /**< Callback for the instruction "write" */
	int (*insn_bits) (struct a4l_subdevice *, a4l_kinsn_t *);
							/**< Callback for the instruction "bits" */
	int (*insn_config) (struct a4l_subdevice *, a4l_kinsn_t *);
							  /**< Callback for the configuration instruction */
	int (*do_cmd) (struct a4l_subdevice *, a4l_cmd_t *);
					/**< Callback for command handling */
	int (*do_cmdtest) (struct a4l_subdevice *, a4l_cmd_t *);
						       /**< Callback for command checking */
	int (*cancel) (struct a4l_subdevice *);
					 /**< Callback for asynchronous transfer cancellation */
	void (*munge) (struct a4l_subdevice *, void *, unsigned long);
								/**< Callback for munge operation */
	int (*trigger) (struct a4l_subdevice *, lsampl_t);
					      /**< Callback for trigger operation */

	char priv[0];
		  /**< Private data */
};
typedef struct a4l_subdevice a4l_subd_t;

#endif /* __KERNEL__ */

	  /*! @} subdevice */

#ifndef DOXYGEN_CPP

/* --- Subdevice related IOCTL arguments structures --- */

/* SUDBINFO IOCTL argument */
struct a4l_subd_info {
	unsigned long flags;
	unsigned long status;
	unsigned char nb_chan;
};
typedef struct a4l_subd_info a4l_sbinfo_t;

/* CHANINFO / NBCHANINFO IOCTL arguments */
struct a4l_chan_info {
	unsigned long chan_flags;
	unsigned char nb_rng;
	unsigned char nb_bits;
};
typedef struct a4l_chan_info a4l_chinfo_t;

struct a4l_chinfo_arg {
	unsigned int idx_subd;
	void *info;
};
typedef struct a4l_chinfo_arg a4l_chinfo_arg_t;

/* RNGINFO / NBRNGINFO IOCTL arguments */
struct a4l_rng_info {
	long min;
	long max;
	unsigned long flags;
};
typedef struct a4l_rng_info a4l_rnginfo_t;

struct a4l_rng_info_arg {
	unsigned int idx_subd;
	unsigned int idx_chan;
	void *info;
};
typedef struct a4l_rng_info_arg a4l_rnginfo_arg_t;

#ifdef __KERNEL__

/* --- Subdevice related functions and macros --- */

a4l_chan_t *a4l_get_chfeat(a4l_subd_t * sb, int idx);
a4l_rng_t *a4l_get_rngfeat(a4l_subd_t * sb, int chidx, int rngidx);
int a4l_check_chanlist(a4l_subd_t * subd,
		       unsigned char nb_chan, unsigned int *chans);

#define a4l_subd_is_input(x) ((A4L_SUBD_MASK_READ & (x)->flags) != 0)
/* The following macro considers that a DIO subdevice is firstly an
   output subdevice */
#define a4l_subd_is_output(x) \
	((A4L_SUBD_MASK_WRITE & (x)->flags) != 0 || \
	 (A4L_SUBD_DIO & (x)->flags) != 0)

/* --- Upper layer functions --- */

a4l_subd_t * a4l_get_subd(struct a4l_device *dev, int idx);
a4l_subd_t * a4l_alloc_subd(int sizeof_priv,
			    void (*setup)(a4l_subd_t *));
int a4l_add_subd(struct a4l_device *dev, a4l_subd_t * subd);
int a4l_ioctl_subdinfo(a4l_cxt_t * cxt, void *arg);
int a4l_ioctl_chaninfo(a4l_cxt_t * cxt, void *arg);
int a4l_ioctl_rnginfo(a4l_cxt_t * cxt, void *arg);
int a4l_ioctl_nbchaninfo(a4l_cxt_t * cxt, void *arg);
int a4l_ioctl_nbrnginfo(a4l_cxt_t * cxt, void *arg);

#endif /* __KERNEL__ */

#endif /* !DOXYGEN_CPP */

#endif /* __ANALOGY_SUBDEVICE__ */
