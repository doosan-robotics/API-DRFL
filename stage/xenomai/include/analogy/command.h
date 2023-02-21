/**
 * @file
 * Analogy for Linux, driver facilities
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

#ifndef __ANALOGY_CMD__
#define __ANALOGY_CMD__

#include <analogy/context.h>

/*!
 * @addtogroup async1_lib
 * @{
 */

/*!
 * @anchor ANALOGY_CMD_xxx @name ANALOGY_CMD_xxx
 * @brief Common command flags definitions
 * @{
 */

/**
 * Do not execute the command, just check it
 */
#define A4L_CMD_SIMUL 0x1
/**
 * Perform data recovery / transmission in bulk mode
 */
#define A4L_CMD_BULK 0x2
/**
 * Perform a command which will write data to the device
 */
#define A4L_CMD_WRITE 0x4

	  /*! @} ANALOGY_CMD_xxx */

/*!
 * @anchor TRIG_xxx @name TRIG_xxx
 * @brief Command triggers flags definitions
 * @{
 */

/**
 * Never trigger
 */
#define TRIG_NONE	0x00000001
/**
 * Trigger now + N ns
 */
#define TRIG_NOW	0x00000002
/**
 * Trigger on next lower level trig
 */
#define TRIG_FOLLOW	0x00000004
/**
 * Trigger at time N ns
 */
#define TRIG_TIME	0x00000008
/**
 * Trigger at rate N ns
 */
#define TRIG_TIMER	0x00000010
/**
 * Trigger when count reaches N
 */
#define TRIG_COUNT	0x00000020
/**
 * Trigger on external signal N
 */
#define TRIG_EXT	0x00000040
/**
 * Trigger on analogy-internal signal N
 */
#define TRIG_INT	0x00000080
/**
 * Driver defined trigger
 */
#define TRIG_OTHER	0x00000100
/**
 * Wake up on end-of-scan
 */
#define TRIG_WAKE_EOS	0x0020
/**
 * Trigger not implemented yet
 */
#define TRIG_ROUND_MASK 0x00030000
/**
 * Trigger not implemented yet
 */
#define TRIG_ROUND_NEAREST 0x00000000
/**
 * Trigger not implemented yet
 */
#define TRIG_ROUND_DOWN 0x00010000
/**
 * Trigger not implemented yet
 */
#define TRIG_ROUND_UP 0x00020000
/**
 * Trigger not implemented yet
 */
#define TRIG_ROUND_UP_NEXT 0x00030000

	  /*! @} TRIG_xxx */

/*!
 * @anchor CHAN_RNG_AREF @name Channel macros
 * @brief Specific precompilation macros and constants useful for the
 * channels descriptors tab located in the command structure
 * @{
 */

/**
 * Channel indication macro
 */
#define CHAN(a) ((a) & 0xffff)
/**
 * Range definition macro
 */
#define RNG(a) (((a) & 0xff) << 16)
/**
 * Reference definition macro
 */
#define AREF(a) (((a) & 0xf) << 24)
/**
 * Flags definition macro
 */
#define FLAGS(a) ((a) & CR_FLAGS_MASK)
/**
 * Channel + range + reference definition macro
 */
#define PACK(a, b, c) (CHAN(a) | RNG(b) | AREF(c))
/**
 * Channel + range + reference + flags definition macro
 */
#define PACK_FLAGS(a, b, c, d) (CHAN(a) | RNG(b) | AREF(c) | FLAGS(d))

/**
 * Analog reference is analog ground
 */
#define AREF_GROUND 0x00
/**
 * Analog reference is analog common
 */
#define AREF_COMMON 0x01
/**
 * Analog reference is differential
 */
#define AREF_DIFF 0x02
/**
 * Analog reference is undefined
 */
#define AREF_OTHER 0x03

	  /*! @} CHAN_RNG_AREF */

#if !defined(DOXYGEN_CPP)

#define CR_FLAGS_MASK 0xfc000000
#define CR_ALT_FILTER (1<<26)
#define CR_DITHER CR_ALT_FILTER
#define CR_DEGLITCH CR_ALT_FILTER
#define CR_ALT_SOURCE (1<<27)
#define CR_EDGE	(1<<28)
#define CR_INVERT (1<<29)

#if defined(__KERNEL__)
/* Channels macros only useful for the kernel side */
#define CR_CHAN(a) CHAN(a)
#define CR_RNG(a) (((a)>>16)&0xff)
#define CR_AREF(a) (((a)>>24)&0xf)
#endif /* __KERNEL__ */

#endif /* !DOXYGEN_CPP */

/*!
 * @brief Structure describing the asynchronous instruction
 * @see a4l_snd_command()
 */

struct a4l_cmd_desc {
	unsigned char idx_subd;
			    /**< Subdevice to which the command will be applied. */

	unsigned long flags;
			 /**< Command flags */

	/* Command trigger characteristics */
	unsigned int start_src;
			    /**< Start trigger type */
	unsigned int start_arg;
			    /**< Start trigger argument */
	unsigned int scan_begin_src;
				 /**< Scan begin trigger type */
	unsigned int scan_begin_arg;
				 /**< Scan begin trigger argument */
	unsigned int convert_src;
			      /**< Convert trigger type */
	unsigned int convert_arg;
			      /**< Convert trigger argument */
	unsigned int scan_end_src;
			       /**< Scan end trigger type */
	unsigned int scan_end_arg;
			       /**< Scan end trigger argument */
	unsigned int stop_src;
			   /**< Stop trigger type */
	unsigned int stop_arg;
			   /**< Stop trigger argument */

	unsigned char nb_chan;
			   /**< Count of channels related with the command */
	unsigned int *chan_descs;
			      /**< Tab containing channels descriptors */

	/* Driver specific fields */
	unsigned int data_len;
			   /**< Driver specific buffer size */
	sampl_t *data;
		   /**< Driver specific buffer pointer */
};
typedef struct a4l_cmd_desc a4l_cmd_t;

	  /*! @} async1_lib */

#if defined(__KERNEL__) && !defined(DOXYGEN_CPP)

/* --- Command related function --- */
void a4l_free_cmddesc(a4l_cmd_t * desc);

/* --- Upper layer functions --- */
int a4l_check_cmddesc(a4l_cxt_t * cxt, a4l_cmd_t * desc);
int a4l_ioctl_cmd(a4l_cxt_t * cxt, void *arg);

#endif /* __KERNEL__ && !DOXYGEN_CPP */

#endif /* __ANALOGY_CMD__ */
