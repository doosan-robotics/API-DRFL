/**
 * @file
 * Analogy for Linux, instruction related features
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

#ifndef __ANALOGY_INSTRUCTION__
#define __ANALOGY_INSTRUCTION__

#include <analogy/types.h>
#include <analogy/context.h>

#define A4L_INSN_MASK_READ 0x8000000
#define A4L_INSN_MASK_WRITE 0x4000000
#define A4L_INSN_MASK_SPECIAL 0x2000000

/*!
 * @addtogroup sync1_lib
 * @{
 */

/*!
 * @anchor ANALOGY_INSN_xxx @name Instruction type
 * @brief Flags to define the type of instruction
 * @{
 */

/**
 * Read instruction
 */
#define A4L_INSN_READ (0 | A4L_INSN_MASK_READ)
/**
 * Write instruction
 */
#define A4L_INSN_WRITE (1 | A4L_INSN_MASK_WRITE)
/**
 * "Bits" instruction
 */
#define A4L_INSN_BITS (2 | A4L_INSN_MASK_READ | \
		       A4L_INSN_MASK_WRITE)
/**
 * Configuration instruction
 */
#define A4L_INSN_CONFIG (3 | A4L_INSN_MASK_READ | \
			 A4L_INSN_MASK_WRITE)
/**
 * Get time instruction
 */
#define A4L_INSN_GTOD (4 | A4L_INSN_MASK_READ | \
		       A4L_INSN_MASK_SPECIAL)
/**
 * Wait instruction
 */
#define A4L_INSN_WAIT (5 | A4L_INSN_MASK_WRITE | \
		       A4L_INSN_MASK_SPECIAL)
/**
 * Trigger instruction (to start asynchronous acquisition)
 */
#define A4L_INSN_INTTRIG (6 | A4L_INSN_MASK_WRITE | \
			  A4L_INSN_MASK_SPECIAL)

	  /*! @} ANALOGY_INSN_xxx */

/**
 * Maximal wait duration
 */
#define A4L_INSN_WAIT_MAX 100000

/*!
 * @anchor INSN_CONFIG_xxx @name Configuration instruction type
 * @brief Values to define the type of configuration instruction
 * @{
 */

#define A4L_INSN_CONFIG_DIO_INPUT		0
#define A4L_INSN_CONFIG_DIO_OUTPUT		1
#define A4L_INSN_CONFIG_DIO_OPENDRAIN		2
#define A4L_INSN_CONFIG_ANALOG_TRIG		16
#define A4L_INSN_CONFIG_ALT_SOURCE		20
#define A4L_INSN_CONFIG_DIGITAL_TRIG		21
#define A4L_INSN_CONFIG_BLOCK_SIZE		22
#define A4L_INSN_CONFIG_TIMER_1			23
#define A4L_INSN_CONFIG_FILTER			24
#define A4L_INSN_CONFIG_CHANGE_NOTIFY		25
#define A4L_INSN_CONFIG_SERIAL_CLOCK		26
#define A4L_INSN_CONFIG_BIDIRECTIONAL_DATA	27
#define A4L_INSN_CONFIG_DIO_QUERY		28
#define A4L_INSN_CONFIG_PWM_OUTPUT		29
#define A4L_INSN_CONFIG_GET_PWM_OUTPUT		30
#define A4L_INSN_CONFIG_ARM			31
#define A4L_INSN_CONFIG_DISARM			32
#define A4L_INSN_CONFIG_GET_COUNTER_STATUS	33
#define A4L_INSN_CONFIG_RESET			34
#define A4L_INSN_CONFIG_GPCT_SINGLE_PULSE_GENERATOR	1001	/* Use CTR as single pulsegenerator */
#define A4L_INSN_CONFIG_GPCT_PULSE_TRAIN_GENERATOR	1002	/* Use CTR as pulsetraingenerator */
#define A4L_INSN_CONFIG_GPCT_QUADRATURE_ENCODER	1003	/* Use the counter as encoder */
#define A4L_INSN_CONFIG_SET_GATE_SRC		2001	/* Set gate source */
#define A4L_INSN_CONFIG_GET_GATE_SRC		2002	/* Get gate source */
#define A4L_INSN_CONFIG_SET_CLOCK_SRC		2003	/* Set master clock source */
#define A4L_INSN_CONFIG_GET_CLOCK_SRC		2004	/* Get master clock source */
#define A4L_INSN_CONFIG_SET_OTHER_SRC		2005	/* Set other source */
#define A4L_INSN_CONFIG_SET_COUNTER_MODE	4097
#define A4L_INSN_CONFIG_SET_ROUTING		4099
#define A4L_INSN_CONFIG_GET_ROUTING		4109

	  /*! @} INSN_CONFIG_xxx */

/*!
 * @anchor ANALOGY_COUNTER_xxx @name Counter status bits
 * @brief Status bits for INSN_CONFIG_GET_COUNTER_STATUS
 * @{
 */

#define A4L_COUNTER_ARMED		0x1
#define A4L_COUNTER_COUNTING		0x2
#define A4L_COUNTER_TERMINAL_COUNT	0x4

	  /*! @} ANALOGY_COUNTER_xxx */

/*!
 * @anchor ANALOGY_IO_DIRECTION @name IO direction
 * @brief Values to define the IO polarity
 * @{
 */

#define A4L_INPUT	0
#define A4L_OUTPUT	1
#define A4L_OPENDRAIN	2

	  /*! @} ANALOGY_IO_DIRECTION */


/*!
 * @anchor ANALOGY_EV_xxx @name Events types
 * @brief Values to define the Analogy events. They might used to send
 * some specific events through the instruction interface.
 * @{
 */

#define A4L_EV_START		0x00040000
#define A4L_EV_SCAN_BEGIN	0x00080000
#define A4L_EV_CONVERT		0x00100000
#define A4L_EV_SCAN_END		0x00200000
#define A4L_EV_STOP		0x00400000

	  /*! @} ANALOGY_EV_xxx */

/*!
 * @brief Structure describing the synchronous instruction
 * @see a4l_snd_insn()
 */

struct a4l_instruction {
	unsigned int type;
		       /**< Instruction type */
	unsigned int idx_subd;
			   /**< Subdevice to which the instruction will be applied. */
	unsigned int chan_desc;
			    /**< Channel descriptor */
	unsigned int data_size;
			    /**< Size of the intruction data */
	void *data;
		    /**< Instruction data */
};
typedef struct a4l_instruction a4l_insn_t;

/*!
 * @brief Structure describing the list of synchronous instructions
 * @see a4l_snd_insnlist()
 */

struct a4l_instruction_list {
	unsigned int count;
			/**< Instructions count */
	a4l_insn_t *insns;
			  /**< Tab containing the instructions pointers */
};
typedef struct a4l_instruction_list a4l_insnlst_t;

	  /*! @} sync1_lib */

#if defined(__KERNEL__) && !defined(DOXYGEN_CPP)

struct a4l_kernel_instruction {
	unsigned int type;
	unsigned int idx_subd;
	unsigned int chan_desc;
	unsigned int data_size;
	void *data;
	void *__udata;
};
typedef struct a4l_kernel_instruction a4l_kinsn_t;

struct a4l_kernel_instruction_list {
	unsigned int count;
	a4l_kinsn_t *insns;
	a4l_insn_t *__uinsns;
};
typedef struct a4l_kernel_instruction_list a4l_kilst_t;

/* Instruction related functions */

/* Upper layer functions */
int a4l_ioctl_insnlist(a4l_cxt_t * cxt, void *arg);
int a4l_ioctl_insn(a4l_cxt_t * cxt, void *arg);

#endif /* __KERNEL__ && !DOXYGEN_CPP */

#endif /* __ANALOGY_INSTRUCTION__ */
