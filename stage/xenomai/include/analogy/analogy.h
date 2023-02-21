/**
 * @file
 * Analogy for Linux, library facilities
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

#ifndef __ANALOGY_ANALOGY__
#define __ANALOGY_ANALOGY__

#include <unistd.h>

#include <analogy/types.h>
#include <analogy/descriptor.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef DOXYGEN_CPP

/* --- Level 0 API (not supposed to be used) --- */

int a4l_sys_open(const char *fname);

int a4l_sys_close(int fd);

int a4l_sys_read(int fd, void *buf, size_t nbyte);

int a4l_sys_write(int fd, void *buf, size_t nbyte);

int a4l_sys_attach(int fd, a4l_lnkdesc_t *arg);

int a4l_sys_detach(int fd);

int a4l_sys_bufcfg(int fd, unsigned int idx_subd, unsigned long size);

int a4l_sys_desc(int fd, a4l_desc_t *dsc, int pass);

int a4l_sys_devinfo(int fd, a4l_dvinfo_t *info);

int a4l_sys_subdinfo(int fd, a4l_sbinfo_t *info);

int a4l_sys_nbchaninfo(int fd, unsigned int idx_subd, unsigned int *nb);

int a4l_sys_chaninfo(int fd, 
		     unsigned int idx_subd, a4l_chinfo_t *info);

int a4l_sys_nbrnginfo(int fd,
		      unsigned int idx_subd,
		      unsigned int idx_chan, unsigned int *nb);

int a4l_sys_rnginfo(int fd,
		    unsigned int idx_subd,
		    unsigned int idx_chan, a4l_rnginfo_t *info);

/* --- Level 1 API (supposed to be used) --- */

int a4l_get_desc(int fd, a4l_desc_t *dsc, int pass);

int a4l_open(a4l_desc_t *dsc, const char *fname);

int a4l_close(a4l_desc_t *dsc);

int a4l_fill_desc(a4l_desc_t *dsc);

int a4l_get_subdinfo(a4l_desc_t *dsc,
		     unsigned int subd, a4l_sbinfo_t **info);

int a4l_get_chinfo(a4l_desc_t *dsc,
		   unsigned int subd,
		   unsigned int chan, a4l_chinfo_t **info);

#define a4l_get_chan_max(x) (1ULL << (x)->nb_bits)

#define a4l_is_chan_global(x) ((x)->chan_flags & A4L_CHAN_GLOBAL)

int a4l_get_rnginfo(a4l_desc_t *dsc,
		    unsigned int subd,
		    unsigned int chan,
		    unsigned int rng, a4l_rnginfo_t **info);

#define a4l_is_rng_global(x) ((x)->flags & A4L_RNG_GLOBAL)

int a4l_snd_command(a4l_desc_t *dsc, a4l_cmd_t *cmd);
    
int a4l_snd_cancel(a4l_desc_t *dsc, unsigned int idx_subd);

int a4l_set_bufsize(a4l_desc_t *dsc,
		    unsigned int idx_subd, unsigned long size);

int a4l_get_bufsize(a4l_desc_t *dsc,
		    unsigned int idx_subd, unsigned long *size);

int a4l_set_wakesize(a4l_desc_t *dsc, unsigned long size);

int a4l_get_wakesize(a4l_desc_t *dsc, unsigned long *size);

int a4l_mark_bufrw(a4l_desc_t *dsc,
		   unsigned int idx_subd,
		   unsigned long cur, unsigned long *newp);

int a4l_poll(a4l_desc_t *dsc,
	     unsigned int idx_subd, unsigned long ms_timeout);
    
int a4l_mmap(a4l_desc_t *dsc,
	     unsigned int idx_subd, unsigned long size, void **ptr);

int a4l_async_read(a4l_desc_t *dsc,
		   void *buf, size_t nbyte, unsigned long ms_timeout);

int a4l_async_write(a4l_desc_t *dsc,
		    void *buf, size_t nbyte, unsigned long ms_timeout);

int a4l_snd_insnlist(a4l_desc_t *dsc, a4l_insnlst_t *arg);

int a4l_snd_insn(a4l_desc_t *dsc, a4l_insn_t *arg);

/* --- Level 2 API (supposed to be used) --- */

int a4l_sync_write(a4l_desc_t *dsc,
		   unsigned int idx_subd,
		   unsigned int chan_desc,
		   unsigned int delay, void *buf, size_t nbyte);
	
int a4l_sync_read(a4l_desc_t *dsc,
		  unsigned int idx_subd,
		  unsigned int chan_desc,
		  unsigned int delay, void *buf, size_t nbyte);

int a4l_config_subd(a4l_desc_t *dsc,
		    unsigned int idx_subd, unsigned int type, ...);

int a4l_sync_dio(a4l_desc_t *dsc,
		 unsigned int idx_subd, void *mask, void *buf);

int a4l_sizeof_chan(a4l_chinfo_t *chan);

int a4l_sizeof_subd(a4l_sbinfo_t *subd);

int a4l_find_range(a4l_desc_t *dsc,
		   unsigned int idx_subd,
		   unsigned int idx_chan,
		   unsigned long unit,
		   double min, double max, a4l_rnginfo_t **rng);

int a4l_rawtoul(a4l_chinfo_t *chan, unsigned long *dst, void *src, int cnt);

int a4l_rawtof(a4l_chinfo_t *chan,
	       a4l_rnginfo_t *rng, float *dst, void *src, int cnt);

int a4l_rawtod(a4l_chinfo_t *chan,
	       a4l_rnginfo_t *rng, double *dst, void *src, int cnt);

int a4l_ultoraw(a4l_chinfo_t *chan, void *dst, unsigned long *src, int cnt);

int a4l_ftoraw(a4l_chinfo_t *chan,
	       a4l_rnginfo_t *rng, void *dst, float *src, int cnt);

int a4l_dtoraw(a4l_chinfo_t *chan,
	       a4l_rnginfo_t *rng, void *dst, double *src, int cnt);

#endif /* !DOXYGEN_CPP */

#ifdef __cplusplus
}
#endif
#endif /* __ANALOGY_ANALOGY__ */
