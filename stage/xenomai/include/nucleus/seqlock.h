#ifndef __SEQLOCK_H
#define __SEQLOCK_H

/* Originally from the linux kernel, adapted for userland and Xenomai */

#include <asm/xenomai/atomic.h>

typedef struct xnseqcount {
	unsigned sequence;
} xnseqcount_t;

#define XNSEQCNT_ZERO { 0 }
#define xnseqcount_init(x) do { *(x) = (xnseqcount_t) XNSEQCNT_ZERO; } while (0)

/* Start of read using pointer to a sequence counter only.  */
static inline unsigned xnread_seqcount_begin(const xnseqcount_t *s)
{
	unsigned ret;

repeat:
	ret = s->sequence;
	xnarch_read_memory_barrier();
	if (unlikely(ret & 1)) {
		cpu_relax();
		goto repeat;
	}
	return ret;
}

/*
 * Test if reader processed invalid data because sequence number has changed.
 */
static inline int xnread_seqcount_retry(const xnseqcount_t *s, unsigned start)
{
	xnarch_read_memory_barrier();

	return s->sequence != start;
}


/*
 * The sequence counter only protects readers from concurrent writers.
 * Writers must use their own locking.
 */
static inline void xnwrite_seqcount_begin(xnseqcount_t *s)
{
	s->sequence++;
	xnarch_write_memory_barrier();
}

static inline void xnwrite_seqcount_end(xnseqcount_t *s)
{
	xnarch_write_memory_barrier();
	s->sequence++;
}

#endif /* __SEQLOCK_H */
