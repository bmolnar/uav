#ifndef _MEMBUF_H
#define _MEMBUF_H

#include <stdlib.h>
//#include <sys/uio.h>

#include "xstream_types.h"

struct membuf_iov {
    void * iov_base;
    size_t iov_len;
};

struct membuf {
    void * mb_buf;
    size_t mb_capacity;
    size_t mb_limit;
    size_t mb_position;
    size_t mb_mark;
};

void membuf_init(struct membuf *mbuf, void *buf, size_t capacity);
size_t membuf_capacity(struct membuf *mbuf);
size_t membuf_remaining(struct membuf *mbuf);
size_t membuf_position(struct membuf *mbuf);
int membuf_setposition(struct membuf *mbuf, size_t position);
size_t membuf_limit(struct membuf *mbuf);
int membuf_setlimit(struct membuf *mbuf, size_t limit);
void membuf_clear(struct membuf *mbuf);
void membuf_flip(struct membuf *mbuf);
void membuf_mark(struct membuf *mbuf);
void membuf_reset(struct membuf *mbuf);
void membuf_rewind(struct membuf *mbuf);
ssize_t membuf_memptrs(struct membuf *mbuf, struct membuf_iov *iov, size_t iovcnt);
ssize_t membuf_putmembuf(struct membuf *mbuf, struct membuf *src);
ssize_t membuf_putbytes(struct membuf *mbuf, const unsigned char *buf, size_t count);
ssize_t membuf_getbytes(struct membuf *mbuf, unsigned char *buf, size_t count);

#endif /* _MEMBUF_H */
