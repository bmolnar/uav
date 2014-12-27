#include <errno.h>
#include <string.h>
#include <sys/uio.h>

#include "membuf.h"

static size_t
memcpyv(const struct membuf_iov *dstiov, int dstiovlen, const struct membuf_iov *srciov, int srciovlen, size_t count)
{
    int dstidx = 0, srcidx = 0;
    size_t dstpos = 0, srcpos = 0;
    size_t total = 0;
    size_t left;

    while (total < count && dstidx < dstiovlen && srcidx < srciovlen) {
        left = count - total;
        if (left > (srciov[srcidx].iov_len - srcpos))
            left = (srciov[srcidx].iov_len - srcpos);
        if (left > (dstiov[dstidx].iov_len - dstpos))
            left = (dstiov[dstidx].iov_len - dstpos);
        memcpy(((unsigned char *) dstiov[dstidx].iov_base) + dstpos,
               ((unsigned char *) srciov[srcidx].iov_base) + srcpos, left);
        total += left;
        srcpos += left;
        dstpos += left;
        if (srcpos >= srciov[srcidx].iov_len) {
            srcidx++;
            srcpos = 0;
        }
        if (dstpos >= dstiov[dstidx].iov_len) {
            dstidx++;
            dstpos = 0;
        }
    }
    return total;
}

void
membuf_init(struct membuf *mbuf, void *buf, size_t capacity)
{
    mbuf->mb_buf = buf;
    mbuf->mb_capacity = capacity;
    mbuf->mb_limit = mbuf->mb_capacity;
    mbuf->mb_position = 0;
}

size_t
membuf_capacity(struct membuf *mbuf)
{
    return mbuf->mb_capacity;
}

size_t
membuf_remaining(struct membuf *mbuf)
{
    return (mbuf->mb_position < mbuf->mb_limit) ? (mbuf->mb_limit - mbuf->mb_position) : 0;
}

size_t
membuf_position(struct membuf *mbuf)
{
    return mbuf->mb_position;
}

int
membuf_setposition(struct membuf *mbuf, size_t position)
{
    if (position > mbuf->mb_limit)
        return -ENOBUFS;
    mbuf->mb_position = position;
    if (position < mbuf->mb_mark)
        mbuf->mb_mark = 0;
    return 0;
}

size_t
membuf_limit(struct membuf *mbuf)
{
    return mbuf->mb_limit;
}

int
membuf_setlimit(struct membuf *mbuf, size_t limit)
{
    if (limit > mbuf->mb_capacity)
        return -ENOBUFS;
    mbuf->mb_limit = limit;
    return 0;
}

void
membuf_clear(struct membuf *mbuf)
{
    mbuf->mb_position = 0;
    mbuf->mb_limit = mbuf->mb_capacity;
    mbuf->mb_mark = 0;
}

void
membuf_flip(struct membuf *mbuf)
{
    mbuf->mb_limit = mbuf->mb_position;
    mbuf->mb_position = 0;
}

void
membuf_mark(struct membuf *mbuf)
{
    mbuf->mb_mark = mbuf->mb_position;
}

void
membuf_reset(struct membuf *mbuf)
{
    mbuf->mb_position = mbuf->mb_mark;
}

void
membuf_rewind(struct membuf *mbuf)
{
    mbuf->mb_position = mbuf->mb_mark = 0;
}

ssize_t
membuf_memptrs(struct membuf *mbuf, struct membuf_iov *iov, size_t iovcnt)
{
    iov[0].iov_base = ((unsigned char *) mbuf->mb_buf) + mbuf->mb_position;
    iov[0].iov_len = membuf_remaining(mbuf);
    return 1;
}

ssize_t
membuf_putmembuf(struct membuf *mbuf, struct membuf *src)
{
    struct membuf_iov dstiov[1];
    struct membuf_iov srciov[1];
    size_t nbytes = membuf_remaining(src);

    if (nbytes > membuf_remaining(mbuf))
        return -ENOBUFS;
    membuf_memptrs(mbuf, dstiov, 1);
    membuf_memptrs(src, srciov, 1);
    nbytes = memcpyv(dstiov, 1, srciov, 1, nbytes);
    mbuf->mb_position += nbytes;
    src->mb_position += nbytes;
    return (ssize_t) nbytes;
}

ssize_t
membuf_putbytes(struct membuf *mbuf, const unsigned char *buf, size_t count)
{
    if (mbuf->mb_position + count > mbuf->mb_limit)
        return -ENOBUFS;
    memcpy(((unsigned char *) mbuf->mb_buf) + mbuf->mb_position, buf, count);
    mbuf->mb_position += count;
    return (ssize_t) count;
}

ssize_t
membuf_getbytes(struct membuf *mbuf, unsigned char *buf, size_t count)
{
    if (mbuf->mb_position + count > mbuf->mb_limit)
        return -ENOBUFS;
    memcpy(buf, ((unsigned char *) mbuf->mb_buf) + mbuf->mb_position, count);
    mbuf->mb_position += count;
    return (ssize_t) count;
}
