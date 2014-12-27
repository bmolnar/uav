#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/uio.h>

#include "xstream.h"
#include "membuf.h"


static int
xstream_membuf_open_fn(xstream_t *xs, const char *mode, void *userdata)
{
    struct membuf *mbuf = (struct membuf *) userdata;
    XSTREAM_PRIV(xs) = mbuf;
    return 0;
}

static ssize_t
xstream_membuf_write_fn(xstream_t *xs, const void *buf, size_t count)
{
    struct membuf *mbuf = (struct membuf *) XSTREAM_PRIV(xs);
    return membuf_putbytes(mbuf, buf, count);
}

static ssize_t
xstream_membuf_read_fn(xstream_t *xs, void *buf, size_t count)
{
    struct membuf *mbuf = (struct membuf *) XSTREAM_PRIV(xs);
    return membuf_getbytes(mbuf, buf, count);
}

static int
xstream_membuf_close_fn(xstream_t *xs)
{
    return 0;
}

static int
xstream_membuf_flush_fn(xstream_t *xs)
{
    return 0;
}

static xstream_ops_t xstream_membuf_ops = {
    .open  = &xstream_membuf_open_fn,
    .write = &xstream_membuf_write_fn,
    .read  = &xstream_membuf_read_fn,
    .close = &xstream_membuf_close_fn,
    .flush = &xstream_membuf_flush_fn,
};

xstream_t *
xstream_membuf_open(const char *mode, struct membuf *mbuf)
{
    return xstream_open(xstream_membuf_ops, mode, mbuf);
}
