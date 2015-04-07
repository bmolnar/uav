#include "Logger.hpp"

#include "xstream.h"

#include <Print.h>

//
// Logger methods
//
static ssize_t Logger_writefn(xstream_t *xs, const void *buf, size_t nbytes)
{
  Logger *l = (Logger*) XSTREAM_PRIV(xs);
  l->append((uint8_t *) buf, nbytes);
  return nbytes;
}
static int Logger_flushfn(xstream_t *xs)
{
  Logger *l = (Logger*) XSTREAM_PRIV(xs);
  ((void) l);
  return 0;
}
static xstream_ops_t logger_xstream_ops = {
  NULL,
  &Logger_writefn,
  NULL,
  &Logger_flushfn,
  NULL
};


Logger::Logger()
{
}
size_t Logger::append(const uint8_t* buf, size_t nbytes)
{
  return nbytes;
}
void Logger::flush()
{
}
void Logger::msg(const char* msg)
{
  append((uint8_t*) msg, strlen(msg));
  flush();
}
void Logger::vprintf(const char* format, va_list ap)
{
  xstream_ops_t xs_ops = { NULL, &Logger_writefn, NULL, &Logger_flushfn, NULL };
  xstream_t xs;

  xstream_init(&xs, xs_ops);
  XSTREAM_PRIV(&xs) = (void *) this;
  vxprintx(&xs, format, ap);

  flush();
}
void Logger::printf(const char* format, ...)
{
  va_list ap;

  va_start(ap, format);
  vprintf(format, ap);
  va_end(ap);
}



//
// PrintLogger methods
//
PrintLogger::PrintLogger(Print& print)
  : print_(print)
{
}
size_t PrintLogger::append(const uint8_t* buf, size_t nbytes)
{
  return print_.write(buf, nbytes);
}
void PrintLogger::flush()
{
  print_.println();
}


NullLogger nullLogger;
Logger& logger = nullLogger;

void setLogger(Logger& l)
{
  logger = l;
}
