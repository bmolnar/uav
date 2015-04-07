#ifndef _LOGGER_HPP
#define _LOGGER_HPP

#include <Print.h>

//
// Logger
//
class Logger
{
public:
  Logger();

  virtual size_t append(const uint8_t* buf, size_t nbytes);
  virtual void flush();

  void msg(const char* msg);
  void vprintf(const char* format, va_list ap);
  void printf(const char* format, ...);
};


//
// PrintLogger
//
class PrintLogger : public Logger
{
private:
  Print& print_;

public:
  PrintLogger(Print& print);
  size_t append(const uint8_t* buf, size_t nbytes);
  void flush();
};

class NullLogger : public Logger
{
};

extern Logger& logger;

void setLogger(Logger& logger);

#endif // _LOGGER_HPP
