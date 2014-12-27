#ifndef _LOGGER_HPP
#define _LOGGER_HPP

#include <Print.h>

//
// LoggerImpl
//
class LoggerImpl
{
public:
  virtual size_t append(const uint8_t* buf, size_t nbytes);
  virtual void flush();
};


//
// Logger
//
class Logger
{
private:
  LoggerImpl* impl_;

public:
  Logger();
  void setImpl(LoggerImpl* impl);

  void append(const uint8_t* buf, size_t nbytes);
  void flush();

  void msg(const char* msg);
  void vprintf(const char* format, va_list ap);
  void printf(const char* format, ...);
};


//
// PrintLogger
//
class PrintLogger : public LoggerImpl
{
private:
  Print& print_;
public:
  PrintLogger(Print& print);
  size_t append(const uint8_t* buf, size_t nbytes);
  void flush();
};

extern Logger logger;

#endif // _LOGGER_HPP
