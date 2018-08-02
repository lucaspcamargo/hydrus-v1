#pragma once

#include <vector>
#include <cstdarg>

#include "util_strfunc.h"

// TYPES
typedef unsigned long micros_t;
typedef float sec_t;

// MACROS
#define BIT(x) (1 << (x))
#define s2micros(x) ((micros_t)((x)*1000000))
#define micros2s(x) ((sec_t)((x)*1e-6f))

// FUNCTIONS
namespace Util 
{
  enum LogSeverity
  {
    LS_UNSET,
    LS_FLOOD,
    LS_INFO,
    LS_WARNING,
    LS_ERROR,
    LS_CRITICAL
  };

  LogSeverity logSeverity( LogSeverity s = LS_UNSET )
  {
    static LogSeverity ls = LS_INFO;
    if(s != LS_UNSET)
    {
      ls = s;
    }

    return ls;
  }

  const char * logSeverityStr( LogSeverity s )
  {
    static const char * const vals[] = {"UNSET", "FLOOD", "INFO", "WARN",
                                       "ERR", "CRIT"};
    return vals[(int)s];
  }

  class LogListener
  {
    public:
      virtual void logged(const char * from, const char * msg, LogSeverity sev) = 0;
  };

  std::vector<LogListener*> s_logListeners;

  void dispatchLog(const char * from, const char * msg, LogSeverity sev)
  {
    std::vector<LogListener*>::iterator it;
    for(it = s_logListeners.begin(); it != s_logListeners.end(); it++)
    {
      (*it)->logged(from, msg, sev);
    }
  }

  void init()
  {
      
#ifndef DRONE_UTIL_NO_SERIAL
    Serial.begin(115200);

    Serial.print("Hydrus Project\r\n");
    Serial.print("INITIALIZING\r\n\r\n");
#endif
    
  }

  void log(const char * msg, LogSeverity sev = LS_INFO, bool dispatch = true)
  {
    if(logSeverity() > sev)
      return;
#ifndef DRONE_UTIL_NO_SERIAL
    Serial.print(msg);
    if(msg[strlen(msg)-1] != '\n')
      Serial.print("\r\n");
#endif
    if(dispatch) dispatchLog(0, msg, sev);
  }

  void log(const char * from, const char * msg, LogSeverity sev = LS_INFO, bool dispatch = true)
  {
    if(logSeverity() > sev)
      return;
    
#ifndef DRONE_UTIL_NO_SERIAL
    Serial.print("[");
    Serial.print(from);
    Serial.print("] ");
    Serial.print(logSeverityStr(sev));
    Serial.print(": ");
#endif
    
    log(msg, sev, false);
    
    if(dispatch) dispatchLog(from, msg, sev);
  }
  
  void logf(const char * from, const char * msg, LogSeverity sev = LS_INFO, bool dispatch = true, ...)
  {
    char buffer[256];
    va_list args;
    va_start (args, msg);
    vsprintf (buffer, msg, args);
    log(from, buffer, sev, dispatch);
    va_end (args);
  }
}
