#pragma once

#include <ctime>
#include <fstream>
#include <sys/time.h>
#include <stdint.h>

class Logger: public Util::LogListener
{
public:
    Logger()
    {
        system("mkdir -p /opt/hydrus");
        
        std::stringstream ss;
        ss << "/opt/hydrus/";
        ss << ((unsigned long int) time(NULL));
        ss << ".log";

        m_logFile.open(ss.str().c_str(), std::ios_base::out | std::ios_base::binary);
        m_open = m_logFile.is_open();
        
        Util::s_logListeners.push_back(this);
        
//         if(m_open)
//             Serial.println("OPEN");
//         else
//             Serial.println("CLOSED");
    }
    
    ~Logger()
    {
        if(m_open)
            close();
    }
    
    bool isOpen()
    {
        return m_open;
    }
    
    void logTimestamp()
    {
        
//         Serial.println("TIMESTAMP");
        struct timeval tv;
        gettimeofday(&tv, NULL);
        uint64_t time_in_micros = 1000000 * tv.tv_sec + tv.tv_usec;
        
        m_logFile.write((const char*) &time_in_micros, sizeof(uint64_t));
    }
    
    
      void logged(const char * from, const char * msg, Util::LogSeverity sev)
      {
//             Serial.println("LOGGED");
          if(!m_open) return;
          
          std::stringstream ss;
          ss << ("[");
          ss << (from);
          ss << ("] ");
          ss << (logSeverityStr(sev));
          ss << (": ");
          ss << (msg);
          
          std::string message = ss.str();
          
          uint32_t size = message.size();
          
          
          m_logFile << '$';
          logTimestamp();
          m_logFile.write((const char*) &size, sizeof(uint32_t));
          m_logFile.write(message.c_str(), size);
      }
      
      void saveBlackboard()
      {
          
//             Serial.println("BB");
          if(!m_open) return;
          
          m_logFile << '@';
          logTimestamp();
          m_logFile.write((const char*) &BB, sizeof(Blackboard));
      }
      
      void close()
      {
          
//             Serial.println("CLOSE");
          if(!m_open) return;
          
          m_logFile.put('E');
          m_logFile.close();
          
          m_open = false;
      }
      
    
public:
    std::fstream m_logFile;
    bool m_open;
};