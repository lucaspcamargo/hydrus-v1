#pragma once

#include "scheduling.h"
#include "blackboard.h"
#include "util.h"

#include "station.h"
#include "threading/thread.h"

#include "logger.h"
#include "helper-client.h"

class CommandListener 
{
  public:
      virtual void onCommandReceived(const std::string & cmd) = 0;
};
  
#include "sim-slave.h"

class CommTask: public CETask<s2micros(0.04)>, public Util::LogListener
{
public:

#if DRONE_WIFI_NEEDY
  class WiFi: public Thread {
    public:
      WiFi(): Thread(-1, true)
      {
          enable();
      }
      
      void enable()
      {
#if DRONE_DEBUG_SHOW_TASKS    
      //Util::log("Wifi", "enable wifi");
#endif
          system("connmanctl enable wifi");
      }
      
      void doScan()
      {
#if DRONE_DEBUG_SHOW_TASKS    
      //Util::log("Wifi", "Scan wifi");
#endif
          system("connmanctl scan wifi");
      }
      
      void connectToHydrus()
      {
#if DRONE_DEBUG_SHOW_TASKS    
      //Util::log("Wifi", "Attempt connect to wifi");
#endif
          system("connmanctl connect wifi_0024d649e4c2_487964727573_managed_psk");
      }
      
      int main()
      {
          while(true)
          {
              doScan();
              connectToHydrus();
              sleep(30);
          }
          return 0;
      }
      
  };
#endif
    
  typedef CommandListener CmdListener;
    
  CommTask():
  _st(),
  m_nav_cb(0)
#if DRONE_WIFI_NEEDY
  ,_wifi()
#endif
  {
    Util::s_logListeners.push_back(this);
  }

  void tick()
  {
#if DRONE_DEBUG_SHOW_TASKS    
      Util::log("Comm", "TICK");
#endif
      
    _st.tick();
    BB.comm.connected = _st.connected();
    std::string msg;
    while(_st.hasMessages())
    {
      msg = _st.unqueueMessage();

      route(msg);
    }
    
#if DRONE_HAS_HELPER
    _helper.tick();
#endif
    
#if DRONE_HAS_LOGGER
    if(BB.sys.state == SS_SHUTDOWN)
        m_logger.close();
    else if(BB.sys.state != SS_HALTED)
    {
        m_logger.saveBlackboard();
    }
#endif
  }

  void logged(const char * o, const char * m, Util::LogSeverity s)
  {
    std::string logline;
    if(o)
    {
      logline.append("[");
      logline.append(o);
      logline.append("] ");
    }
    logline.append(Util::logSeverityStr(s));
    logline.append(": ");
    logline.append(m);

    _st.queueLog(logline);
  }

  void route(const std::string & msg)
  {
    Util::log("Comm", msg.c_str());
    
    // do not route for now, just process the messages
    if(!msg.find("$HALT"))
    {
      BB.sys.state = SS_SHUTDOWN;
    }    
    if(!msg.find("$REBOOT"))
    {
      BB.sys.reboot = true;
      BB.sys.state = SS_SHUTDOWN;
    }
    
    if(!msg.find("$RCON"))
    {
      Util::log("Comm", "Turned on RC");
      BB.nav.rcVecX = BB.nav.rcVecX = 0;
      BB.nav.rcMode = true;
    }
    
    if(!msg.find("$RCOFF"))
    {
      Util::log("Comm", "Turned off RC");
      BB.nav.motor.spdL = BB.nav.motor.spdR = 0;
      BB.nav.rcMode = false;
    }
    
    if(!msg.find("$RCUP,"))
    {
      stringvec_t tok;
      Util::split(msg, ',', tok);
      if(tok.size() == 3)
      {
        BB.nav.rcVecX = Util::parseFloat(tok[1]);
        BB.nav.rcVecY = Util::parseFloat(tok[2]);
      }
      else
      { 
        BB.nav.rcVecX = 0;
        BB.nav.rcVecY = 0;
      }
    }
    
    if(!msg.find("$NAV"))
    {
        if(m_nav_cb)
        {
            m_nav_cb->onCommandReceived(msg);
        }
    }
    
    if(!msg.find("$SIM"))
    {
        if(!m_sim)
            m_sim = new SimulationSlave();
        
        m_sim->onCommandReceived(msg);
    }
    
    if(!msg.find("$GPS,"))
    {
        Serial1.print(msg.c_str() + 5);
        Serial1.print("\r\n");
    }
        
  }

  void setNavigationCmdListener(CmdListener * l)
  {
      m_nav_cb = l;
  }
  
private:
  Station _st;
  CmdListener *m_nav_cb;
  SimulationSlave * m_sim;
  
#if DRONE_WIFI_NEEDY
  WiFi _wifi;
#endif
  
#if DRONE_HAS_LOGGER
  Logger m_logger;
#endif
  
#if DRONE_HAS_HELPER
  HelperClient _helper;
#endif
}; 
