#pragma once

#include <cmath>

#include "scheduling.h"
#include "blackboard.h"
#include "util.h"
#include "drv_motors.h"
#include "geomag.h"
#include "task-comm.h"

#include "nav-controller.h"

class NavTask: public CETask< s2micros(0.04) >, public CommTask::CmdListener
{
public:
    
  NavTask(CommTask * comm)
  {
    m_motors.init();
    comm->setNavigationCmdListener(this);
  }
  
  void parseRoute(const std::string & routecmd)
  {      
      stringvec_t tok;
      Util::split(routecmd, ',', tok);
      int idx = 2;
      
      Waypoints wps;
      
      while(tok.size() - idx >= 3)
      {
          Waypoint w;
          
          w.longitude = Util::parseDouble(tok[idx]);
          w.latitude = Util::parseDouble(tok[idx+1]);
          w.acquire = tok[idx+2].size() > 0;
          w.customRadius = -1;
          
          wps.push_back(w);
          
          idx += 3;
          
          
        std::stringstream ss;
        ss << "WAYPOINT ";
        ss << w.longitude << " " << w.latitude << " ";
        ss << (w.acquire? "A" : "");
        Util::log("Nav", ss.str().c_str());
        
      }
      
      if(validateRoute(wps))
      {
          m_route = wps;
          Util::logf("Nav", "Received new route %s with %d waypoints", Util::LS_INFO, true, tok[1].c_str(), wps.size());
      }
      else
      {
          wps.clear();
          Util::log("Nav", "Received invalid route. Current route cleared for safety.", Util::LS_ERROR);
      }
      
  }
  
  bool validateRoute(const Waypoints &wps)
  {
      return wps.size() > 1; // more complex logic can be put here      
  }
  
  virtual void onCommandReceived(const std::string & cmd)
  {
    if(!cmd.find("$NAVROUTE,"))
    {
        parseRoute(cmd);
    }
    
    if(!cmd.find("$NAVBEGIN"))
    {
      beginNavigation();
    }
  }

  void tick()
  {
#if DRONE_DEBUG_SHOW_TASKS
    Util::log("Nav", "TICK");
#endif
    
    if(BB.nav.rcMode)
    {
      BB.nav.motor.enabled = true;
      // TODO proper trigonometry maybe
      BB.nav.motor.spdL = fmax(0, +DRONE_RC_SPEED*BB.nav.rcVecX + fmax(0, DRONE_RC_SPEED*BB.nav.rcVecY));
      BB.nav.motor.spdR = fmax(0, -DRONE_RC_SPEED*BB.nav.rcVecX + fmax(0, DRONE_RC_SPEED*BB.nav.rcVecY));
    }
    else
    {
      if(BB.sys.state == SS_NAVIGATING)
      {
          bool cont = m_control.step(0.04);
          if(!cont) 
          {
              BB.nav.state = NS_NOT_NAVIGATING;
              BB.sys.state = SS_READY;
          }
      }
      else
      {
          BB.nav.motor.enabled = false;
          BB.nav.motor.spdL = 0;
          BB.nav.motor.spdR = 0;
      }
    }
      
    m_motors.tick();
  }
  
  void beginNavigation()
  {
    
    // check navigation state here
    
    if(!m_route.size())
    {
      Util::log("Nav", "No valid route specified. Can't begin navigation.");
      return;
    }
    
    if(!BB.nav.gpsHasFix)
    {
      Util::log("Nav", "Drone GPS has no fix. Can't begin navigation.");
      return;
    }
    
    if(!m_control.withinRange(m_route[0].longitude, m_route[0].latitude))
    {      
      std::stringstream ss;
      ss << "Drone is not within range of base station location ( ";
      // TODO write location name and distance
      Util::log("Nav", ss.str().c_str());
    }
    
    m_control.setup(m_route);
  }

private:
  
  Waypoints m_route;
  NavController m_control;
  
  Motors m_motors;
  
}; 
