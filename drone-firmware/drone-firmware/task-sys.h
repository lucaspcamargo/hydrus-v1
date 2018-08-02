#pragma once

#include "scheduling.h"
#include "blackboard.h"
#include "util.h"

#include "drv_frontend.h"

class SysTask: public CETask<s2micros(0.02)>
{
public:

  void tick()
  {
#if DRONE_DEBUG_SHOW_TASKS
      Util::log("Sys", "TICK");
#endif
      
    SystemState prev = BB.sys.state;

    if(!seconds())
    {
      Util::log("SYSTEM", "Init");
      // first tick
      BB.sys.state = SS_INITIALIZING;
#if DRONE_HAS_FRONTEND
      _fr.init();
      _fr.leds.state(SS_INITIALIZING);  
#endif
      
    }

    // debug commands
    while(Serial.available())
    {
      char cmd = Serial.read();
      parseDebugCommand(cmd);
    }

    switch(BB.sys.state)
    {
      case SS_INITIALIZING:
      if(BB.sys.initializedSubSys == BB.sys.expectedInitializedSubSys)
      {
        BB.sys.state = SS_READY;
      }
      break;
    }

#if DRONE_HAS_FRONTEND
    // verify shutdown button
    if(BB.sys.state != SS_SHUTDOWN)
    {
      if(_fr.shutdownBtn.pressed())
      {
        Util::log("SYSTEM", "Shutdown pressed");
        BB.sys.state = SS_SHUTDOWN; // tells all tasks to finish whatever it is they are doing
      }
    }
#endif

    if(BB.sys.state == SS_HALTED)
    { 
        if(!I2C::busy())  // finish writing to devices
        {
            if(BB.sys.reboot)
            system("reboot");
            else
            system("shutdown now"); // clean up linux
        }
    }

    if(BB.sys.state == SS_SHUTDOWN) 
    {
      if(BB.sys.initializedSubSys == 0) // if everything has shutdown correctly
      {
        // TODO do not do this when not booting up from sd card
        BB.sys.state = SS_HALTED;
        
#if DRONE_HAS_FRONTEND
#if DRONE_HAS_OLED
        _fr.oled.turnOff();
#endif
#endif
      }
    }

    // advance subsystems
    if(BB.sys.state != SS_HALTED)
    {
        
#if DRONE_HAS_FRONTEND
        _fr.tick();
#else
     pinMode(13, OUTPUT);
     digitalWrite(13, (((int)seconds())%2)? HIGH : LOW);
#endif
    }

    // update led if state changed
    
#if DRONE_HAS_FRONTEND
    if(BB.sys.state != prev)
      _fr.leds.state(BB.sys.state);
#endif
    
    
  }

  void parseDebugCommand(char cmd)
  {
    if(cmd == 'e')
      BB.sys.state = SS_FAULT;
      
    if(cmd == 'h')
      BB.sys.state = SS_SHUTDOWN;
    
    if(cmd == 'm')
    {
      BB.nav.motor.enabled = !BB.nav.motor.enabled;
    }

    if(cmd >= '0' && cmd <= '9')
    {
      BB.nav.motor.spdL = BB.nav.motor.spdR = (cmd - '0') * 0.1111111111f;
    }
    
  }

private:
#if DRONE_HAS_FRONTEND
  Frontend _fr;
#endif
    
}; 
