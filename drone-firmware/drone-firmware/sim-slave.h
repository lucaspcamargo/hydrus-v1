#pragma once

#include "task-comm.h"

class SimulationSlave: public CommandListener
{
public:
    virtual void onCommandReceived(const std::string & cmd)
    {
        if(!cmd.find("$SIMON"))
        {
            Util::log("SimSlave", "Simulation mode ON");
            BB.nav.simMode = true;
        }
    
        if(!cmd.find("$SIMOFF"))
        {
            Util::log("SimSlave", "Simulation mode OFF");
            BB.nav.simMode = false;
        }
        
        if(!cmd.find("$SIMDAT,"))
        {
            // this is a data simulation packet, override navigation data                            
            stringvec_t tok;
            Util::split(cmd, ',', tok);
            
            BB.nav.gpsHasFix = true;
            BB.nav.gpsLon = Util::parseDouble(tok[1]);
            BB.nav.gpsLat = Util::parseDouble(tok[2]);
            
        }
    }
};