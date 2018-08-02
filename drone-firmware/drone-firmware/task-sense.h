#pragma once

#include "scheduling.h"
#include "blackboard.h"
#include "drv_gps.h"
#include "util.h"
#include "stdlib.h"
#include "sensors/Nano.h"
#include "sensors/Magnetometer.h"
#include "sensors/Temperature.h"
#include "sensors/Turbidity.h"
#include "sensors/PHProbe.h"

#include <fstream>

class SenseTask: public CETask<s2micros(0.02)>
{
public:

  static const unsigned int PIN_BATT_SENSE = A0;
  static const unsigned int PIN_GPS_FIX = 4;

  SenseTask():
  nano(0)
  {
      pinMode(PIN_GPS_FIX, OUTPUT);
  }

  void tick()
  {
#if DRONE_DEBUG_SHOW_TASKS
      Util::log("Sense", "TICK");
#endif
    static int ticks = 0;
    
    if(BB.sys.state == SS_HALTED)
        return;
    
#if !DRONE_HAS_HELPER
    // tick sensors
    GPS::tick(micros());
    BB.sensors.imuHeading = mag.readMagnetometer();
#endif
    
    digitalWrite(PIN_GPS_FIX, BB.nav.gpsHasFix? HIGH : LOW);
    
    static int lastSeconds = -1;
    int s = (int) seconds();
    if(s != lastSeconds)   
    {
      updateSystemStats();
      lastSeconds = s;
    }
    
#if DRONE_HAS_NANO
    if(!nano)
    {
        nano = new Nano();
    }
#endif

    if(!(ticks % 50))
    {
        
      if(nano) nano->requestReadings();
      
      BB.sensors.waterTemp = tempWater.readTemperature();
      
      BB.sensors.waterTurb = turbidity.readTurbidity();
      
      BB.sensors.waterPH = ph.readPH();
    }
    
    if(nano && nano->canRead())
    {
        nano->readData();
        
        BB.sensors.sonarA = nano->sonarCM(0) * 0.01f;
        BB.sensors.sonarB = nano->sonarCM(1) * 0.01f;
        BB.sensors.sonarC = nano->sonarCM(2) * 0.01f;
        BB.sensors.sonarD = nano->sonarCM(3) * 0.01f;
        
        BB.sensors.weatherTemp = nano->temperatureC();
        BB.sensors.weatherHum = nano->humidityPercentage();
        BB.sensors.weatherLight = nano->ldrADC();
        
        /*
        std::stringstream ss;
        ss << "NANO ";
        ss << nano->sonarCM(0) << " ";
        ss << nano->sonarCM(1) << " ";
        ss << nano->sonarCM(2) << " ";
        ss << nano->sonarCM(3) << " ";
        ss << nano->humidityPercentage() << " ";
        ss << nano->temperatureC() << " ";
        ss << nano->ldrADC() << " ";
        
        Util::log("SENSE", ss.str().c_str());
        */
    }
    
    ticks ++;
    
  }

  void updateSystemStats()
  {
    // read battery voltage
    // calculated by using least squares over the following dataset.
    //451 12.54
    //520 14.86
    //543 15.50
    //561 16.00
    // y = 3.17118346·10-2 x - 1.725514196

      
    I2C::lockBus();
    float v = ADC::read(PIN_BATT_SENSE);
    I2C::unlockBus();
    v = v*32.4412067958 - 1.725514196f; 
    BB.sys.battVoltage = v;//(BB.sys.battVoltage == 0.0f? v : BB.sys.battVoltage * 0.8 + v * 0.2);
    BB.sys.battLevel = (BB.sys.battVoltage - 12.0f)* 0.25f;

    BB.sys.coreTemp = readSOCTemperature();

    BB.sys.cpuLoad = readCPULoad();
  }

  float readSOCTemperature()
  {
    char temp_raw[6];
    
    FILE* fp = fopen("/sys/class/thermal/thermal_zone0/temp", "r"); 
    fgets(temp_raw, 5, fp);
    fclose(fp);
    
    int temp = atoi(temp_raw);
    return temp * 0.01f; 
  }
  
  float readCPULoad()
  {
    float ret = 0;
    
    static bool initialized = false;
    static int ousr, onice, osys, oidle;
    
    int usr, nice, sys, idle;

    std::fstream in("/proc/stat", std::ios_base::in);
    in.ignore(4);
    in >> usr >> nice >> sys >> idle;
    in.close();

    if(initialized)
    {
      int dusr = usr - ousr;
      int dnice = nice - onice;
      int dsys = sys - osys;
      int didle = idle - oidle;

      ret = (dusr+dnice+dsys)*100.0f/(dusr+dnice+dsys+didle);

    }
    else initialized = true;
    
    ousr = usr;
    onice = nice;
    osys = sys;
    oidle = idle;   

    return ret;
  }

  private:
    Nano *nano;
#if !DRONE_HAS_HELPER
    Magnetometer mag;
#endif
    Temperature tempWater;
    Turbidity turbidity;
    PHProbe ph;
      
};
