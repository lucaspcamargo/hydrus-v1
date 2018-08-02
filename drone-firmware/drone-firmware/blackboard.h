#pragma once

enum SystemState {
  SS_INITIALIZING,
  SS_READY,
  SS_NAVIGATING,
  SS_SHUTDOWN,
  SS_HALTED,
  SS_FAULT
};

enum NavigationState {
    NS_NOT_NAVIGATING,  // navigation not ongoing
    NS_ALIGN,           // point vessel towards waypoint
    NS_ALIGN_WAIT,      // vessel is pointing towards waypoint
    NS_TRAVERSE,        // vessel moves towards waypoint
    NS_ARRIVAL_WAIT,    // vessel in in radius of waypoint
    NS_ACQUIRE,         // vessel is acquiring data
    NS_HOMING_EMERGENCY // vessel is returning home because something is not right   
};

typedef SystemState State;

#ifndef BB_NO_TO_STRING
const char * toString(SystemState st)
{
  const char * const tbl[] = 
     {"INIT", 
      "READY",
      "NAVIGATING",
      "SHUTDOWN",
      "HALTED",
      "FAULT"};
      
  return tbl[(int) st];
}

const char * toString(NavigationState st)
{
  const char * const tbl[] =
     {"NOT NAVIGATING",
      "ALIGN",
      "ALIGN_WAIT",
      "TRAVERSE",
      "ARRIVAL_WAIT",
      "ACQUIRE",
      "HOMING EMERGENCY"};

  return tbl[(int) st];
}

#endif

struct Blackboard
{
  struct sys_struct {

    SystemState state = SS_INITIALIZING;

    int initializedSubSys = 0;
    static const int expectedInitializedSubSys = 1
#if DRONE_HAS_HELPER
- 1
#endif
    ;

    bool reboot;

    float battVoltage;
    float battLevel;
    float coreTemp;
    float cpuLoad;
  } sys;

  struct nav_struct {
    NavigationState state = NS_NOT_NAVIGATING;  
      
    State gpsState = SS_INITIALIZING;
    bool gpsHasFix = false;
    float gpsLat = 0;
    float gpsLon = 0;
    
    float geoHeading = 0;
    float distanceFromNextWaypoint;

    bool rcMode;
    float rcVecX;
    float rcVecY;

    struct motor_struct {
      bool enabled = false;
      float spdL = 0;
      float spdR = 0;
    } motor;
    
    bool simMode = false;
    
  } nav;

  struct sensors_struct
  {
    float waterTurb;
    float waterTemp;
    float waterPH;
    
    float imuHeading;
    
    
    float sonarA;
    float sonarB;
    float sonarC;
    float sonarD;
    
//     float weatherTemp;
//     float weatherHum;
//     float weatherLight;
    
  } sensors;

  struct comm_struct
  {
    bool connected;
  } comm;
  
  
};

extern Blackboard BB;
