
// BEGIN MASTER LIB INCLUDE BLOCK
// This is done to circumvent the limitations of the Arduino IDE
#include <Wire.h>
#include <I2Cdev.h>
#include <HMC5883L.h>
// END LIBS


// Global Configuration

#define DRONE_DEBUG_SHOW_TASKS 0
#define DRONE_DEBUG_SHOW_GPS 0

#define DRONE_HAS_FRONTEND 0
#define DRONE_HAS_OLED 0
#define DRONE_HAS_NANO 0
#define DRONE_HAS_LOGGER 1
#define DRONE_HAS_HELPER 1
#define DRONE_IGNORE_SHUTDOWN 0
#define DRONE_WIFI_NEEDY 1

#define DRONE_NAV_SUPRESS_MOTORS 0
#define DRONE_COMPASS_REVERSE 1
#define DRONE_COMPASS_CALIB 11
#define DRONE_COMPASS_FIXED_DECLINATION 1
#define DRONE_COMPASS_FIXED_DECLINATION_VALUE -19.4333333333333333
#define DRONE_RC_SPEED 0.30

#define DRONE_CONNECTION_ATTEMPT_PERIOD_MS 5000
#define DRONE_STATION_HOST "192.168.43.48"
#define DRONE_HELPER_HOST "192.168.43.1"
#define DRONE_HELPER_PORT 7777

// now some sensible code:

#include "drv_i2c.h"
#include "drv_adc.h"
#include "drv_gps.h"

#include "scheduling.h"
#include "blackboard.h"
#include "util.h"

#include "task-sys.h"
#include "task-sense.h"
#include "task-nav.h"
#include "task-comm.h"

Blackboard BB;

static const micros_t PERIOD = 40000;

CEScheduler<4, 2, PERIOD> scheduler;

void createScheduleTable()
{
  // created here
  static SysTask sysTask;
  static CommTask commTask;
  static SenseTask senseTask;
  static NavTask navTask(&commTask);

  scheduler.setTask(&sysTask, 0); // task 0 is sys
  scheduler.setTask(&senseTask, 1); // task 1 is sense
  scheduler.setTask(&navTask, 2); // task 2 is nav
  scheduler.setTask(&commTask, 3); // task 3 is comm

  scheduler.setCompartment( BIT(0) | BIT(1) | BIT(2), 0 ); // first compartment: sys, sense, nav
  scheduler.setCompartment( BIT(0) | BIT(1) | BIT(3), 1 ); // second compartment: sys, sense, comm
}

void setup()
{    
  // drop serial console on Serial2 (ttyS1)
  system("systemctl stop serial-getty@ttyS1.service"); 
  
  // (0.5s) avoid writing to serial so fast
  usleep(500 * 1000); 
  Util::init();
  
  Util::log("Hydrus Drone");
  
  Util::log("ADC Init");
  ADC::init();
  Util::log("I2C Init");
  I2C::init();
  Util::log("GPS Init");
  GPS::init();

  Util::log("Creating tasks");
  createScheduleTable();
  
  Util::log("Starting loop");
}

void loop() {
  scheduler.exec();
}

// BEGIN SOURCE INCLUDE BLOCK
// we need to add code here as headers so that the arduino IDE parses the Library includes
#include "drv_i2c_cpp.h"
#include "drv_gps_cpp.h"
#include "drv_frontend_cpp.h"
// END SOURCE INCLUDE BLOCK
