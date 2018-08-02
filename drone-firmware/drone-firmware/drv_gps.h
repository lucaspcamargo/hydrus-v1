#pragma once

#include "util.h"

#include "threading/semaphore.h"
#include "threading/mutex.h"

class GPS
{
public:

  static const bool GPS_FAST = false;
  static const bool GPS_ENABLE_5HZ = false; // depends on GPS_FAST
  static const unsigned GPS_UPDATE_RATE = 30;
  static const unsigned GPS_MAX_LINE_LENGTH = 160;

  static const int PIN_ON_OFF = 7;
  static const int PIN_WAKEUP = 8;
  static const int PIN_RESET = 9;
  
  static void init();

  static void reset();
  static void turnOn();
  static void turnOff(bool saveData = true);
  
  static void enable5Hz();
  static void disable5Hz();
  
  static void beginBinaryMode();
  static void endBinaryMode();
  static void sendBinaryCommand(void* payload, int size);
  
  static void restoreData();
  static void saveData();  
  
  
  static void tick(micros_t us);

  static void * service(void*);

  typedef std::queue<std::string> Queue;
  
private:
  static void checkLine();
  static void dispatchLine();
  
  static int getBinaryMessageData(uint8_t messageType, void *buffer);

  static char linebuf[GPS_MAX_LINE_LENGTH];
  static int linecount;
  
  static volatile bool _binaryMode;

  static Queue _queue;
  static Semaphore _sem;
  static Mutex _mtx;


};
