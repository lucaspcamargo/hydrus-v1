#pragma once

#include <queue> //for priority queue
#include <map>
#include <stdint.h> //for uint8_t

// my threading classes
#include "threading/semaphore.h"
#include "threading/mutex.h"

class I2C
{
public:

  // config
    
  static const bool thread_enabled = false;
  static const int service_rq_limit = 30;  
    
    
  // enums   
    
  enum Priority {
    P_LOWEST = 0,
    P_LOW = 1,
    P_MEDIUM = 16,
    P_HIGH = 32,
    P_CRITICAL = 64
  };

  // data types

  struct Op
  {
    uint8_t prio;
    uint8_t addr;
    uint8_t reg;
    uint8_t data;

    bool operator <(const Op& op) const
    {
      return prio < op.prio;
    }
  };

  //typedef std::priority_queue<Op> Queue;
  typedef std::queue<Op> Queue;

  typedef Op Reply; // for now use same struct
  
  class Handler
  {
    virtual void handle(const Reply & res) = 0;
  };
  
  typedef std::map<uint8_t, Handler *> Handlers;


  // class methods

  static void init();

  static void registerHandler(uint8_t address, Handler * hnd)
  {
    _handlers[address] = hnd;
  }

  static void read(uint8_t addr, uint8_t reg, uint8_t priority)
  {
    _queueMutex.lock();

    _queue.push(Op{ priority, addr, reg, 0x00 });
    
    _queueMutex.unlock();
    _queueSemaphore.v();
  }
  
  static void write(uint8_t addr, uint8_t reg,  uint8_t value, uint8_t priority)
  {
    _queueMutex.lock();

    _queue.push(Op{ priority, addr | 0x80, reg, value }); // sets highest bit for a write operation
    
    _queueMutex.unlock();
    _queueSemaphore.v();
  }

  /*
  static uint8_t readNow(uint8_t addr, uint8_t reg)
  {
      lockBus();
      
      //...
      
      unlockBus();
  }

  static void writeNow(uint8_t addr, uint8_t reg, uint8_t value)
  {
      lockBus();
      
      //..
      
      unlockBus();
  }*/
      
  
  
  static bool busy()
  {
      
    bool ret = _queueSemaphore.try_p();
    if(ret) _queueSemaphore.v();
    return ret;
  }

  static void lockBus()
  {
    _busMutex.lock();
  }

  static void unlockBus()
  {
    _busMutex.unlock();
  }

  static void * service(void*);

private:
  static Queue _queue;
  static Handlers _handlers;
  static Semaphore _queueSemaphore;
  static Mutex _queueMutex, _busMutex;
};

