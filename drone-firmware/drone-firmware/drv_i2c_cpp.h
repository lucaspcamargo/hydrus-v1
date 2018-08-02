#pragma once

#include "drv_i2c.h"
#include "threading/thread.h"

// class members
I2C::Queue I2C::_queue; 
I2C::Handlers I2C::_handlers;
Semaphore I2C::_queueSemaphore(0);
Mutex I2C::_queueMutex, I2C::_busMutex;

class I2CThread: public Thread
{
public:
  I2CThread(): Thread(-1, true){}

  int main()
  {
    while(true)
      I2C::service(0);
    return 0;
  }
};

void I2C::init() 
{
  if(thread_enabled)
      new I2CThread();
      
  Wire.begin();
}

void * I2C::service(void*)
{
  int limit = service_rq_limit;
  while(_queueSemaphore.try_p())
  {
    
    _queueMutex.lock();

    Op op = _queue.front(); // front when queue
    _queue.pop();
    
    _queueMutex.unlock();

    // now execute operation

    _busMutex.lock();
    
    if(op.addr & 0x80)
    {
      // write
      op.addr -= 0x80; // clear msb
      Wire.beginTransmission(op.addr);
      Wire.write(op.reg);
      Wire.write(op.data);
      Wire.endTransmission();      
    }
    else
    {
      // read
      // todo Wire read
      // todo call handler
    }

    _busMutex.unlock();

    if(!--limit) break;
    
  }
  
  return 0;
}

