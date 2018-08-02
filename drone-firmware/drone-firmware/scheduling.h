#pragma once

#include <string>
#include "util.h"

#include "threading/semaphore.h"
#include "threading/thread.h"
#include "threading/periodic_thread.h"

#define DRONE_PROFILER_ENABLED 0

class CETaskBase
{
public:
  virtual void perform_tick() = 0;
};

template <micros_t INTERVAL>
class CETask: public CETaskBase
{
public:
  static const micros_t INTERVAL_US = INTERVAL;

  micros_t frequency() const { return 1000000ul / INTERVAL; }

  virtual void tick() = 0;
  
  virtual void perform_tick()
  {
    tick();
    _absMicros += INTERVAL;
    _absTime += micros2s(INTERVAL);
  }

  sec_t seconds() const
  {
    return _absTime;
  }
  
  micros_t micros() const
  {
    return _absMicros;
  }
  
private:
  sec_t _absTime;
  micros_t _absMicros;
};

template < int COMPART_US >
class CETimer : public Thread
{
  public:
    CETimer(): Thread(), _s(0) {}
  
    int main()
    {
      for(;;)
      {
        usleep(COMPART_US);
        _s.v();
      }
      return 0;
    }

    void wait_next() { _s.p(); }

  private:
    Semaphore _s;
};

template <int NUM_TASKS>
class CEProfiler
{
public:
    
    CEProfiler()
    {
        m_running = false;
    }
    
    void taskBegun(int idx, uint64_t micros)
    {
        m_runningIdx = idx;
        m_runningMicros = micros;
    }

    void taskEnded(int idx, uint64_t micros)
    {
        if(m_runningIdx == idx)
        {
            uint64_t delta = micros - m_runningMicros;
            
            m_numRuns[idx]++;
            m_worstTime[idx] = m_worstTime[idx]<delta? delta : m_worstTime[idx];
            m_accumTime[idx] += delta;
            
        }
        else
        {
            // something bad happened
            // or improper usage
            m_runningIdx = -1; 
        }
    }
    
    bool running() const
    {
        return m_running;
    }
    
    void start()
    {
        m_running = true;
        
        m_runningIdx = -1;
        m_runningMicros = 0;
        
        for(int i = 0; i < NUM_TASKS; i++)
            m_numRuns[i] = m_accumTime[i] = m_worstTime[i] = 0;
    }
    
private:
    bool m_running;
    
    int m_runningIdx;
    uint64_t m_runningMicros;
    
    uint64_t m_worstTime[NUM_TASKS];
    uint64_t m_accumTime[NUM_TASKS];
    uint64_t m_numRuns[NUM_TASKS];
};

template <int NUM_TASKS, int NUM_COMPARTMENTS, int COMPART_US>
class CEScheduler
{
public:
  typedef unsigned char task_mask_t;
  
  CEScheduler()
  {
    memset(_tasks, 0, sizeof(_tasks));
    memset(_schedule, 0, sizeof(_schedule));
  }

  void setTask(CETaskBase * t, int index)
  {
    _tasks[index] = t;
  }

  void addTaskToCompartment(int taskI, int compartmentI)
  {
    _schedule[compartmentI] |= 1 << taskI;
  }
  
  void setCompartment(task_mask_t mask, int compartmentI)
  {
    _schedule[compartmentI] = mask;
  }

  void exec()
  {
    int currCompartment = 0;
    
    for(;;)//ever
    {
      // HACK
      //_timer.wait_next();
      usleep(10000);
      
      // this is a tick
      // now we run the tasks
      for(int i = 0; i < NUM_TASKS; i++)
      {
        if(!_tasks[i])
          continue;

        if(_schedule[currCompartment] & BIT(i))
        {            
#if DRONE_PROFILER_ENABLED
          m_profiler.taskBegun(i, micros());
#endif
          
          _tasks[i]->perform_tick();      

#if DRONE_PROFILER_ENABLED
          m_profiler.taskEnded(i, micros());
#endif
        }
        
      }

      // HACK here we service I2C synchronously with the main thread
      // we would have problems otherwise (galileo bug?)
      if(!I2C::thread_enabled)
        I2C::service(0);
      
      currCompartment = (currCompartment+1) % NUM_COMPARTMENTS;
    } // forever
  }
  
private:
  CETaskBase * _tasks[NUM_TASKS];
  task_mask_t _schedule[NUM_COMPARTMENTS];
  CETimer<COMPART_US> _timer;
  
#if DRONE_PROFILER_ENABLED
  CEProfiler<NUM_TASKS> m_profiler;
#endif
};
