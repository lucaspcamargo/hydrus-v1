#pragma once

#include <pthread.h>
#ifdef THREAD_CPUAFFINITY
#include <sched.h>
#endif
#include <cstdlib>
#include <iostream>

class Thread {

  public:

    /* NOTE to be implemented when necessary
      enum State{
        STOPPED,
        RUNNING,
        SLEEPING
      };*/

    Thread(int cpuAffinity = -1, bool lowPriority = false):
      _ret(0)
    {
      pthread_attr_t attrs;
      pthread_attr_init(&attrs);

      pthread_attr_setdetachstate(&attrs, PTHREAD_CREATE_JOINABLE);

#ifdef THREAD_CPUAFFINITY
      if (cpuAffinity >= 0)
      {
        cpu_set_t cpu_set;
        CPU_ZERO(&cpu_set);
        CPU_SET(cpuAffinity, &cpu_set);

        pthread_attr_setaffinity_np(&attrs, sizeof(cpu_set_t), &cpu_set);
      }
#endif
      int status = pthread_create(&_thread, &attrs, &_impl_runThread, this);

      if (status != 0)
      {
        std::cerr << "pthread_create failed with status " << status << std::endl;
        exit(-1);
      }

      if(lowPriority)
      {
        int policy;
        pthread_attr_getschedpolicy(&attrs, &policy);
  
        int prio = sched_get_priority_min(policy);
        pthread_setschedprio(_thread, prio);
    
      }
      
    }

    ~Thread()
    {
      // TODO destructor
      // verify state of thread
      // maybe join by default???
    }

    static void * _impl_runThread(void * arg)
    {
      return static_cast<Thread*>(arg)->exec();
    }

    void * exec()
    {
      _ret = main();
      return this;
    }

    virtual int main() {
      return 0;
    }

    int join()
    {
      pthread_join(_thread, 0 );
      return _ret;
    }
    
  private:
    int _ret;
    pthread_t _thread;

};