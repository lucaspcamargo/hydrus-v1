#pragma once

#include <pthread.h>
#include <semaphore.h>

class Semaphore
{
public:
    
    Semaphore(unsigned int val = 0)
    {
        sem_init(&_s, 0/*not shared*/, val);
    }
    
    ~Semaphore()
    {
        sem_destroy(&_s);
    }
    
    bool try_p()
    {
        return sem_trywait(&_s)? /*returned EAGAIN*/ false : /* returned zero */ true;
    }
    
    void p()
    {
        sem_wait(&_s);
    }
    
    void v()
    {
        sem_post(&_s);
    }
    
private:
    sem_t _s;
    
};