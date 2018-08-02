#pragma once

#include <pthread.h>

class Mutex
{
public:
    Mutex() { pthread_mutex_init(&_m, 0); } 
    ~Mutex() { pthread_mutex_destroy(&_m); }
    bool tryLock() { return pthread_mutex_trylock(&_m) == 0; }
    void lock() { pthread_mutex_lock(&_m); }
    void unlock() { pthread_mutex_unlock(&_m); }
    pthread_mutex_t * toPThread(){ return &_m; }
    
private:
    pthread_mutex_t _m;
};

class MutexLock
{
public:
    MutexLock(Mutex & m):_m(m){ _m.lock(); }
    ~MutexLock(){ _m.unlock(); }
    
private:
    Mutex &_m;
};