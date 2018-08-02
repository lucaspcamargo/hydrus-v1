#ifdef HYDRUS_DRONE_TEST

#define DRONE_UTIL_NO_SERIAL 1

#include <iostream>
#include <cstdio>

#include "../util.h"
#include "../blackboard.h"
#include "../logger.h"

Blackboard BB;

int main(int argc, char **argv)
{
    using namespace std;
    
    Logger logger;
    
    if(!logger.isOpen())
    {
        cerr << "Logger not open.\n";
        return -1;
    }
    
    
    for(int i = 0; i < 200; i++)
    {
    
    logger.logged("Test", "Test message", Util::LS_INFO);
    cerr << "Test message.\n";
    
    BB.nav.gpsHasFix = true;
    BB.nav.gpsLat = i;
    BB.nav.gpsLat = 2*i;
    logger.saveBlackboard();
    cerr << "Test blackboard.\n";
    }
    
    logger.close();
    cerr << "Test completed.\n";
    return 0;
}

#endif