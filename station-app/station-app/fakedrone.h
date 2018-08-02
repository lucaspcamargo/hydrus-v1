#ifndef FAKEDRONE_H
#define FAKEDRONE_H

#include <QDebug>
#include "unistd.h"
namespace Util
{
    static const int LS_FLOOD = 0;

    void log(const char * src, const char * what, int ignore = 0)
    {
        qDebug() << src << ":" << what;
    }
}
#define BB_NO_TO_STRING
#include "droneblackboard.h"
Blackboard BB;

#define STATION_SKIP_CONFIG
#define STATION_HOST "127.0.0.1"
#define NTP_KILL "killall ntpd"
#define NTP_START "ntpd -p 10.42.0.1"
#include "dronestation.h"

static const int RATE = 25; // hz

void fakeDrone()
{
    memset(&BB, 0, sizeof(Blackboard));
    BB.sys.state = SS_READY;

    Station station;

    while(station.connected())
    {
        station.tick();
        usleep(1000000ul/RATE);
    }

    qDebug("Disconnected");
}


#endif // FAKEDRONE_H
