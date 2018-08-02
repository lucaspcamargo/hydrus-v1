#ifdef HYDRUS_DRONE_TEST

#define HYDRUS_NMEA_DEBUG

#include <cstdio>

#include "../util_strfunc.h"
#include "../nmea.h"


int main(int argc, char **argv)
{
    using namespace std;
    
    string s = "$GPRMC,225446,A,4916.45,N,12311.12,W,000.5,054.7,191194,020.3,E*68\r\n";
    NMEA::Sentence stc;

    bool ret = NMEA::parseSentence(s, stc);
    
    if(!ret)
    {
        printf("Could not parse $GGPRMC\n");
        exit(1);
    }
    
    if(stc.type != NMEA::ST_RMC)
    {
        printf("Could not identify $GPRMC\n");
        exit(1);
    }
    
    printf("Parsing $GPRMC\n");
    NMEA::RMCData data;
    NMEA::parseRMC(stc, data);
    
    timeval timestamp;
    bool valid;
    float lat_deg;
    float long_deg;
    float speed_m_s;
    float course_rad;   
    
    time_t nowtime;
    struct tm *nowtm;
    char tmbuf[64];

    nowtime = data.timestamp.tv_sec;
    nowtm = localtime(&nowtime);
    strftime(tmbuf, sizeof tmbuf, "%Y-%m-%d %H:%M:%S", nowtm);
    
    printf("GPRMC IS %s\n", data.valid? "VALID": "NOT VALID");
    printf("TIME IS %s\n", tmbuf);
    printf("LATITUDE IS %f\n", data.lat_deg);
    printf("LONGITUDE IS %f\n", data.long_deg);
    printf("SPEED IS %f\n", data.speed_m_s);
    printf("COURSE IS %f\n", data.course_rad);
    
    printf("Test successful.\n");
    return 0;
}

#endif