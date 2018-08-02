#pragma once

#include <string>
#include <cstdlib>
#include <sys/time.h>
#include <ctime>

#ifdef HYDRUS_NMEA_DEBUG
#include <cstdio>
#endif

namespace NMEA
{
  using namespace std;

  enum SentenceType
  {
    ST_UNDEFINED,
    ST_RMC, // Recommended minimum specific GPS/Transit data 
    ST_GGA, // Global Positioning System Fix Data
    ST_GSV, // GPS satellites in view
    ST_GSA, // GPS DOP and active satellites
    ST_OTHER  // Something I have no idea how to handle
  };

  struct Sentence
  {
    SentenceType type;
    stringvec_t tokens;
  };

  struct RMCData
  {
    timeval timestamp;
    bool valid;
    float lat_deg;
    float long_deg;
    float speed_m_s;
    float course_rad;    
  };

  static const float KNOTS_TO_M_S = 0.51444444444f;
  
  bool validateSentence(string line)
  {
    int len = line.length();
        
    if(len < 9)
      return false;

    if(line[0] != '$')
      return false;

    // need to trim before uncommenting this
    //if(line[len - 1 - 2] != '*')
      //return false;

    if(line.find('$', 1) != string::npos) // is the string two sentences mushed together?
      return false;

    // verify checksum
    
    return true;
  }

  SentenceType getSentenceType(string slice)
  {
    if(slice == "GPRMC")
      return ST_RMC;

    return ST_OTHER;
  }

  bool parseSentence(string sentence, Sentence &ret, bool validate = true)
  {
    if(validate && !validateSentence(sentence))
      return false;
    
    int commaPos = sentence.find(',', 0);
    if(commaPos == string::npos) // needs a first comma
      return false;
      
    string firstSlice = sentence.substr(1, commaPos-1);
    ret.type = getSentenceType(firstSlice);

    if(ret.type == ST_UNDEFINED)
      return false;

    Util::split(sentence.substr(commaPos+1, sentence.length()-3), ',', ret.tokens);

    return true;
  }

  timeval parseTime( string hms, string dmy = "" )
  {
      
#ifdef HYDRUS_NMEA_DEBUG
    fprintf(stderr, "Parsing time %s --- %s\n", hms.c_str(), dmy.c_str());
#endif
    
    timeval ret;
    tm tmd;

    if(hms.size() < 6)
    {
        memset(&ret, 0, sizeof(timeval));
        return ret;
    }
    
    if(dmy.size() >= 6)
    {
        tmd.tm_year = Util::parseInt(dmy, 4, 6);
        tmd.tm_mon  = Util::parseInt(dmy, 2, 4);
        tmd.tm_mday = Util::parseInt(dmy, 0, 2);
    }
    else
        tmd.tm_year = tmd.tm_mon = tmd.tm_mday = 0;
    
    tmd.tm_hour = Util::parseInt(hms, 0, 2);
    tmd.tm_min =  Util::parseInt(hms, 2, 4);
    tmd.tm_sec =  Util::parseInt(hms, 4, 6);
    
    ret.tv_sec = mktime(&tmd);
    
    ret.tv_usec = 1000000*Util::parseDouble(hms, 6, hms.length());

    fprintf(stderr, "Parsed time\n");
    
    return ret;
  }

  float parseLatLongAngle( string ang )
  {
#ifdef HYDRUS_NMEA_DEBUG
    fprintf(stderr, "Parsing angle %s\n", ang.c_str());
#endif
    int dotPos = ang.find('.', 0);
    if(dotPos == string::npos) // needs a first comma
      return -999.99f;
    
    int minPos = dotPos-2;
    
    float deg = Util::parseInt(ang, 0, minPos);
    deg += Util::parseDouble(ang, minPos)/60.0f;
    
#ifdef HYDRUS_NMEA_DEBUG
    printf("parsed angle %f\n", deg);
#endif
    
    return deg;
    
  }

/**
 * $GPRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmyy,x.x,a,m*hh
 * Field #
 * 1    = UTC time of fix
 * 2    = Data status (A=Valid position, V=navigation receiver warning)
 * 3    = Latitude of fix
 * 4    = N or S of latitude
 * 5    = Longitude of fix
 * 6    = E or W of longitude
 * 7    = Speed over ground in knots
 * 8    = Track made good in degrees True
 * 9    = UTC date of fix
 * 10   = Magnetic variation degrees (Easterly var. subtracts from true course)
 * 11   = E or W of magnetic variation
 * 12   = Mode indicator, (A=Autonomous, D=Differential, E=Estimated, N=Data not valid) OPTIONAL, STANDARD IN 3.0
 * 13   = Checksum
 * 
 * WARNING: needs to be a valid sentence
 * 
 */ 
  bool parseRMC( const Sentence & stc, RMCData & data )
  {
    if(stc.type != ST_RMC)
      return false;

    // now parse stuff
    data.timestamp = parseTime(stc.tokens[0], stc.tokens[8]);
    data.valid = stc.tokens[1][0] == 'A';
    if(data.valid)
    {
      data.lat_deg = parseLatLongAngle(stc.tokens[2]);
      if(stc.tokens[3] == "S") data.lat_deg *= -1;
      
      data.long_deg = parseLatLongAngle(stc.tokens[4]);
      if(stc.tokens[5] == "W") data.long_deg *= -1;

      data.speed_m_s = Util::parseFloat(stc.tokens[6]) * KNOTS_TO_M_S;
    }
    return true;

  }
  
}

