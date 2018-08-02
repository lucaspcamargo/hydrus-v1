#include "drv_gps.h"
#include "threading/thread.h"
#include "nmea.h"
#include <unistd.h>

char GPS::linebuf[GPS_MAX_LINE_LENGTH];
int GPS::linecount = 0;
volatile bool GPS::_binaryMode = false;

GPS::Queue GPS::_queue;
Semaphore GPS::_sem(0);
Mutex GPS::_mtx;

NMEA::RMCData _gps_rmc;

class GPSThread: public Thread
{
  public:
    GPSThread() : Thread(-1, true) {}
  
    int main()
    {
      while (true)
      {
        GPS::service(0);
        usleep( 1000000ul / GPS::GPS_UPDATE_RATE );
      }
      return 0;
    }
};

void GPS::init()
{
  Util::log("GPS",  "Init");

  pinMode(PIN_WAKEUP, INPUT);
  pinMode(PIN_ON_OFF, OUTPUT);
  pinMode(PIN_RESET, OUTPUT);
  
  digitalWrite(PIN_RESET, HIGH);
  digitalWrite(PIN_ON_OFF, LOW);
 
  //turnOff(false);
  reset();
}

void GPS::reset()
{    
  Util::log("GPS", "RESET");
  digitalWrite(PIN_RESET, LOW);
  delay(50); // ms
  digitalWrite(PIN_RESET, HIGH);
}

void GPS::turnOn()
{
  Util::log("GPS", "ON");
  digitalWrite(PIN_ON_OFF, HIGH);
  delay(200); // ms
  digitalWrite(PIN_ON_OFF, LOW);
  
  delay(1000);
  
  Serial1.begin(4800);  
  
  //endBinaryMode();
  
  if (GPS_FAST)
  {
    Util::log("GPS", "FAST enabled (115200)");
    Serial1.print("$PSRF100,1,115200,8,1,0*05\r\n"); // from psirf reference
    Serial1.end();
    Serial1.begin(115200);
    
    if(GPS_ENABLE_5HZ)
        enable5Hz();
  }
  

  //restoreData();
  
  static GPSThread thread;
  Util::log("GPS", "Thread started");
  
}

void GPS::enable5Hz()
{
    Serial1.print("$PSRF103,00,6,00,0*23\r\n"); // from a2035 manual page 28
}

void GPS::disable5Hz()
{
    Serial1.print("$PSRF103,00,7,00,0*22\r\n"); // from a2035 manual page 28
}

void GPS::turnOff(bool save)
{
  //if(save) saveData();  
    
  Util::log("GPS", "OFF");
  digitalWrite(PIN_ON_OFF, HIGH);
  delay(200); // ms
  digitalWrite(PIN_ON_OFF, LOW);
  delay(1000); // a second for correct shutdown
  // keep high till shutdown
}

void * GPS::service(void *)
{
    
  while (Serial1.available() && !_binaryMode)
  {
    int i = Serial1.read();
    if (i)
    {
      char c = (char) i;
      linebuf[linecount] = c;

      linecount = (linecount + 1) % GPS_MAX_LINE_LENGTH;

      if (c == '\n')
        dispatchLine();
    }
  }

  return 0;
}

void GPS::tick(micros_t us)
{
  std::stringstream ss;
  switch(BB.nav.gpsState)
  {
    case SS_INITIALIZING:
    if(digitalRead(PIN_WAKEUP) || us >= 1000000u)
    {
      turnOn();
      BB.nav.gpsState = SS_READY;
      BB.sys.initializedSubSys += 1;
    }
    return; // do nothing else

    case SS_READY:
    if(BB.sys.state == SS_SHUTDOWN)
    {
      turnOff();
      BB.nav.gpsState = SS_HALTED;
      BB.sys.initializedSubSys -= 1;
    }
  }
  
  while (_sem.try_p())
  {

    _mtx.lock();

    std::string str = _queue.front(); // front when queue
    _queue.pop();

    _mtx.unlock();
    
    // TREATMENT CODE
    NMEA::Sentence stc;
    if (!NMEA::parseSentence(str, stc))
    {
      Util::log("GPS", NMEA::validateSentence(str) ? "Could not parse sentence" : "Ignoring invalid NMEA sentence", Util::LS_WARNING);
      continue;
    }
    
#if DRONE_DEBUG_SHOW_GPS
    Util::log("GPS", str.c_str(), Util::LS_INFO);
#endif
    
    if(BB.nav.simMode)
        return; // do not override fake simulated GPS, bail instead
        
    switch (stc.type)
    {
      case NMEA::ST_RMC:
        {
          
          NMEA::parseRMC(stc, _gps_rmc);
          std::stringstream ss;
          ss << "RMC - Fix: " << (_gps_rmc.valid ? "YES" : "NO");
          if (_gps_rmc.valid)
          {
            char * t = ctime((time_t*)&_gps_rmc.timestamp.tv_sec);
            t[strlen(t) - 2] = 0;
            ss << " Time: " << t;
            ss << " Lat: " << _gps_rmc.lat_deg;
            ss << " Lon: " << _gps_rmc.long_deg;
            
            BB.nav.gpsHasFix = true;
            BB.nav.gpsLon = _gps_rmc.long_deg;
            BB.nav.gpsLat = _gps_rmc.lat_deg;
          }
          else BB.nav.gpsHasFix = false;
              
          Util::log("GPS", ss.str().c_str());
        }
        break;

        // default:
        // did not recognize sentence
        // let it be
    }
  }
}

void GPS::dispatchLine()
{
  if (linecount < 3)
  {
    linecount = 0;
    return;
  }

  if (linebuf[linecount - 1] == '\n')
    if (linebuf[linecount - 1] == '\r')
      linecount -= 2;
    else
      linecount -= 1;

  linebuf[linecount] = 0; // add zero terminator
  std::string line((const char *) linebuf);
  line.erase(line.find_last_not_of(" \n\r\t") + 1); //trim
  linecount = 0; // start anew


  _mtx.lock();
  _queue.push(line);
  _mtx.unlock();

  _sem.v();

}



void GPS::sendBinaryCommand(void* payload, int size)
{
    unsigned char header[] = {0xA0, 0xA2}; 
    unsigned char footer[] = {0xB0, 0xB3};
    
    // calculate checksum
    int csum = 0;
    for(int i = 0; i < size; i++)
        csum += ((unsigned char*) payload)[i];
    
    Serial1.write(header, 2);
    
    header[0] = size >> 8;
    header[1] = size & 0xff;
    Serial1.write(header, 2);
    
    Serial1.write((const uint8_t*)payload, size);
    
    header[0] = csum >> 8;
    header[1] = csum & 0xff;
    Serial1.write(header, 2);
    Serial1.write(footer, 2);
}

void GPS::beginBinaryMode()
{
    _binaryMode = true;
    
    if(GPS_FAST)
        Serial1.print("$PSRF100,0,115200,8,1,0*04\r\n");
    else
        Serial1.print("$PSRF100,0,4800,8,1,0*0F\r\n");
        
    delay(200);
    
}

void GPS::endBinaryMode()
{
    struct {
        
        uint8_t id = 0x81;
        uint8_t mode = 0x02;
        
        uint8_t cga = 0x01;
        uint8_t cga_cs = 0x01;
        uint8_t gll = 0x00;
        uint8_t gll_cs = 0x01;
        uint8_t gsa = 0x01;
        uint8_t gsa_cs = 0x01;
        uint8_t gsv = 0x05;
        uint8_t gsv_cs = 0x01;
        uint8_t rmc = 0x01;
        uint8_t rmc_cs = 0x01;
        uint8_t vtg = 0x00;
        uint8_t vtg_cs = 0x01;
        uint8_t mss = 0x00;
        uint8_t mss_cs = 0x01;
        uint8_t epe = 0x00;
        uint8_t epe_cs = 0x01;
        uint8_t zda = 0x00;
        uint8_t zda_cs = 0x01;
        
        uint8_t un1 = 0x00;
        uint8_t un2 = 0x00;
        
        
        uint8_t br_upper = 0x00;
        uint8_t br_lower = 0x00;
        
    } command __attribute__((packed));
    
    int bitrate = GPS_FAST? 115200 : 4800;
    
    command.br_upper = bitrate >> 8;
    command.br_lower = bitrate & 0xff;
    
    sendBinaryCommand(&command, sizeof(command));    
}

int GPS::getBinaryMessageData(uint8_t messageType, void *buffer)
{
    uint8_t * out = (uint8_t *) buffer;
    
    bool readPreamble1 = false;
    bool readPreamble2 = false;
    bool readingMessage = false;
    int bytesToGo = 0;
    int messageSize = 0;
    
    bool restart = false;
    
    
    while(true)
    {
        if(!Serial1.available())
            continue; //spin
        
        uint8_t c = Serial1.read();
        
        if(readingMessage)
        {
            Util::log("GPSP", "msg");
            *out = c;
            
            bytesToGo --;
            out ++;
            
            if(bytesToGo == 0)
                return messageSize;
        }
        else if(readPreamble2)
        {
            Util::log("GPSP", "PREAMBLE2");
            
            // get payload length
            uint8_t in_s[2];
            if(!Serial1.available()) continue; //spin
            in_s[0] = Serial.read();
            if(!Serial1.available()) continue; //spin
            in_s[1] = Serial.read();
            
            bytesToGo = in_s[1] + in_s[0] << 8;
            
            std::stringstream ss;
            ss << ((int)in_s[1]) << " " << ((int)in_s[0]) << "to go " << bytesToGo;
            Util::log("GPSP", ss.str().c_str());
                    
            if(bytesToGo)
            {
                if(!Serial1.available()) continue; //spin
                int type = Serial1.read();
                
                if(type == messageType)
                {
                    Util::log("GPSP", "MSGTYPE");
                    bytesToGo--;
                    messageSize = bytesToGo;
                    readingMessage = true;
                    
                    
                }
                else
                    restart = true;
                
            }
            else
            {
                restart = true;
            }

            
        }
        else if(readPreamble1)
        {
            Util::log("GPSP", "PREAMBLE1");
            if(c == 0xA2)
                readPreamble2 = true;
            else
                restart = true;
        }else{
            if(c == 0xA0)
                readPreamble1 = true;
            else
                restart = true;
        }
            
        if(restart)
        {
            readPreamble1 = readPreamble2 = false;
            readingMessage = false;
            
            restart = false;
        }
    }
}


#include <fstream>

// ALMANAC DATA: 32 x (2 byte clock, 24 byte data, 2 byte checksum (big endian shorts)) 

void GPS::restoreData()
{
    
    Util::log("GPS", "Restoring almanac to GPS");
    
    // load data from file, if any
    std::fstream almanac;
    almanac.open("/var/hydro/almanac", std::ios_base::in | std::ios_base::binary);
    
    if(almanac.is_open())
    {
        const int NUM_SATS = 32;
        char command[1 + 28 * NUM_SATS];
        
        command[0] = 0x82; // 130 SetAlmanac
        
        almanac.read(command+1, 28 * NUM_SATS);
        
        bool good = almanac.rdstate() == std::ios::goodbit;
        
        almanac.close();
        
        if(good)
        {
            // put data on device      
            
            beginBinaryMode();
            
          
            sendBinaryCommand(command, sizeof(command));    
            
            endBinaryMode();
        }
    }
    else
        Util::log("GPS", "No almanac file exists");
}

void GPS::saveData()
{
    Util::log("GPS", "Saving almanac from GPS");
        
    const int NUM_SATS = 32;
    char block[28 * NUM_SATS];
    char buf[512];
    
    struct {
        
        uint8_t id = 0x92; // decimal 1461
        uint8_t control = 0x00;
        
    } command __attribute__((packed));
    
    // get data from device
    
    beginBinaryMode();
    
    sendBinaryCommand(&command, 2);
    Util::log("GPS", "Requesting almanac data");
    
    for(int any = 0; any < NUM_SATS; any++)
    {
        if(getBinaryMessageData(0x0E /*14*/, buf) < 29)
        {
            Util::log("GPS", "Received incomplete almanac data. Not saving.", Util::LS_ERROR);
            endBinaryMode();
            return;
        }
        
        // received data from one satellite, copying
        
        char satellite = buf[0] - 1;        
        
        Util::log("GPS", "Received satellite almanac info");
        
        memcpy(block + (28*satellite), buf + 1, 28);
    }
    
    endBinaryMode();
    
    // save data to file
    
    std::fstream almanac;
    almanac.open("/var/hydro/almanac", std::ios_base::out | std::ios_base::binary);
    
    if(almanac.is_open())
    {
        almanac.write(block, sizeof(block));
        almanac.close();
        
        Util::log("GPS", "Almanac saved successfully");
        
    }
}
