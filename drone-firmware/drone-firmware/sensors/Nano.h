#pragma once

#include <sys/ioctl.h>
#include <cstdio>

class Nano {

public:
    Nano(int baudRate = 115200);
    
    ~Nano();
    
    bool read();
                
    void requestReadings();
    bool canRead();
    bool readData();
                
    int bytesAvailable();
    
    bool stopCommunication();
    bool restartCommunication();
    
    
    
    //getters
    unsigned long sonarCM(int i);
    float humidityPercentage();
    float temperatureC();
    uint16_t ldrADC();
    

private:

    int _baudRate;

    typedef struct {
        uint16_t sonar[4];
        int16_t humidityDHT;
        int16_t temperatureDHT;
        uint16_t ldrADC;
    } nano_readings_struct;
    
    unsigned _sonarCM[4];
    float _humidityPercentage;
    float _temperatureC;
    uint16_t _ldrADC;
    FILE *serial;
};


Nano::Nano(int baudRate) {
    _baudRate = baudRate;
    // todo remove baud rate argument or ignore it
    
    
    Util::log("Nano", "Flick serial 2", Util::LS_FLOOD);
    Serial2.begin(115200);
    Serial2.end();
    
    Util::log("Nano", "Detaching console", Util::LS_FLOOD);
    if(Serial2._detach_console())
        Util::log("Nano", "could not detach console", Util::LS_WARNING);
    
    Util::log("Nano", "Muxing", Util::LS_FLOOD);
    Serial2._enable_mux();
    
    serial = 0;
    restartCommunication();
}
    
Nano::~Nano() {
    stopCommunication();


    Serial2._disable_mux();
    
    if(Serial2._reattach_console())
        Util::log("Nano", "could not reattach console", Util::LS_WARNING);
}


void Nano::requestReadings()
{
    if(!serial)
    {
        Util::log("Nano", "requestReadings(): no serial port", Util::LS_WARNING);
        return;
    }
    
    //const char *cmd = "r\n";
    //fwrite(cmd, 3, 1, serial);
    //fflush(serial);

    // HACK
    system("echo r > /dev/ttyS1");
}


bool Nano::canRead()
{
    return bytesAvailable() >= sizeof(nano_readings_struct);
}


int Nano::bytesAvailable()
{
    if(!serial)
    {
        Util::log("Nano", "bytesAvailable(): no serial port", Util::LS_WARNING);
        return 0;
    }
    
    int ret;
    ioctl(fileno(serial), FIONREAD, &ret);
    return ret;
}


bool Nano::readData()
{
    if(!serial)
        return false;
    
    if(!canRead())
        return false;
    
    static const size_t pktSize = sizeof(nano_readings_struct);
    
    char receivedBytes[pktSize];
    
    
    if(!fread(receivedBytes, pktSize, 1, serial))
    {
        Util::log("Nano", "read data failed", Util::LS_WARNING);
        return false;
    }
    
    char *charptr = receivedBytes;
    
    void *temp = static_cast<void*>(charptr);
    
    nano_readings_struct *nanoTptr = static_cast<nano_readings_struct *>(temp);

    _sonarCM[0] = static_cast<unsigned>(nanoTptr->sonar[0]);
    _sonarCM[1] = static_cast<unsigned>(nanoTptr->sonar[1]);
    _sonarCM[2] = static_cast<unsigned>(nanoTptr->sonar[2]);
    _sonarCM[3] = static_cast<unsigned>(nanoTptr->sonar[3]);
    _humidityPercentage = static_cast<float>(nanoTptr->humidityDHT) / 100;
    _temperatureC = static_cast<float>(nanoTptr->temperatureDHT) / 100;
    _ldrADC = static_cast<unsigned>(nanoTptr->ldrADC);    
    
    return true;
}

bool Nano::read() {
    requestReadings();    
    while(!canRead());    
    readData();
}

bool Nano::stopCommunication() {
    if(serial)
    {
        fclose(serial);
        serial = 0;
    }
}

bool Nano::restartCommunication() {
    
    Util::log("Nano", "restartCommunication()", Util::LS_FLOOD);
    if(serial)
        return false;
    
    serial = fopen("/dev/ttyS1", "rw+");
    Util::log("Nano", "Opened serial device", Util::LS_INFO);
    
    if(!serial)
    {
        Util::log("Nano", "Could not open serial device ttyS1", Util::LS_WARNING);
        return false;
    }
    
//     int s = fcntl(fileno(serial), F_GETFL);
//     s |= O_SYNC; // set SYNC bit
//     fcntl(fileno(serial), F_SETFL, s);
    
    return true;
}

//get
unsigned long Nano::sonarCM(int i) {
    return _sonarCM[i];
}

float Nano::humidityPercentage() {
    return _humidityPercentage;
}

float Nano::temperatureC() {
    return _temperatureC;
}

uint16_t Nano::ldrADC() {
    return _ldrADC;
}

