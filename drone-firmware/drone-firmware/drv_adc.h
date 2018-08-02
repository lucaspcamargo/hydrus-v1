#pragma once

#include "drv_i2c.h"

class ADC { 

public:
    static const bool enable_12bit = true;

    static void init()
    {
        I2C::lockBus();
        
        if(enable_12bit)
            analogReadResolution(12);
        
        pinMode(A0, INPUT);
        pinMode(A1, INPUT);
        pinMode(A2, INPUT);
        pinMode(A3, INPUT);
        
        I2C::unlockBus();
    }
    
    static int readRaw(int port)
    {
        //I2C::lockBus();
        int reading = analogRead(port);
        //I2C::unlockBus();
        
        return reading;
    }
    
    static float read(int port)
    {
        return readRaw(port)/(enable_12bit? 4095.0f : 1023.0f);
    }
};