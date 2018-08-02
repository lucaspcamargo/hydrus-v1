#pragma once

#include <Wire.h>
#include <I2Cdev.h>
#include <HMC5883L.h>
#include "../drv_i2c.h"

class Magnetometer
{

public:
	//configurar o HMC5883L no construtor
	Magnetometer() {
        // initialize devices
        I2C::lockBus();
	    mag.initialize();
        I2C::unlockBus();

        mx = my = mz = 0;
        heading = 0;
	}

	float readMagnetometer() {
		I2C::lockBus();
        mag.getHeading(&mx, &my, &mz);
        I2C::unlockBus();

	    // To calculate heading in degrees. 0 degree indicates North
	    heading = atan2(my, mx);
	    if (heading < 0) {
	    	heading += 2 * M_PI;
	    }

        float ret = heading * 180/M_PI;
        
#if DRONE_COMPASS_REVERSE
        ret = 360 - ret;
#endif
        
#ifdef DRONE_COMPASS_CALIB
        ret += (DRONE_COMPASS_CALIB);
#endif
        
        while(ret < 0)
            ret += 360.0f;
        
        while(ret > 360)
            ret -= 360.0f;
        
        return ret;
		
	}

private:
    HMC5883L mag;
    int16_t mx, my, mz;
    float heading;

};


