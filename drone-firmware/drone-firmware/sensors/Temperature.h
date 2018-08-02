#pragma once

class Temperature {
public:

    static const uint8_t TMP36PIN = A2;

    Temperature(){

    }

    float readTemperature() {
        float raw = ADC::read(TMP36PIN);
        float volt = raw*5.0f;
        return (volt - 0.5)/0.010f; // 10 mV per degree celsius
    }

};
