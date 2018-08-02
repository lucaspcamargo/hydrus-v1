#pragma once

class PHProbe {
public:

    static const uint8_t PH_PIN = A3;

    PHProbe(){

    }

    float readPH() {
        float raw = ADC::read(PH_PIN);
        float volt = raw*5.0f;
        return (volt); 
    }

};
