#pragma once

class Turbidity {

public:
    
    static const uint8_t TSD10PIN = A1;

    Turbidity() {

    }

    float readTurbidity() {
        float volts = ADC::read(TSD10PIN) * 5.0f;
        return volts;
    }
    
private:

};
