#pragma once

extern void analogWriteResolution(int res);

class Motors
{
  public:
      
  static const int PIN_L = 5;
  static const int PIN_R = 6;

  static const bool HIGHRES_ENABLED = false;
  static const int P_MULT = HIGHRES_ENABLED? 16 : 1;
  
  // experimental values
  static const int PERIOD_MAX = 255 * P_MULT;
  static const int MIN_PERIOD = 6 * P_MULT;
  static const int MAX_PERIOD = 250 * P_MULT;
  static const int MIN_DEADBAND = 0 * P_MULT;

  static const bool SIM_MODE_DISABLES_MOTORS = true;
  
  static const float DRONE_MOTOR_FACTOR_L = 0.6;
  static const float DRONE_MOTOR_FACTOR_R = 1.0;
  
  Motors(){}

  void init()
  {
    pinMode(PIN_L, OUTPUT);
    pinMode(PIN_R, OUTPUT);
    
    I2C::lockBus(); // assume controller is on the bus
    
    if(HIGHRES_ENABLED)
        analogWriteResolution(12);
    
    analogWrite(PIN_L, 0); // no signal
    analogWrite(PIN_R, 0);
    I2C::unlockBus();
  }

  int calculateCycle(float alpha)
  {
    return map(alpha*10000, 0, 10000, MIN_PERIOD + MIN_DEADBAND, MAX_PERIOD);
  }

  void tick()
  {

    int pwmL = MIN_PERIOD,
        pwmR = MIN_PERIOD;
        
    if(BB.nav.motor.enabled || (SIM_MODE_DISABLES_MOTORS && BB.nav.simMode))
    {
      pwmL = calculateCycle(BB.nav.motor.spdL * DRONE_MOTOR_FACTOR_L);
      pwmR = calculateCycle(BB.nav.motor.spdR * DRONE_MOTOR_FACTOR_R);
    }
    
    I2C::lockBus(); // assume controller is on the bus
    analogWrite(PIN_L, pwmL); // update speeds
    analogWrite(PIN_R, pwmR);
    I2C::unlockBus();
  }
  
};

