#ifndef MOTORS_H
#define MOTORS_H

#include "Arduino.h"
#include <Encoder.h>


class Motor
{
  public:
    Motor(uint8_t pinEN,uint8_t pinIN1, uint8_t pinIN2, uint8_t mainSpeedPin, uint8_t secondarySpeedPin); 
    void initPins();
    void commandMotor(int command);
    Encoder enc;
    
    
  private:  
        const uint8_t _pinEN;       
        const uint8_t _pinIN1;            
        const uint8_t _pinIN2;             
        const uint8_t _mainSpeedPin;       
        const uint8_t _secondarySpeedPin; 
};


#endif
