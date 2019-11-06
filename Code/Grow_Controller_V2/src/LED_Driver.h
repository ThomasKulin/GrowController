/*
    LED_Driver.h - Class holding parameters and functions for LED Driver boards
    Created by Thomas Kulin, November 5, 2019.
    stop copying my code you lazy shit
*/

#ifndef LED_Driver_h
#define LED_Driver_h

#include <Arduino.h>

class LED_Driver
{
private:
    byte cPin, vPin, fPin;

public:
    LED_Driver(byte currentPin, byte voltagePin, byte feedbackPin);
    
    void init();
    float getCurrent();
    float getVoltage();
    void setFB(float fbVoltage);
};
#endif