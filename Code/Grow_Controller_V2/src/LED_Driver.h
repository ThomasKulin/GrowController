/*
    LED_Driver.h - Class holding parameters and functions for LED Driver boards
    Created by Thomas Kulin, November 5, 2019.
    stop copying my code you lazy shit
*/

#ifndef LED_Driver_h
#define LED_Driver_h

#include <Arduino.h>
#include "filters.h"
#include "PID_v1.h"

#define V_THRESH 2  
#define C_THRESH 0.1

class LED_Driver
{
public:
    byte cPin, vPin, fPin;

    float cutoff_freq, sampling_time;
    IIR::ORDER  order;
    Filter *f_c, *f_v;

    float P, I, D;
    double input, output, setpoint;
    PID *p_c, *p_v;

public:
    LED_Driver(byte currentPin, byte voltagePin, byte feedbackPin);
    
    void init();
    float getCurrent();
    float getVoltage();
    float getFilteredCurrent();
    float getFilteredVoltage();
    double computePID(PID *pid);
    double computePID(PID *pid, float in, float set);
    void setFB(float fbVoltage);
    void LED_Driver::setVoltage(float v_out);
};
#endif