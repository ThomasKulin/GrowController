#include <Arduino.h>
#include "LED_Driver.h"
#include "filters.h"
#include "PID_v1.h"

LED_Driver::LED_Driver(byte currentPin, byte voltagePin, byte feedbackPin)
{
    LED_Driver::cPin = currentPin;
    LED_Driver::vPin = voltagePin;
    LED_Driver::fPin = feedbackPin;

    //default Low-Pass filter parameters
    LED_Driver::cutoff_freq = 1;
    LED_Driver::sampling_time = 0.005;
    LED_Driver::order = IIR::ORDER::OD1;
    LED_Driver::f_c = new Filter(cutoff_freq, sampling_time, order); //current filter
    LED_Driver::f_v = new Filter(cutoff_freq, sampling_time, order); //voltage filter

    //default PID parameters
    // operates on raw voltage readings (very responsive, slight jitter)
    // LED_Driver::P = 0.037;
    // LED_Driver::I = 0.425;
    // LED_Driver::D = 0.0;
    //operates on filtered voltage (slower response but very stable)
    LED_Driver::P = 0.037;
    LED_Driver::I = 0.0425;
    LED_Driver::D = 0.001;
    LED_Driver::p_c = new PID(&input, &output, &setpoint, P, I, D, REVERSE);
    LED_Driver::p_v = new PID(&input, &output, &setpoint, P, I, D, REVERSE);

}

void LED_Driver::init()
{
    pinMode(cPin, INPUT);
    pinMode(vPin, INPUT);
    pinMode(fPin, OUTPUT);
    digitalWrite(fPin, HIGH); //set pin high at turn on to set output voltage to zero

    //set PID parameters
    p_c->SetOutputLimits(0, 5.0);
    p_v->SetOutputLimits(0, 5.0);
    p_c->SetMode(AUTOMATIC);
    p_v->SetMode(AUTOMATIC);

    //Set starting values for filtering. Prevents full send condition on startup
    for(int i =0;i<300;i++){
        computePID(p_v, 24.0, 1.0);
        f_v->filterIn(24.0);
    }
}
float LED_Driver::getCurrent()
{ //returns the voltage at current pin
    float Rs = 0.005;
    float As = 10;
    float current = analogRead(cPin) / 1024.0 * 5.0;
    float i_out = current / (2 * Rs * As) * 0.76;
    return i_out;
}
float LED_Driver::getVoltage()
{ //returns the voltage at voltage pin
    float voltage = analogRead(vPin) / 1024.0 * 5.0;
    float v_out = voltage * 6; // voltage divider outputs 1/6 input voltage
    return v_out;
}
float LED_Driver::getFilteredCurrent()
{ //returns the digitally filtered voltage at current pin
    return f_c->filterIn(getCurrent());
}
float LED_Driver::getFilteredVoltage()
{ //returns the digitally filtered voltage at voltage pin
    return f_v->filterIn(getVoltage());
}
double LED_Driver::computePID(PID *pid){
    pid->Compute();
    return output;
}
double LED_Driver::computePID(PID *pid, float in, float set){
    input = in;
    setpoint = set;
    return computePID(pid);
}

void LED_Driver::setFB(float fbVoltage)
{
    int pwm = int(fbVoltage / 5.0 * 255); //8 bit DAC
    analogWrite(fPin, pwm);
    return;
}
void LED_Driver::setPWM(int pwm)
{
    analogWrite(fPin, pwm);
    return;
}
void LED_Driver::setVoltage(float v_out)
{
    // //         Vout
    // //         R2
    // //         |
    // // ARD R1--+--- FB_PIN
    // //         |
    // //         R3
    // //         GND
    // float R1 = 17000.0;
    // float R2 = 178000.0;
    // float R3 = 10000.0;
    // // v_out = (0.8*(R2*R3 + R1*R3 + R1*R2) - v_fb*R2*R3)/(R1*R3);
    // float v_fb = -1.0 * (v_out * R1 * R3 - 0.8 * (R2 * R3 + R1 * R3 + R1 * R2)) / (R2 * R3);
    
    double v_fb = computePID(p_v, getFilteredVoltage(), v_out);
    setFB(float(v_fb));
}
