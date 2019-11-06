#include <Arduino.h>
#include "LED_Driver.h"

LED_Driver::LED_Driver(byte currentPin, byte voltagePin, byte feedbackPin){
    byte cPin = currentPin;
    byte vPin = voltagePin;
    byte fpin = feedbackPin;
}
void LED_Driver::init()
{
    pinMode(cPin, INPUT);
    pinMode(vPin, INPUT);
    pinMode(fPin, OUTPUT);
    Serial.println("SET PINS");
    // digitalWrite(fPin, HIGH); //set pin high at turn on to set output voltage to zero
    return;
}
float LED_Driver::getCurrent()
{ //returns the voltage at current pin
    float current = analogRead(cPin) / 1024.0 * 5.0;
    return current;
}
float LED_Driver::getVoltage()
{ //returns the voltage at voltage pin
    float voltage = analogRead(vPin) / 1024.0 * 5.0;
    float v_out = voltage * 6; // voltage divider outputs 1/6 input voltage
    return v_out;
}
void LED_Driver::setFB(float fbVoltage)
{
    int pwm = int(fbVoltage / 5.0 * 255); //8 bit DAC
    analogWrite(fPin, pwm);
    return;
}
