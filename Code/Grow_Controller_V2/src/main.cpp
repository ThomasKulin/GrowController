#include <Arduino.h>
#include "filters.h"
#include "PID_v1.h"

#include "LED_Driver.h"

byte potPin = A5;

LED_Driver LED(A0, A1, 3);
double input, output, setpoint;

void setup()
{
  Serial.begin(19200);
  LED.init();

  pinMode(potPin, INPUT);

}

void loop()
{

  float v_out = analogRead(potPin) / 1024.0 * 24.0;
  float fb = analogRead(potPin) / 1024.0 * 5.0;
  int pwm = int(analogRead(potPin) / 1024.0 * 255);
  Serial.print(LED.getFilteredVoltage());
  Serial.print("\t");
  Serial.print(LED.getVoltage());
  Serial.print("\t");
  Serial.print(LED.getFilteredCurrent());
  Serial.print("\t");
  Serial.println(pwm);
  // LED.setVoltage(v_out);
  // LED.setFB(fb);
  LED.setPWM(45);

  // Serial.print(LED.getVoltage());
  // Serial.print("\t");
  // Serial.print(LED.getFilteredVoltage());
  // Serial.print("\t");
  // Serial.print(LED.getCurrent());
  // Serial.print("\t");
  // Serial.print(LED.getFilteredCurrent());
  // Serial.print("\t");
  // Serial.println(v_out);

}