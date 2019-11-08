#include <Arduino.h>
#include "filters.h"
#include "PID_v1.h"

#include "LED_Driver.h"

byte potPin = A5;

LED_Driver LED(A0, A1, 3);
double input, output, setpoint;
// PID pid(&input, &output, &setpoint, 0.0037, 0.425, 0, REVERSE); // operates on raw voltage readings (very responsive, slight jitter)
PID pid(&input, &output, &setpoint, 0.037, 0.225, 0, REVERSE); //operates on filtered voltage (slower response but very stable)

void setup()
{
  Serial.begin(19200);
  LED.init();

  pid.SetOutputLimits(0, 5.0);
  pid.SetMode(AUTOMATIC);
  // for(int i =0;i<10;i++){
  //   input = 24.0;
  //   setpoint = 15.0;
  //   pid.Compute();
  // }

  pinMode(potPin, INPUT);
}

void loop()
{
  float v_out = analogRead(potPin) / 1024.0 * 5.0;
  LED.setFB(v_out);


  // float v_out = analogRead(potPin) / 1024.0 * 24.0;
  // float currentVoltage = LED.getFilteredVoltage();
  // input = currentVoltage;
  // setpoint = v_out;
  // pid.Compute();
  // float fb = output;
  // Serial.print(setpoint);
  // Serial.print("\t");
  // Serial.print(currentVoltage);
  // Serial.print("\t");
  // Serial.print(LED.getVoltage());
  // Serial.print("\t");
  // Serial.println(fb);
  // LED.setFB(fb);

  // Serial.print(LED.getVoltage());
  // Serial.print("\t");
  // Serial.print(LED.getFilteredVoltage());
  // Serial.print("\t");
  // Serial.print(LED.getCurrent());
  // Serial.print("\t");
  // Serial.print(LED.getFilteredCurrent());
  // Serial.print("\t");
  // Serial.println(v_out);

  // if(millis() - starttime > 10000){v_fb = 1.5;}
  // LED.setFB(v_fb);
  // Serial.println(v_out);
  // Serial.print("\t");
  // Serial.print(LED.getFilteredVoltage());
  // Serial.print("\n");

}