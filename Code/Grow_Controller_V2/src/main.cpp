#include <Arduino.h>
#include "LED_Driver.h"

LED_Driver LED(A0, A1, 3);
void setup()
{
  Serial.begin(19200);
  LED.init();
}

void loop()
{
//   Serial.print("current: ");
//   Serial.print(LED.getCurrent());
//   Serial.print("\tvoltage: ");
//   Serial.println(LED.getVoltage());

  // LED.setFB(3.0);
  LED.init();
}