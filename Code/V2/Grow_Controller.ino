#include <Adafruit_seesaw.h>
#include <PID_v1.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_AM2320.h>
#include <RTClib.h>
#include <Wire.h>
#include <Bounce2.h>
#include <Encoder.h>
#include <EEPROM.h>
#include <LiquidMenu.h>
#include "LCD_Menu_Handler.h"
#include "Encoder_Handler.h"

#define RED_MAX_VOLTAGE 22
#define RED_MAX_CURRENT 800
#define BLUE_MAX_VOLTAGE 20.4
#define BLUE_MAX_CURRENT 1600
#define WHITE_MAX_VOLTAGE 20.4
#define WHITE_MAX_CURRENT 3200
#define DACCURACY 255 //resolution of PWM 2^8=256

#if defined(ARDUINO_ARCH_SAMD)
// for Zero, output on USB Serial console, remove line below if using programming port to program the Zero!
#define Serial SerialUSB
#endif

//LCD MENU
const byte enterPin = 31;
Bounce enterbutton = Bounce(enterPin, 10);
boolean enterPressed = false;
int delta = 10;
unsigned long pressTime = 0;
boolean prevPressed = false;

//SENSORS
Adafruit_AM2320 am2320 = Adafruit_AM2320();
unsigned long sensorTime = 0;
RTC_DS1307 rtc;
DateTime now;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
byte time_hour, time_minute, time_second;
Adafruit_seesaw ss;

//----Pin Definition----
int vInPin = A0;
int redCurrentPin = A1;// v after current shunt
int redVoltagePin = A2;// v below LED string
int redPwmPin = 2;
int blueCurrentPin = A3;
int blueVoltagePin = A4;
int bluePwmPin = 3;
int whiteCurrentPin = A5;
int whiteVoltagePin = A6;
int whitePwmPin = 5;
int fanControlPin = 7;
int pumpControlPin = 8;

//----Feedback Variables----
double inputRatio = 0, redCRatio = 0, blueCRatio = 0, whiteCRatio = 0, redVRatio = 0, blueVRatio = 0, whiteVRatio = 0; //multiplier to get actual voltage
unsigned long Rf1 = 55500; //to VCC
unsigned long Rf2 = 9860; //to GND
double redCurrent = 0, blueCurrent = 0, whiteCurrent = 0;
unsigned long redCR1 = 470000, redCR2 = 100000, redVR1 = 470000, redVR2 = 100000;
unsigned long blueCR1 = 470000, blueCR2 = 100000, blueVR1 = 470000, blueVR2 = 100000;
unsigned long whiteCR1 = 470000, whiteCR2 = 100000, whiteVR1 = 470000, whiteVR2 = 100000;

//----Light Control Variables----
double inputVoltage = 0;
double redShunt = 0, blueShunt = 0, whiteShunt = 0;
double redVoltage = 0, blueVoltage = 0, whiteVoltage = 0;
//unsigned int rSetCurrent = 0, bSetCurrent = 0, wSetCurrent = 0; moved to menu handler
unsigned int rSetCurrentPrev = 0, bSetCurrentPrev = 0, wSetCurrentPrev = 0;
unsigned int rDuty = 0, bDuty = 0, wDuty = 0;

//---filter stuff---
#define N 16
#define C 1
unsigned long rfDuty = 0, bfDuty = 0, wfDuty = 0;;
long redCurrentFiltered = 000 , blueCurrentFiltered = 4000, whiteCurrentFiltered = 6000;
double redShuntFiltered = 24, blueShuntFiltered = 24, whiteShuntFiltered = 24;
double inputVoltageFiltered = 24, redVoltageFiltered = 24, blueVoltageFiltered = 24, whiteVoltageFiltered = 24;
//---PID stuff---
double redSetpoint = 0, redInput, redOutput;
double blueSetpoint = 0, blueInput, blueOutput;
double whiteSetpoint = 0, whiteInput, whiteOutput;
double Kp = 0.001, Ki = .01, Kd = 0;
PID redPID(&redInput, &redOutput, &redSetpoint, Kp, Ki, Kd, DIRECT);
PID bluePID(&blueInput, &blueOutput, &blueSetpoint, Kp, Ki, Kd, DIRECT);
PID whitePID(&whiteInput, &whiteOutput, &whiteSetpoint, Kp, Ki, Kd, DIRECT);

unsigned long lasttime = 0;
unsigned long exhausttime = 0;

void setup() {
#ifndef ESP8266
  while (!Serial); // for Leonardo/Micro/Zero
#endif
  Serial.begin(115200);

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  //  if (! rtc.isrunning()) {
  //    Serial.println("RTC is NOT running!");
  //    // following line sets the RTC to the date & time this sketch was compiled
  //      // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  //    //    This line sets the RTC with an explicit date & time, for example to set
  //    //   January 21, 2014 at 3am you would call:
  //    //rtc.adjust(DateTime(2019, 2, 1, 0, 38, 0));
  //  }
  if (!ss.begin(0x36)) {
    Serial.println("ERROR! seesaw not found");
    while (1);
  } else {
    Serial.print("seesaw started! version: ");
    Serial.println(ss.getVersion(), HEX);
  }
  am2320.begin();
  read_settings(); //pull saved settings from EEPROM

  pinMode(enterPin, INPUT_PULLUP);
  pinMode(vInPin, INPUT);
  pinMode(redCurrentPin, INPUT);
  pinMode(redPwmPin, OUTPUT);
  pinMode(redVoltagePin, INPUT);
  pinMode(blueCurrentPin, INPUT);
  pinMode(bluePwmPin, OUTPUT);
  pinMode(blueVoltagePin, INPUT);
  pinMode(whiteCurrentPin, INPUT);
  pinMode(whitePwmPin, OUTPUT);
  pinMode(whiteVoltagePin, INPUT);
  pinMode(fanControlPin, OUTPUT);
  pinMode(pumpControlPin, OUTPUT);

  redPID.SetMode(AUTOMATIC);
  redPID.SetOutputLimits(0, 240);
  bluePID.SetMode(AUTOMATIC);
  bluePID.SetOutputLimits(0, 240);
  whitePID.SetMode(AUTOMATIC);
  whitePID.SetOutputLimits(0, 240);

  TCCR3B = TCCR3B & B11111000 | B00000001; //prescaler for Timer3 pins 2,3,5

  lcd.begin(16, 4);
  screen1_line1.attach_function(1, toggle_led);
  screen1_line1.attach_function(2, toggle_led);
  screen1_line2.attach_function(1, rSetCurrent_up);
  screen1_line2.attach_function(2, rSetCurrent_down);
  screen1_line2.attach_function(3, change_delta);
  screen1_line3.attach_function(1, bSetCurrent_up);
  screen1_line3.attach_function(2, bSetCurrent_down);
  screen1_line3.attach_function(3, change_delta);
  screen1_line4.attach_function(1, wSetCurrent_up);
  screen1_line4.attach_function(2, wSetCurrent_down);
  screen1_line4.attach_function(3, change_delta);
  screen2_line2.attach_function(1, led_on_time_up);
  screen2_line2.attach_function(2, led_on_time_down);
  screen2_line3.attach_function(1, led_off_time_up);
  screen2_line3.attach_function(2, led_off_time_down);
  screen4_line2.attach_function(1, save_settings);
  screen5_line2.attach_function(1, exhaust_toggle);
  screen5_line2.attach_function(2, exhaust_toggle);
  screen5_line3.attach_function(1, pump_toggle);
  screen5_line3.attach_function(2, pump_toggle);
  screen6_line1.attach_function(1, pump_time_up);
  screen6_line1.attach_function(2, pump_time_down);
  screen6_line2.attach_function(1, hum_thresh_up);
  screen6_line2.attach_function(2, hum_thresh_down);
  screen6_line3.attach_function(1, exhaust_time_up);
  screen6_line3.attach_function(2, exhaust_time_down);
  screen6_line4.attach_function(1, temp_thresh_hi_up);
  screen6_line4.attach_function(2, temp_thresh_hi_down);
  LCD_menu.add_screen(led_screen);
  LCD_menu.add_screen(timer_screen);
  LCD_menu.add_screen(data_screen);
  LCD_menu.add_screen(automate_screen);
  LCD_menu.add_screen(control_screen);
  LCD_menu.add_screen(save_screen);

  int32_t temp = 40000;
  controlKnob.write(temp);
}

void loop() {
  update_screen(); // update screen contents
  readInputs(); // check state of inputs
  readSensors();
  handleTime();
  handleOutputs();
  automate();

  if ((millis() - lasttime) > 15000) {
    lasttime = millis();
    //debug();
    //debugTime();
    debugEnvironment();
  }
  delay(10);
}

void HandleFeedback() {
  inputRatio = (double) (Rf1 + Rf2) / Rf2;
  inputVoltage = (double)analogRead(vInPin) / 1023 * 5 * inputRatio * 0.99885;
  inputVoltageFiltered = emad(inputVoltageFiltered, inputVoltage);

  // Handle Red feedback
  redCRatio = (double) (redCR1 + redCR2) / redCR2;
  redVRatio = (double) (redVR1 + redVR2) / redVR2;
  redShunt = (double)analogRead(redCurrentPin) / 1023 * 5 * redCRatio * 1.0109;
  redShuntFiltered = emad(redShuntFiltered, redShunt);
  redCurrent = (double)abs(inputVoltageFiltered - redShuntFiltered) * 2000;
  redCurrentFiltered = redCurrent;//ema(redCurrentFiltered, redCurrent);
  redVoltage = (double)analogRead(redVoltagePin) / 1023 * 5 * redVRatio * 0.99545;
  redVoltageFiltered = emad(redVoltageFiltered, redVoltage);

  // Handle Blue feedback
  blueCRatio = (double) (blueCR1 + blueCR2) / blueCR2;
  blueVRatio = (double) (blueVR1 + blueVR2) / blueVR2;
  blueShunt = (double)analogRead(blueCurrentPin) / 1023 * 5 * blueCRatio * 1.00166;
  blueShuntFiltered = emad(blueShuntFiltered, blueShunt);
  blueCurrent = (double)abs(inputVoltageFiltered - blueShuntFiltered) * 2000;
  blueCurrentFiltered = blueCurrent;//ema(blueCurrentFiltered, blueCurrent);
  blueVoltage = (double)analogRead(blueVoltagePin) / 1023 * 5 * blueVRatio * 0.99917;
  blueVoltageFiltered = emad(blueVoltageFiltered, blueVoltage);

  // Handle White feedback
  whiteCRatio = (double) (whiteCR1 + whiteCR2) / whiteCR2;
  whiteVRatio = (double) (whiteVR1 + whiteVR2) / whiteVR2;
  whiteShunt = (double)analogRead(whiteCurrentPin) / 1023 * 5 * whiteCRatio * 1.00501;
  whiteShuntFiltered = emad(whiteShuntFiltered, whiteShunt);
  whiteCurrent = (double)abs(inputVoltageFiltered - whiteShuntFiltered) * 2000;
  whiteCurrentFiltered = whiteCurrent;//ema(whiteCurrentFiltered, whiteCurrent);
  whiteVoltage = (double)analogRead(whiteVoltagePin) / 1023 * 5 * whiteVRatio * 0.99752;
  whiteVoltageFiltered = emad(whiteVoltageFiltered, whiteVoltage);
}

void HandleRed() {

  if ((redShuntFiltered - redVoltageFiltered) < RED_MAX_VOLTAGE) {
    if (rSetCurrent > RED_MAX_CURRENT)
      rSetCurrent = RED_MAX_CURRENT;
    redInput = redCurrentFiltered;
    redSetpoint = rSetCurrent;
    redPID.Compute();
    rDuty = redOutput;
  } else {
    rDuty--;
  }
  rfDuty = ema(rfDuty, rDuty);

  analogWrite(redPwmPin, rfDuty);
}

void HandleBlue() {

  if ((blueShuntFiltered - blueVoltageFiltered) < BLUE_MAX_VOLTAGE) {
    if (bSetCurrent > BLUE_MAX_CURRENT)
      bSetCurrent = BLUE_MAX_CURRENT;
    blueInput = blueCurrentFiltered;
    blueSetpoint = bSetCurrent;
    bluePID.Compute();
    bDuty = blueOutput;
  } else {
    bDuty--;
  }
  bfDuty = ema(bfDuty, bDuty);

  analogWrite(bluePwmPin, bfDuty);
}

void HandleWhite() {

  if ((whiteShuntFiltered - whiteVoltageFiltered) < WHITE_MAX_VOLTAGE) {
    if (wSetCurrent > WHITE_MAX_CURRENT)
      wSetCurrent = WHITE_MAX_CURRENT;
    whiteInput = whiteCurrentFiltered;
    whiteSetpoint = wSetCurrent;
    whitePID.Compute();
    wDuty = whiteOutput;
  } else {
    wDuty--;
  }
  wfDuty = ema(wfDuty, wDuty);

  analogWrite(whitePwmPin, wfDuty);
}

long ema(long average, uint16_t sample)
{
  average = (average * (C - 1) + sample) / C; // running average

  return average;
}

double emad(double average, double sample)
{
  average = (average * (N - 1) + sample) / N; // running average

  return average;
}

void readInputs() {

  focused = LCD_menu.is_focused();

  //--------------------------------
  //button state handling
  //--------------------------------

  if (enterbutton.update()) {
    if (enterbutton.fallingEdge()) {
      enterPressed = true;
      prevPressed = true;
      pressTime = millis();
    }
    else {
      enterPressed = false;
    }
  }
  if (enterPressed) {
  }

  //--------------------------------
  //button change state handling
  //--------------------------------

  if (enterPressed == false && prevPressed == true) {
    prevPressed = false;
    if ((millis() - pressTime) < 1000) { //short press
      LCD_menu.switch_focus();
    }
    else {//long press
      LCD_menu.call_function(3);
    }
  }

  //--------------------------------
  //rotary encoder handling
  //--------------------------------
  switch (updateEncoder()) {

    case 1: { //CW rotation
        if (focused == 1) { //returns 1 for true, 47 for false (no fucking clue why)
          LCD_menu.call_function(1);
        }
        else {
          LCD_menu.next_screen();
        }
        break;
      }
    case 2: { //CCW rotation
        if (focused == 1) { //returns 1 for true, 47 for false (no fucking clue why)
          LCD_menu.call_function(2);
        }
        else {
          LCD_menu.previous_screen();
        }
        break;
      }
    default: //do nothing
      break;
  }
}
void readSensors() {
  if ((millis() - sensorTime) > 2000) {
    air_temp = am2320.readTemperature();
    rel_hum = am2320.readHumidity();
    //Serial.print("Temp: "); Serial.println(am2320.readTemperature());
    //Serial.print("Hum: "); Serial.println(am2320.readHumidity());
    soil_temp = ss.getTemp();
    soil_moisture = ss.touchRead(0);
    //Serial.print("Temperature: "); Serial.print(tempC); Serial.println("*C");
    //Serial.print("Capacitive: "); Serial.println(capread);
    sensorTime = millis();
  }
}
void handleTime() {
  now = rtc.now();
  time_hour = now.hour();
  time_minute = now.minute();
  time_second = now.second();

  time_now_int = time_hour * 100 + time_minute;
}
void handleOutputs() {
  if (pump_on || pump_request)
    digitalWrite(pumpControlPin, HIGH);
  else
    digitalWrite(pumpControlPin, LOW);

  if (exhaust_on || exhaust_request)
    digitalWrite(fanControlPin, HIGH);
  else
    digitalWrite(fanControlPin, LOW);
}

void automate() {
  byte led_on_hour = led_on_time / 100;
  byte led_on_minute = led_on_time % 100;
  byte led_off_hour = led_off_time / 100;
  byte led_off_minute = led_off_time % 100;
  if (time_hour == led_on_hour && time_minute == led_on_minute) {
    light_on = true;
  }
  if (time_hour == led_off_hour && time_minute == led_off_minute) {
    light_on = false;
  }
  if (light_on) {
    redPID.SetMode(AUTOMATIC);
    bluePID.SetMode(AUTOMATIC);
    whitePID.SetMode(AUTOMATIC);
    HandleFeedback();
    HandleRed();
    HandleBlue();
    HandleWhite();
  }
  else {
    analogWrite(redPwmPin, 0);
    analogWrite(bluePwmPin, 0);
    analogWrite(whitePwmPin, 0);
    redPID.SetMode(MANUAL);
    bluePID.SetMode(MANUAL);
    whitePID.SetMode(MANUAL);
    rDuty = 0;
    bDuty = 0;
    wDuty = 0;
    rfDuty = 0;
    bfDuty = 0;
    wfDuty = 0;
  }

  if (air_temp > TEMP_THRESH_HI || (time_minute == 0 && time_second == 0)) {
    exhausttime = millis();
    exhaust_request = true;
  }
  if ((millis() - exhausttime) > (EXHAUST_TIME * 1000 * 60)) {
    exhaust_request = false;
  }

  if (time_minute == 0 && time_second < 3 && rel_hum < HUM_THRESH) { //check every hour
    pump_request = true; //time in seconds to pump for humidity
  }
  if (time_minute == PUMP_TIME) { //pump for PUMP_TIME minutes
    pump_request = false;
  }
}

void EEPROMWriteInt(int address, int value) {
  byte two = (value & 0xFF);
  byte one = ((value >> 8) & 0xFF);

  EEPROM.update(address, two);
  EEPROM.update(address + 1, one);
}

int EEPROMReadInt(int address) {
  long two = EEPROM.read(address);
  long one = EEPROM.read(address + 1);

  return ((two << 0) & 0xFFFFFF) + ((one << 8) & 0xFFFFFFFF);
}

void debug() {
  Serial.print("Input Voltage: ");
  Serial.print(inputVoltageFiltered);

  //  Serial.print("   Red V+");
  //  Serial.print(redShuntFiltered);
  //  Serial.print("   Red V-");
  //  Serial.print(redVoltageFiltered);
  Serial.print(" Red C: ");
  Serial.print(redCurrentFiltered);
  Serial.print(" Red C Set: ");
  Serial.print(rSetCurrent);
  Serial.print(" Red V: ");
  Serial.print(redShuntFiltered - redVoltageFiltered);
  Serial.print(" Red Duty: ");
  Serial.print(rDuty);

  //  Serial.print(" Blue V+");
  //  Serial.print(blueShunt);
  //  Serial.print(" Blue V-");
  //  Serial.print(blueVoltage);
  Serial.print(" Blue C: ");
  Serial.print(blueCurrentFiltered);
  Serial.print("   Blue C Set: ");
  Serial.print(bSetCurrent);
  Serial.print(" Blue V: ");
  Serial.print(blueShuntFiltered - blueVoltageFiltered);
  Serial.print(" Blue Duty: ");
  Serial.print(bDuty);

  //  Serial.print("   White V+");
  //  Serial.print(whiteShunt);
  //  Serial.print("   White V-");
  //  Serial.print(whiteVoltage);
  Serial.print(" White C: ");
  Serial.print(whiteCurrentFiltered);
  Serial.print(" White C Set: ");
  Serial.print(wSetCurrent);
  Serial.print(" White V: ");
  Serial.print(whiteShuntFiltered - whiteVoltageFiltered);
  Serial.print(" White Duty: ");
  Serial.println(wDuty);
}
void debugTime() {
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" (");
  Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
  Serial.print(") ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();
}
void debugEnvironment() {
  Serial.print(air_temp);
  Serial.print("  ");
  Serial.print(soil_temp);
  Serial.print("  ");
  Serial.print(rel_hum);
  Serial.println("  ");
  //Serial.println(soil_moisture);
}

