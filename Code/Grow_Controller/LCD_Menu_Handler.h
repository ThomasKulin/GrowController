#include <EEPROM.h>
//lcd pins
const int rs = 35, en = 33, d4 = 37, d5 =39, d6 = 41, d7 = 43;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

//Set LCD refresh rate
unsigned long last_update = millis();
unsigned short period_update = 250; //screen update period in ms

//variables
long ENC_val = 0;
byte focused = 0;
unsigned int rSetCurrent = 400, bSetCurrent = 1600, wSetCurrent = 2400;
unsigned int led_on_time = 530, led_off_time = 2330;
unsigned int air_temp, soil_temp, rel_hum, soil_moisture;
int time_now_int = 0000;

//automation
bool light_on = false, pump_on = false, exhaust_on = false;
bool pump_request = false, exhaust_request = false;
byte PUMP_TIME = 48;
byte EXHAUST_TIME = 5;
byte HUM_THRESH = 22;
byte TEMP_THRESH_HI = 29;


// ----- LED SCREEN -----
LiquidLine screen1_line1(1, 0, "LED Power: ", light_on);
LiquidLine screen1_line2(1, 1, "Red: ", rSetCurrent);
LiquidLine screen1_line3(5, 2, "Blue: ", bSetCurrent);
LiquidLine screen1_line4(5, 3, "White: ", wSetCurrent);
LiquidScreen led_screen(screen1_line1, screen1_line2, screen1_line3, screen1_line4);
// --------------------------

// ----- TIMER SCREEN -----
LiquidLine screen2_line1(1, 0, "Time: ", time_now_int);
LiquidLine screen2_line2(1, 1, "Light On: ", led_on_time); // 4 digit time ie 2135 for 21:35
LiquidLine screen2_line3(5, 2, "Light Off: ", led_off_time);
LiquidScreen timer_screen(screen2_line1, screen2_line2, screen2_line3);
// --------------------------

// ----- CONTROL SCREEN -----
LiquidLine screen5_line1(1, 0, "Controls: ");
LiquidLine screen5_line2(1, 1, "Exhaust On: ", exhaust_on); 
LiquidLine screen5_line3(5, 2, "Pump On: ", pump_on);
LiquidScreen control_screen(screen5_line1, screen5_line2, screen5_line3);
// --------------------------

// ----- AUTOMATE SCREEN -----
LiquidLine screen6_line1(1, 0, "Pump Time: ", PUMP_TIME);
LiquidLine screen6_line2(1, 1, "Humidity Thresh: ", HUM_THRESH); 
LiquidLine screen6_line3(5, 2, "Exhaust Time: ", EXHAUST_TIME);
LiquidLine screen6_line4(5, 3, "Temp Thresh Hi: ", TEMP_THRESH_HI);
LiquidScreen automate_screen(screen6_line1, screen6_line2, screen6_line3, screen6_line4);
// --------------------------

// ----- DATA SCREEN -----
LiquidLine screen3_line1(1, 0, "Air Temp: ", air_temp);
LiquidLine screen3_line2(1, 1, "Soil Temp: ", soil_temp);
LiquidLine screen3_line3(5, 2, "Humidity: ", rel_hum);
LiquidLine screen3_line4(5, 3, "Soil Moisture: ", soil_moisture);
LiquidScreen data_screen(screen3_line1, screen3_line2, screen3_line3, screen3_line4);
// --------------------------

// ----- SAVE SCREEN -----
LiquidLine screen4_line1(1, 0, "Save Settings ");
LiquidLine screen4_line2(1, 1, "Scroll Fwd: ");
LiquidScreen save_screen(screen4_line1, screen4_line2);
// --------------------------

LiquidMenu LCD_menu(lcd);
