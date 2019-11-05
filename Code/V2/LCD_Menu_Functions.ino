void update_screen() {
  if (millis() - last_update > period_update) {
    last_update = millis();
    LCD_menu.update();
  }
}

void change_delta() {
  if (delta == 1)
    delta = 100;
  else if (delta == 100)
    delta = 1;
}

void save_settings() {
  EEPROMWriteInt(0, rSetCurrent);
  EEPROMWriteInt(2, bSetCurrent);
  EEPROMWriteInt(4, wSetCurrent);
  EEPROMWriteInt(6, led_on_time);
  EEPROMWriteInt(8, led_off_time);
  EEPROM.update(10, PUMP_TIME);
  EEPROM.update(11, HUM_THRESH);
  EEPROM.update(12, EXHAUST_TIME);
  EEPROM.update(13, TEMP_THRESH_HI);
}

void read_settings() {
  rSetCurrent = EEPROMReadInt(0);
  bSetCurrent = EEPROMReadInt(2);
  wSetCurrent = EEPROMReadInt(4);
  led_on_time = EEPROMReadInt(6);
  led_off_time = EEPROMReadInt(8);
  PUMP_TIME = EEPROM.read(10);
  HUM_THRESH = EEPROM.read(11);
  EXHAUST_TIME = EEPROM.read(12);
  TEMP_THRESH_HI = EEPROM.read(13);
}

void toggle_led() {
  light_on = !light_on;
}
void exhaust_toggle(){
  exhaust_on = !exhaust_on;
}
void pump_toggle(){
  pump_on = !pump_on;
}
void rSetCurrent_up() {
  rSetCurrent += delta;
}
void rSetCurrent_down() {
  rSetCurrent -= delta;
}
void bSetCurrent_up() {
  bSetCurrent += delta;
}
void bSetCurrent_down() {
  bSetCurrent -= delta;
}
void wSetCurrent_up() {
  wSetCurrent += delta;
}
void wSetCurrent_down() {
  wSetCurrent -= delta;
}

void led_on_time_up() {
  led_on_time += delta;
}
void led_on_time_down() {
  led_on_time -= delta;
}
void led_off_time_up() {
  led_off_time += delta;
}
void led_off_time_down() {
  led_off_time -= delta;
}
void pump_time_up(){
  PUMP_TIME++;
}
void pump_time_down(){
  PUMP_TIME--;
}
void hum_thresh_up(){
  HUM_THRESH++;
}
void hum_thresh_down(){
  HUM_THRESH--;
}
void exhaust_time_up(){
  EXHAUST_TIME++;
}
void exhaust_time_down(){
  EXHAUST_TIME--;
}
void temp_thresh_hi_up(){
  TEMP_THRESH_HI++;
}
void temp_thresh_hi_down(){
  TEMP_THRESH_HI--;
}

