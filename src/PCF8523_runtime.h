void IRAM_ATTR wakeUpDetectedRTC() {
  // Serial.println("Wake-up event from the RTC detected!");
  // Add your code to handle the wake-up event here
  EXT_RTC_INT = true;
}

void printTime(DateTime now) {
    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print('T');
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();
}

unsigned long unix(DateTime now) {
  unsigned long unixTime = (now.year() - 1970) * 365 * 24 * 3600 +
                            (now.month() - 1) * 30 * 24 * 3600 +
                            (now.day() - 1) * 24 * 3600 +
                            now.hour() * 3600 +
                            now.minute() * 60 +
                            now.second();
  return unixTime;
}