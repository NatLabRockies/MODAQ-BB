
TaskHandle_t PWRMon;
TaskHandle_t PWRCons;

int INA219_Hz = 2500;
char PWRdata[128];
extern uint32_t bootTime;

void INA219_worker(void * parameter) {
  int ms;
  // DateTime inaTime;
  
  busvoltage = 0;
  current_mA = 0;
  power_mW = 0;
  int uptime = 0;
  int i = 0;
  bool success;

  while (!tasksComplete) {
    ms = millis();
    success = false;
    while(!success) {
      if (xSemaphoreTake( i2cSemaphore, (TickType_t) 10) == pdTRUE) {
        // inaTime = rtc.now();
        // int MS = millis() % 1000;
        double uptime = (rtc.now().unixtime() - bootTime) / 3600.0; //Note: vulnerable to Y2038 problem

        char dateBuffer[32] = "MM/DD/YYYY";
        char timeBuffer[32] = "hh:mm:ss";
        rtc.now().toString(dateBuffer);
        rtc.now().toString(timeBuffer);

        //uptime = int(millis()/1000); //store uptime 
        busvoltage = ina219.getBusVoltage_V();
        current_mA = ina219.getCurrent_mA();
        power_mW = ina219.getPower_mW();
        xSemaphoreGive(i2cSemaphore);

        // int t = inaTime.unixtime();

        #ifdef DEBUG_PWR
          Serial.println("PWR Data Updated:");
          Serial.print(" - Bus Voltage (V): ");
          Serial.print(busvoltage);
          Serial.print(" - Current (mA): ");
          Serial.print(current_mA);
          Serial.print(" - Power (mW): ");
          Serial.print(power_mW);
          Serial.print(" - Loop Delay: ");
          Serial.print(INA219_Hz - (millis()-ms));
          Serial.print(" - Uptime (s): ");
          Serial.println(uptime);
          appendFile(SD, logFile, "Power Data Logged");
        #endif

        sprintf(pwrBuffer, "%s,%s,%4.2f,%4.2f,%4.2f,%.2f",dateBuffer, timeBuffer, busvoltage, current_mA, power_mW, uptime);
        
        appendFile(SD, pwrFile, pwrBuffer);

        success = true;

        vTaskDelay(INA219_Hz);
      } else {
        vTaskDelay(50);
        #ifdef DEBUG_PWR
          Serial.println("stuck outer");
          appendFile(SD, logFile, "INA219 stuck outer");
        #endif
      } 
    }
  }

  pwrComplete = true;
  vTaskDelete(PWRMon);
}

void Task_INA219_Worker() {
  xTaskCreatePinnedToCore(
    INA219_worker, /* Function to implement the task */
    "INA219Worker", /* Name of the task */
    20000, /* Stack size in words */
    NULL, /* Task input parameter */
    1, /* Priority of the task */
    &PWRMon, /* Task handle. */
    1
  ); 
}

