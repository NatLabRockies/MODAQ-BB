#include <TinyGPS++.h>


TaskHandle_t GPSWorker;
TaskHandle_t GPSMon;


/*
// GPS Definitions
#define N2222                 4
#define RX2                   27
#define TX2                   14
#define GPSBaud               9600

*/

int WAIT_SECS = 60000; //ms for GPS to search for IDEAL fix. Defaults to TIME_TO_SLEEP - 5 seconds if less than TIME_TO_SLEEP

// Module Definitions
TinyGPSPlus gps;
TinyGPSCustom FixType(gps, "GPGGA", 6); // grabs the fix type from $GPGGA, where 0 = no fix, 1 for GPS, 2 for DGPS
TinyGPSCustom numSats(gps, "GPGGA", 7); // grabs the number of satellites from $GPGGA

TinyGPSDate d;
TinyGPSTime t;

// HardwareSerial ss2(2);  // Create serial object for GPS on remapped UART2

/*
  Global Definitions
*/
//int gpsMaxAcqTime = 5; // number of seconds to allow for GPS fix on wakeup (Not used)
int ttffThresh = 240; // if TTFF is > this value, GPS will not be powered off during next sleep cycle
int fixCount = 0;
int numFixes = 25;
int wakeCycles = 0;
int exeStart;
int TTFFix; 
int sats = 0;
int idealSatCount = 5;
int rtcUpdateInterval = 24;
int lastUpdate = 0;


bool GPSdata = false;
bool fixOK = false;
bool firstGoodFix = true;

String numSatsV;
String FixTypeV;
String hold;


/*
    Function Definitions
*/


void GPSWorkerFunc(void * parameter) {
  // GPS Worker
  
  unsigned long loopStart = millis();
  gps = TinyGPSPlus();
  while (true) {

    uint32_t notificationValue;
    static int ms = 500;
    unsigned long start = millis();

    if (xSemaphoreTake( serialSemaphore, (TickType_t) 10) == pdTRUE) {
      do
      {
        while (GPS.available()){
          gps.encode(GPS.read());
        } 
      }
      while (millis() - start < ms);
      xSemaphoreGive(serialSemaphore);

      #ifdef DEBUG_GPS
        Serial.print("GPS Worker Searching for Fix - ");
        Serial.print("Location Age: ");
        Serial.print(gps.location.age());
        Serial.print(" | Number of Satellites: ");
        Serial.print(gps.satellites.value());
        Serial.print(" | sentenceWithFix: ");
        Serial.print(gps.sentencesWithFix());
        Serial.print(" | Time in Loop: ");
        Serial.print(( millis() - loopStart) / 1000);
        Serial.println(" s");
        appendFile(SD, logFile, "GPS Worker Searching for Fix");
        
        UBaseType_t freeHeap = uxTaskGetStackHighWaterMark(NULL);
        Serial.print("GPS Worker Free Heap: ");
        Serial.println(freeHeap);
        char buffer[64];
        snprintf(buffer, sizeof(buffer), "GPS Worker Free Heap: %i", freeHeap);
        appendFile(SD, logFile, buffer);

      #endif
      
      if (gps.location.isUpdated(),
          gps.satellites.value() > 5) {
          
        xTaskNotify(GPSMon, 0, eSetValueWithOverwrite);
        vTaskDelay(10);
        if (xTaskNotifyWait(0, 0, &notificationValue, pdMS_TO_TICKS(60000))) {
          vTaskDelete(GPSWorker);
        }
      } 
      else if (millis() - loopStart > WAIT_SECS) {
        if (gps.location.isUpdated() &&
                  gps.satellites.value() > 0) {
          xTaskNotify(GPSMon, 1, eSetValueWithOverwrite);
          vTaskDelay(10);
          if (xTaskNotifyWait(0, 0, &notificationValue, pdMS_TO_TICKS(60000))) {
            vTaskDelete(GPSWorker);
          }
        }  
        else {
          xTaskNotify(GPSMon, -1, eSetValueWithOverwrite);
          vTaskDelay(10);
          if (xTaskNotifyWait(0, 0, &notificationValue, pdMS_TO_TICKS(60000))) {
            vTaskDelete(GPSWorker);
          }
        }
      }
    }
  vTaskDelay(50);
  }
}

/*
    GPS Worker Call Task Handle
*/

void Task_GPSWorker() {

  // if app produces kernal panics, increase the stack size in the xTaskCreate function
  xTaskCreatePinnedToCore(
    GPSWorkerFunc, /* Function to implement the task */
    "GPSWorker", /* Name of the task */
    20000, /* Stack size in words */
    NULL, /* Task input parameter */
    1, /* Priority of the task */
    &GPSWorker, /* Task handle. */
    1); /* Core where the task should run */
}

void GPSMonitorFunc(void * parameter) {


  GPS.sendCommand("");
  vTaskDelay(250);
  
  Task_GPSWorker();
  exeStart = millis();
  uint32_t notificationValue = -1;

  char sz[32];
  char dz[32];
          
  if (xTaskNotifyWait(0, 0, &notificationValue, pdMS_TO_TICKS(300000))) {
  
    // #ifdef DEBUG_GPS
    //   Serial.print("Location Age: ");
    //   Serial.print(gps.location.age());
    //   Serial.print(" | Number of Satellites: ");
    //   Serial.print(gps.satellites.value());
    //   Serial.print(" | sentenceWithFix: ");
    //   Serial.print(gps.sentencesWithFix());
    //   Serial.println(" s");
    // #endif
    #ifdef DEBUG_GPS
      UBaseType_t freeHeap = uxTaskGetStackHighWaterMark(NULL);
      Serial.print("GPS Monitor Free Heap: ");
      Serial.println(freeHeap);
      appendFile(SD, logFile, "GPS Monitor Free Heap:");
      appendFile(SD, logFile, String(freeHeap).c_str());
    #endif
      
    switch (notificationValue) {
      
      case 0:
        #ifdef DEBUG_GPS
          Serial.println("GPS data received is IDEAL");
          appendFile(SD, logFile, "GPS data received is IDEAL");
        #endif
        d = gps.date;
        t = gps.time;
        setClock = true;


        sprintf(sz, "%02d/%02d/%02d", d.month(), d.day(), d.year());
        sprintf(dz, "%02d:%02d:%02d", t.hour(), t.minute(), t.second());
        sprintf(gpsBuffer, "%s,%s,%3.6f,%3.6f,%d,%4.1f,%d,%2.2f,%3.2f", sz, dz,
                  gps.location.lat(), gps.location.lng(), gps.location.age(), 
                  gps.altitude.meters(), gps.satellites.value(), gps.speed.knots(), 
                  gps.course.deg() 
                  );
        #ifdef DEBUG_GPS
          Serial.print("GPS Data Parsed: ");
          Serial.println(gpsBuffer);
        #endif

        appendFile(SD, gpsFile, gpsBuffer);

        xTaskNotify(GPSWorker, 0, eSetValueWithOverwrite);
        
        GPS.sendCommand(PMTK_STANDBY);
        vTaskDelay(1000);

        gpsComplete = true;
        vTaskDelay(50);
        vTaskDelete(GPSMon);

      case 1:
        #ifdef DEBUG_GPS
          Serial.println("GPS data received is NOT IDEAL");
          appendFile(SD, logFile, "GPS data received is NOT IDEAL");
        #endif
        d = gps.date;
        t = gps.time;

        sprintf(sz, "%02d/%02d/%02d", d.month(), d.day(), d.year());
        sprintf(dz, "%02d:%02d:%02d", t.hour(), t.minute(), t.second());
        sprintf(gpsBuffer, "%s,%s,%3.6f,%3.6f,%d,%4.1f,%d,%2.2f,%3.2f", sz, dz,
                  gps.location.lat(), gps.location.lng(), gps.location.age(), 
                  gps.altitude.meters(), gps.satellites.value(), gps.speed.knots(), 
                  gps.course.deg() 
                  );
        
        #ifdef DEBUG_GPS
          Serial.print("GPS Data Parsed: ");
          Serial.println(gpsBuffer);
        #endif

        appendFile(SD, gpsFile, gpsBuffer);
        
        xTaskNotify(GPSWorker, 0, eSetValueWithOverwrite);
        
        GPS.sendCommand(PMTK_STANDBY);
        vTaskDelay(1000);
        
        gpsComplete = true;
        vTaskDelay(50);
        vTaskDelete(GPSMon);

      default: 

        sprintf(gpsBuffer, "No Fix ,,,,,,,,,\n");
        #ifdef DEBUG_GPS
          Serial.println("Failed To receive new GPS data");
          Serial.println(gpsBuffer);
          appendFile(SD, logFile, "Failed To receive new GPS data");
        #endif
        
        GPS.sendCommand(PMTK_STANDBY);
        vTaskDelay(1000);
        
        xTaskNotify(GPSWorker, -1, eSetValueWithOverwrite);

        gpsComplete = true;
        vTaskDelay(50);
        vTaskDelete(GPSMon);
    }

 
  }
  else {
      
    sprintf(gpsBuffer, "No Fix ,,,,,,,,,\n");
    
    #ifdef DEBUG_GPS
      Serial.println("Failed To receive new GPS data");
      Serial.println(gpsBuffer);
      appendFile(SD, logFile, "Failed To receive new GPS data\n");
    #endif

      GPS.sendCommand(PMTK_STANDBY);
      vTaskDelay(1000);

      xTaskNotify(GPSWorker, -1, eSetValueWithOverwrite);

      gpsComplete = true;
      vTaskDelay(50);
      vTaskDelete(GPSMon);
      vTaskDelete(GPSWorker);
    
  }

}


/*
    GPS Monitor Call Task Handle
*/

void Task_GPS_Monitor() {

  xTaskCreatePinnedToCore(
    GPSMonitorFunc, /* Function to implement the task */
    "GPSMonitor", /* Name of the task */
    20000, /* Stack size in words */
    NULL, /* Task input parameter */
    1, /* Priority of the task */
    &GPSMon, /* Task handle. */
    1); /* Core where the task should run */
}
