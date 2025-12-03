#include <Wire.h>
#include <SoftwareSerial.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <Adafruit_GPS.h>
#include <Adafruit_INA219.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_ISM330DHCX.h>
#include <Adafruit_Sensor.h>
#include "RTClib.h"
#include <IridiumSBD.h>

TaskHandle_t mainTaskHandle;

// #define DEBUG_MAIN
// #define DEBUG_LIGHT
// #define DEBUG_IMU
// #define DEBUG_GPS
// #define DEBUG_PWR
// #define DEBUG_SAT
// #define DEBUG_FILE

#define GPSSerial Serial1

RTC_PCF8523 rtc;
Adafruit_INA219 ina219;
Adafruit_LIS3MDL lis3mdl;
Adafruit_ISM330DHCX ism330dhcx;
Adafruit_GPS GPS(&GPSSerial);

int GLED = 15;
int RLED = 4;

int signalQuality = -1;
int err;
const byte SAT_SLEEP = 13;
const byte SAT_TX = 12;
const byte SAT_RX = 27;
SoftwareSerial IridiumSerial(SAT_RX, SAT_TX);
IridiumSBD modem(IridiumSerial);
enum SBDState
{
  SBD_IDLE,
  SBD_WAIT_FOR_RESPONSE,
  SBD_TRANSMIT,
  SBD_RECEIVE,
  SBD_PASS
} stateSBD;
unsigned long lastActionTime = 0;
// const unsigned long timeoutInterval = 5000;  // 5 seconds timeout for each step
char sbdBuffer[100]; // Buffer to store SBD messages
bool transmissionDone = false;
int IridiumStart = 0;
bool successfulSatTransmission = false;
// int IridiumTimeout = 5000;  //ms for irridium send (Reduced to 5 seconds for indoor testing, revert to 180 sec)

bool gpsComplete;
bool imuComplete;
bool pwrComplete;
bool satComplete;
bool tasksComplete;
bool fault = false;
bool setClock;

SemaphoreHandle_t i2cSemaphore = NULL;
SemaphoreHandle_t serialSemaphore = NULL;
SemaphoreHandle_t uartSemaphore = NULL;

DateTime now;
bool EXT_RTC_INT = false;
bool EXT_IMU_INT = false;
int shortSleep, wakeup;

float busvoltage;
float current_mA;
float power_mW;

float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;

int imuWakeupCount = 0;

float temp;

QueueHandle_t IMUQueue;
char imuBuffer[128];

QueueHandle_t PWRQueue;
char pwrBuffer[128];

char gpsBuffer[128];
char fileBuffer[256];
char satBuffer[128];

char pwrFile[64];
char oldpwrFile[64];

char imuFile[64];
char oldimuFile[64];

char gpsFile[64];
char oldgpsFile[64];

char satFile[64];
char oldSatFile[64];

char satData[1024];

const char *pwrDir = "/pwr_data";
const char *imuDir = "/imu_data";
const char *gpsDir = "/gps_data";
const char *satDir = "/sat_data";

#include "ESP32_runtime.h"
#include "PCF8523_runtime.h"
#include "LIS3MDL_runtime.h"
#include "ISM330DHCX_runtime.h"
#include "INA219_runtime.h"
#include "Feather_GPS.h"
#include "IRIDIUM_runtime.h"

const uint64_t uS_TO_S_FACTOR = 1000000ULL;                     /* Conversion factor for micro seconds to seconds */
const int TIME_TO_SLEEP = 600;                                  /* Time ESP32 will go to sleep for Data Colection (in seconds) */
const uint8_t EXT_RTC_COUNTDOWN_TIMER = 1;                     // external RTC sleep timer(Must be < 255)
PCF8523TimerClockFreq countdown_unit = PCF8523_FrequencyHour; // Set the countdown timer frequency to 1 hour

char datafile[] = "";

RTC_DATA_ATTR int bootCount = 0;

const bool GPSECHO = false;

uint32_t timer = millis();

uint32_t bootTime = 0; // Store the boot time in seconds since epoch

uint32_t ulNotificationValue;

const int IMUInterruptPin = 14;

const int RTCInterruptPin = 32;
esp_sleep_ext1_wakeup_mode_t wakeup_mode = ESP_EXT1_WAKEUP_ALL_LOW; // Wake up on any high level on the selected pin(s)
volatile bool countdownInterruptTriggered = false;
volatile int numCountdownInterrupts = 0;

void setup()
{
  mainTaskHandle = xTaskGetCurrentTaskHandle();
  PWRQueue = xQueueCreate(10, sizeof(pwrBuffer)); // Create a queue that holds 5 pwrBuffer
  if (PWRQueue == NULL)
  {
#ifdef DEBUG_MAIN
    // Queue creation failed
    Serial.println("Power monitor queue created Successfully");
#endif
  }
  else
  {
#ifdef DEBUG_MAIN
    // Queue created successfully
    Serial.println("Power monitor queue failed to be created");
#endif
  }

  IMUQueue = xQueueCreate(10, sizeof(imuBuffer)); // Create a queue that holds 5 pwrBuffer
  if (IMUQueue == NULL)
  {
#ifdef DEBUG_MAIN
    // Queue creation failed
    Serial.println("IMU monitor queue created Successfully");
#endif
  }
  else
  {
#ifdef DEBUG_MAIN
    // Queue created successfully
    Serial.println("IMU monitor queue failed to be created");
#endif
  }

  pinMode(GLED, OUTPUT);
  pinMode(RLED, OUTPUT);
  pinMode(SAT_SLEEP, OUTPUT);
  digitalWrite(GLED, HIGH);
  digitalWrite(RLED, HIGH);
  digitalWrite(SAT_SLEEP, LOW);
  delay(100);

#ifdef DEBUG_MAIN
  Serial.begin(115200);
  while (!Serial)
  {
    // will pause Zero, Leonardo, etc until serial console opens
    delay(1);
  }
#endif

  if (!rtc.begin())
  {
#ifdef DEBUG_MAIN
    Serial.println("Couldn't find RTC");
#else
    fault = true;
    vTaskDelay(1);
#endif
  }
  else
  {
    rtc.deconfigureAllTimers();
    // set for the desired interval, Note these are set by multiples of a base
    // rtc.enableCountdownTimer(PCF8523_FrequencySecond, EXT_RTC_COUNTDOWN_TIMER);  // 30 seconds
    rtc.enableCountdownTimer(countdown_unit, EXT_RTC_COUNTDOWN_TIMER);
    pinMode(RTCInterruptPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(RTCInterruptPin), wakeUpDetectedRTC, FALLING);
    syncTimeWithRTC();
    bootTime = rtc.now().unixtime(); // Store the boot time in seconds since epoch
  }

  if (!SD.begin())
  {
#ifdef DEBUG_MAIN
    Serial.println("Card Mount Failed");
#endif
    fault = true;
    return;
  }
  else
  {
#ifdef DEBUG_MAIN
    Serial.println("Card Mounted");
#endif
  }

  GPS.begin(9600);
  GPS.sendCommand("");
  vTaskDelay(100);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  vTaskDelay(100);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  vTaskDelay(100);
  GPS.sendCommand(PGCMD_ANTENNA);
  vTaskDelay(100);
  GPS.sendCommand(PMTK_STANDBY);
  vTaskDelay(1000);
#ifdef DEBUG_MAIN
  Serial.println("Adafruit feather GPS setup commands sent and put to sleep");
#endif

  if (!ina219.begin())
  {
#ifdef DEBUG_MAIN
    Serial.println("Failed to find INA219 chip");
#endif
    fault = true;
  }
  else
  {
#ifdef DEBUG_MAIN
    Serial.println("Adafruit INA219 Test Success");
#endif
  }

  if (!lis3mdl.begin_I2C())
  { // hardware I2C mode, can pass in address & alt Wire
#ifdef DEBUG_MAIN
    Serial.println("Failed to find LIS3MDL chip");
#endif
    fault = true;
  }
  else
  {

#ifdef DEBUG_MAIN
    Serial.println("LIS3MDL Test Success");
#endif

    lis3mdl.setPerformanceMode(LIS3MDL_LOWPOWERMODE);
    printPerformanceMode();

    lis3mdl.setOperationMode(LIS3MDL_SINGLEMODE);
    printOperatingMode();

    lis3mdl.setDataRate(LIS3MDL_DATARATE_0_625_HZ);
    printDataRate();

    lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
    printRange();
  }

  if (!ism330dhcx.begin_I2C())
  { // hardware I2C mode, can pass in address & alt Wire
#ifdef DEBUG_MAIN
    Serial.println("Failed to find ism330dhcx chip");
#endif
    fault = true;
  }
  else
  {
#ifdef DEBUG_MAIN
    Serial.println("ism330dhcx test Success");
#endif

    ism330dhcx.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    printAccelRange();

    ism330dhcx.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
    printAccelDataRate();

    ism330dhcx.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
    printGyroDataRate();

    configureWakeupInterrupt_IMU();

    // Configure the interrupt pin on the Adafruit Feather ESP32 V2
    pinMode(IMUInterruptPin, INPUT_PULLUP); // Use GPIO 14, which corresponds to the pin labeled D14 on the Feather ESP32 V2
    attachInterrupt(digitalPinToInterrupt(IMUInterruptPin), wakeUpDetectedIMU, FALLING);
  }

  digitalWrite(SAT_SLEEP, HIGH);
  delay(1000);
  IridiumSerial.begin(19200);

// Begin satellite modem operation
#ifdef DEBUG_MAIN
  Serial.println("Starting modem...");
#endif
  stateSBD = SBD_IDLE;
  err = modem.begin();
  if (err != ISBD_SUCCESS)
  {
#ifdef DEBUG_MAIN
    Serial.print("Begin failed: error ");
    Serial.println(err);
#endif
    fault = true;
    if (err == ISBD_NO_MODEM_DETECTED)
#ifdef DEBUG_MAIN
      Serial.println("No modem detected: check wiring.");
#endif
    return;
  }

  digitalWrite(GLED, LOW);
  digitalWrite(RLED, LOW);
  digitalWrite(SAT_SLEEP, LOW);

  // Configure semaphores
  i2cSemaphore = xSemaphoreCreateMutex();
  serialSemaphore = xSemaphoreCreateMutex();
  uartSemaphore = xSemaphoreCreateMutex();

  // Create Directories and Initialize the file names
  if (xSemaphoreTake(uartSemaphore, (TickType_t)10) == pdTRUE)
  {
    
    #ifdef DEBUG_MAIN
      Serial.println("  Creating New DataFiles  ");
#endif

      char timeBuffer[32] = "YYYYMMDD-hhmmss";
      rtc.now().toString(timeBuffer);

      sprintf(pwrFile, "%s/pwr-data-%s.csv", pwrDir, timeBuffer);
      createDir(SD, pwrDir);
      writeFile(SD, pwrFile, "Date, Time, Voltage (V), Current (mA), Power (mW), Uptime (hrs)\n");

      sprintf(imuFile, "%s/imu-data-%s.csv", imuDir, timeBuffer);
      createDir(SD, imuDir);
      writeFile(SD, imuFile, "Date, Time, Ax (m/s^2), Ay (m/s^2), Az (m/s^2), Gx (/s), Gy (/s), Gz (/s), Mx (T), My (T), Mz (T), Temp (C), IMU Wakeup Count\n");

      sprintf(gpsFile, "%s/gps-data-%s.csv", gpsDir, timeBuffer);
      createDir(SD, gpsDir);
      writeFile(SD, gpsFile, "Date, Time, Latitude, Longitude, Location Age, Altitude (m), Satellite Count, Speed (knots), Course (deg)\n");

      sprintf(satFile, "%s/sat-data-%s.csv", satDir, timeBuffer);
      createDir(SD, satDir);
      writeFile(SD, satFile, "SatMsgSuccess?, Date, Time, Latitude, Longitude, Location Age, Altitude (m), Satellite Count, Speed (knots), Course (deg), Date, Time, Ax (m/s^2), Ay (m/s^2), Az (m/s^2), Gx (/s), Gy (/s), Gz (/s), Mx (T), My (T), Mz (T), Temp (C), IMU Wakeup Count, Date, Time, Voltage (V), Current (mA), Power (mW), Uptime (hrs)\n");

#ifdef DEBUG_MAIN
      listDir(SD, "/", 0);
#endif
    
//     char timeBuffer[32] = "YYYYMMDD-hhmmss";
//     rtc.now().toString(timeBuffer);

//     sprintf(oldpwrFile, "%s", pwrFile);
//     sprintf(pwrFile, "%s/pwr-data-%s.csv", pwrDir, timeBuffer);
//     writeFile(SD, pwrFile, "Date, Time, Voltage (V), Current (mA), Power (mW), Uptime (s)\n");
//     sprintf(oldimuFile, "%s", imuFile);
//     sprintf(imuFile, "%s/imu-data-%s.csv", imuDir, timeBuffer);
//     writeFile(SD, imuFile, "Date, Time, Ax (m/s^2), Ay (m/s^2), Az (m/s^2), Gx (/s), Gy (/s), Gz (/s), Mx (T), My (T), Mz (T), Temp (C), IMU Wakeup Count\n");
//     sprintf(oldgpsFile, "%s", gpsFile);
//     sprintf(gpsFile, "%s/gps-data-%s.csv", gpsDir, timeBuffer);
//     writeFile(SD, gpsFile, "Date, Time, Latitude, Longitude, Location Age, Altitude (m), Satellite Count, Speed (knots), Course (deg)\n");
//     sprintf(oldSatFile, "%s", satFile);
//     sprintf(satFile, "%s/sat-data-%d.csv", satDir, timeBuffer);
//     writeFile(SD, satFile, ", , \n");

// #ifdef DEBUG_MAIN
//     listDir(SD, "/", 0);
// #endif
    xSemaphoreGive(uartSemaphore);
  }
}

void loop()
{

  tasksComplete = false;
  gpsComplete = false;
  imuComplete = false;
  pwrComplete = false;
  setClock = false;

  digitalWrite(GLED, LOW);
  vTaskDelay(5);
  digitalWrite(RLED, LOW);
  vTaskDelay(5);
  /*
  if (fault) {
    int f;
    for (f = 0; f > 9; f++) {
      blinkRed(10 - f);
      vTaskDelay(250);
    }
    esp_restart();
  }
  */

  bool summarizeNow = false;
  int timeSlept;
  int state = execution_state();
  if (EXT_RTC_INT)
  {
    EXT_RTC_INT = false;
    state = 0;
  }
  else if (EXT_IMU_INT)
  {
    EXT_IMU_INT = false;
    state = 1;
  }

  // State machine to handle different wake up conditions
  switch (state)
  {

  // Case 0: External RTC Interrupt - Send sat message and collect data
  /*----------------------------------------------------------------------------------*/
  case 0:
    blinkGreen(1);
#ifdef DEBUG_LIGHT
    Serial.println("Start Case 0");
#endif
    digitalWrite(SAT_SLEEP, HIGH);
#ifdef DEBUG_MAIN
    Serial.print(" -- Awoke from External RTC interrupt -- ");
    if (xSemaphoreTake(i2cSemaphore, (TickType_t)10) == pdTRUE)
    {
      xSemaphoreGive(i2cSemaphore);
    }
    printTime(now);
#endif

    if (xSemaphoreTake(uartSemaphore, (TickType_t)10) == pdTRUE)
    {
      char timeBuffer[32] = "YYYYMMDD-hhmmss";
      rtc.now().toString(timeBuffer);

      sprintf(oldpwrFile, "%s", pwrFile);
      sprintf(pwrFile, "%s/pwr-data-%s.csv", pwrDir, timeBuffer);
      writeFile(SD, pwrFile, "Date, Time, Voltage (V), Current (mA), Power (mW), Uptime (hrs)\n");
      sprintf(oldimuFile, "%s", imuFile);
      sprintf(imuFile, "%s/imu-data-%s.csv", imuDir, timeBuffer);
      writeFile(SD, imuFile, "Date, Time, Ax (m/s^2), Ay (m/s^2), Az (m/s^2), Gx (/s), Gy (/s), Gz (/s), Mx (T), My (T), Mz (T), Temp (C), IMU Wakeup Count\n");
      sprintf(oldgpsFile, "%s", gpsFile);
      sprintf(gpsFile, "%s/gps-data-%s.csv", gpsDir, timeBuffer);
      writeFile(SD, gpsFile, "Date, Time, Latitude, Longitude, Location Age, Altitude (m), Satellite Count, Speed (knots), Course (deg)\n");
      sprintf(oldSatFile, "%s", satFile);
      sprintf(satFile, "%s/sat-data-%s.csv", satDir, timeBuffer);
      writeFile(SD, satFile, "SatMsgSuccess?, Date, Time, Latitude, Longitude, Location Age, Altitude (m), Satellite Count, Speed (knots), Course (deg), Date, Time, Ax (m/s^2), Ay (m/s^2), Az (m/s^2), Gx (/s), Gy (/s), Gz (/s), Mx (T), My (T), Mz (T), Temp (C), IMU Wakeup Count, Date, Time, Voltage (V), Current (mA), Power (mW), Uptime (hrs)\n");

#ifdef DEBUG_MAIN
      listDir(SD, "/", 0);
#endif
      xSemaphoreGive(uartSemaphore);
    }

    Task_INA219_Worker();
    Task_ISM330DHCX_Worker();
    Task_GPS_Monitor();

#ifdef DEBUG_MAIN
    Serial.println("Running External RTC interrupt Loop . . . . . . ");
#endif

    stateSBD = SBD_IDLE;
    tasksComplete = false;
    IridiumStart = millis();
    satComplete = false;
    while (!satComplete)
    {
      if (gpsComplete)
      {
        vTaskDelay(25);
        tasksComplete = true;
        while (!summarizeNow)
        {
          if (pwrComplete && imuComplete)
          {
#ifdef DEBUG_MAIN
            Serial.println("Remaining Tasks Cleaned Up");
#endif

            combine_data_buffers();
#ifdef DEBUG_MAIN
            Serial.print("Collected Data: ");
            Serial.println(fileBuffer);
#endif
            summarizeNow = true;
          }
          else
          {
#ifdef DEBUG_MAIN
            Serial.println("Processes have not Finished");
#endif
            vTaskDelay(250);
          }
        }
        err = modem.sendSBDText(fileBuffer);
        if (err != ISBD_SUCCESS)
        {
          Serial.print("sendSBDText failed: error ");
          Serial.println(err);
          successfulSatTransmission = false;
          if (err == ISBD_SENDRECEIVE_TIMEOUT)
            Serial.println("Try again with a better view of the sky.");
        }

        else
        {
          Serial.println("Hey, it worked!");
          successfulSatTransmission = true;
          //imuWakeupCount = 0; // Reset IMU wakeup count only if sat message succesfully sent
        }
        satComplete = true;
        // if ( (millis() - IridiumStart) > IridiumTimeout ) {
        //   stateSBD = SBD_PASS;
        //   #ifdef DEBUG_MAIN
        //     Serial.println("Failed To send Iridium Message. Going to sleep......");
        //   #endif
        // } else {
        //   // #ifdef DEBUG_MAIN
        //   //   Serial.print("Time in Iridium Loop: ");
        //   //   Serial.println(millis() - IridiumStart);
        //   // #endif
        // }
        // // stateSBD = SBD_PASS; // Remove for deployment
        // switch (stateSBD) {
        //   case SBD_IDLE:
        //     // Send SBD message to modem
        //     #ifdef DEBUG_MAIN
        //       Serial.println("SBD_IDLE");
        //       Serial.println("Sending SBD message...");
        //     #endif
        //     IridiumSerial.print(fileBuffer); // Send the data to transmit
        //     Serial.println(fileBuffer);
        //     lastActionTime = millis();
        //     stateSBD = SBD_WAIT_FOR_RESPONSE;
        //     break;

        //   case SBD_WAIT_FOR_RESPONSE:
        //     #ifdef DEBUG_MAIN
        //       Serial.println("SBD_WAIT_FOR_RESPONSE");
        //     #endif
        //     // Wait for modem to respond to the AT command
        //     if (IridiumSerial.available()) {
        //       String response = IridiumSerial.readString();
        //       #ifdef DEBUG_MAIN
        //         Serial.println("Modem response: " + response);
        //       #endif
        //       if (response.indexOf("OK") != -1) {
        //         // Start SBD session after successfully writing message
        //         IridiumSerial.println("AT+SBDIX");
        //         stateSBD = SBD_TRANSMIT;
        //         lastActionTime = millis();
        //       } else {
        //         // Error or no OK, retry or handle error
        //         stateSBD = SBD_IDLE;
        //       }
        //     }
        //     else {
        //       #ifdef DEBUG_MAIN
        //         Serial.println("Iridium not available");
        //       #endif
        //     }
        //     break;

        //   case SBD_TRANSMIT:
        //     // #ifdef DEBUG_MAIN
        //     //   Serial.println(stateSBD);
        //     // #endif
        //     // Handle SBD transmission session
        //     if (IridiumSerial.available()) {
        //       String response = IridiumSerial.readString();
        //       #ifdef DEBUG_MAIN
        //         Serial.println("Transmission response: " + response);
        //       #endif
        //       if (response.indexOf("SBDIX:") != -1) {
        //         // Check if message successfully sent
        //         parseSBDResponse(response);
        //         if (moStatus == 0) {
        //           transmissionDone = true;
        //           satComplete = true;
        //           #ifdef DEBUG_MAIN
        //             Serial.println("Message sent successfully!");
        //           #endif
        //           stateSBD = SBD_IDLE;  // Reset to idle after success
        //           tasksComplete = true;
        //         } else {
        //           stateSBD = SBD_IDLE;
        //           #ifdef DEBUG_MAIN
        //             Serial.println("Transmission timed out Straight to IDLE.");
        //           #endif
        //         }
        //       } else if (millis() - lastActionTime > timeoutInterval) {
        //         // Timeout, handle retry or error
        //         #ifdef DEBUG_MAIN
        //           Serial.println("Transmission timed out.");
        //         #endif
        //         stateSBD = SBD_IDLE;
        //       }
        //     }
        //     break;

        //   case SBD_RECEIVE:
        //     #ifdef DEBUG_MAIN
        //       Serial.println(stateSBD);
        //     #endif
        //     // Handle SBD reception session if needed
        //     // Similar logic to SBD_TRANSMIT
        //     break;

        //   case SBD_PASS:
        //     #ifdef DEBUG_MAIN
        //       Serial.println(stateSBD);
        //     #endif
        //     tasksComplete = true;
        //     satComplete = true;
        //     break;
      }
      else
      {
        vTaskDelay(250);
      }
    }
    // summarizeNow = false;
    // while (!summarizeNow) {
    //   if (pwrComplete && imuComplete) {
    //     #ifdef DEBUG_MAIN
    //       Serial.println("Remaining Tasks Cleaned Up");
    //     #endif

    //     #ifdef DEBUG_MAIN
    //       Serial.print("Collected Data: ");
    //       Serial.println(fileBuffer);
    //     #endif
    //     summarizeNow = true;

    //   } else {

    //     #ifdef DEBUG_MAIN
    //       Serial.println("Processes have not Finished");
    //     #endif
    //     vTaskDelay(250);
    //   }
    // }

    if(!summarizeNow) {
      combine_data_buffers();
    }

    sprintf(satData, "%s,%s", successfulSatTransmission ? "true" : "false", fileBuffer);
    appendFile(SD, satFile, satData);

    digitalWrite(SAT_SLEEP, LOW);
    if (xSemaphoreTake(i2cSemaphore, (TickType_t)10) == pdTRUE)
    {
      now = rtc.now();
      xSemaphoreGive(i2cSemaphore);
    }
    timeSlept = now.unixtime() - shortSleep;
    if (timeSlept < TIME_TO_SLEEP)
    {
#ifdef DEBUG_MAIN
      Serial.print("Sleeping for: ");
      Serial.print(TIME_TO_SLEEP - timeSlept);
      Serial.println(" s");
#endif
      esp_sleep_enable_timer_wakeup((TIME_TO_SLEEP - timeSlept) * uS_TO_S_FACTOR);
    }
    else if ((timeSlept > TIME_TO_SLEEP) && ((timeSlept - TIME_TO_SLEEP) < TIME_TO_SLEEP))
    {
#ifdef DEBUG_MAIN
      Serial.print("Sleeping for: ");
      Serial.print(timeSlept - TIME_TO_SLEEP);
      Serial.println(" s");
#endif
      esp_sleep_enable_timer_wakeup((timeSlept - TIME_TO_SLEEP) * uS_TO_S_FACTOR);
    }
    else
    {
#ifdef DEBUG_MAIN
      Serial.print("Sleeping for: ");
      Serial.print(TIME_TO_SLEEP);
      Serial.println(" s");
#endif
      esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    }
    blinkRed(1);
#ifdef DEBUG_LIGHT
    Serial.println("End Case 0");
#endif
    break;

  // Case 1: External IMU Interrupt - Collect data and go back to sleep
  /*----------------------------------------------------------------------------------*/
  case 1:

    blinkALT(2);
#ifdef DEBUG_LIGHT
    Serial.println("Start Case 1");
#endif
    if (xSemaphoreTake(i2cSemaphore, (TickType_t)10) == pdTRUE)
    {
      now = rtc.now();
      xSemaphoreGive(i2cSemaphore);
    }
#ifdef DEBUG_MAIN
    Serial.print(" -- Awoke from IMU interrupt -- ");
    printTime(now);
#endif

    imuWakeupCount++;

    Task_INA219_Worker();
    Task_ISM330DHCX_Worker();
    Task_GPS_Monitor();

#ifdef DEBUG_MAIN
    Serial.println("Running IMU interrupt Loop . . . . . . ");
#endif

    while (!tasksComplete)
    {
      vTaskDelay(250);
      if (gpsComplete)
      {
        tasksComplete = true;
        while (!summarizeNow)
        {
          if (pwrComplete && imuComplete)
          {
#ifdef DEBUG_MAIN
            Serial.println("Remaining Tasks Cleaned Up");
#endif

            combine_data_buffers();
#ifdef DEBUG_MAIN
            Serial.print("Collected Data: ");
            Serial.println(fileBuffer);
#endif
            summarizeNow = true;
          }
          else
          {
#ifdef DEBUG_MAIN
            Serial.println("Processes have not Finished");
#endif
            vTaskDelay(250);
          }
        }
#ifdef DEBUG_MAIN
        Serial.println("IMU interrupt Tasks Complete");
#endif
      }
    }

    if (xSemaphoreTake(i2cSemaphore, (TickType_t)10) == pdTRUE)
    {
      now = rtc.now();
      xSemaphoreGive(i2cSemaphore);
    }
    timeSlept = now.unixtime() - shortSleep;
    if (timeSlept < TIME_TO_SLEEP)
    {
#ifdef DEBUG_MAIN
      Serial.print("Sleeping for: ");
      Serial.print(TIME_TO_SLEEP - timeSlept);
      Serial.println(" s");
#endif
      esp_sleep_enable_timer_wakeup((TIME_TO_SLEEP - timeSlept) * uS_TO_S_FACTOR);
    }
    else if ((timeSlept > TIME_TO_SLEEP) && ((timeSlept - TIME_TO_SLEEP) < TIME_TO_SLEEP))
    {
#ifdef DEBUG_MAIN
      Serial.print("Sleeping for: ");
      Serial.print(timeSlept - TIME_TO_SLEEP);
      Serial.println(" s");
#endif
      esp_sleep_enable_timer_wakeup((timeSlept - TIME_TO_SLEEP) * uS_TO_S_FACTOR);
    }
    else
    {
#ifdef DEBUG_MAIN
      Serial.print("Sleeping for: ");
      Serial.print(TIME_TO_SLEEP);
      Serial.println(" s");
#endif
      esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    }
    blinkRed(1);
#ifdef DEBUG_LIGHT
    Serial.println("End Case 1");
#endif
    break;

  /*----------------------------------------------------------------------------------*/
  case 2:

    blinkGreen(1);
#ifdef DEBUG_LIGHT
    Serial.println("Start Case 2");
#endif
    if (xSemaphoreTake(i2cSemaphore, (TickType_t)10) == pdTRUE)
    {
      now = rtc.now();
      xSemaphoreGive(i2cSemaphore);
    }
    shortSleep = now.unixtime();

#ifdef DEBUG_MAIN
    Serial.print(" -- Awoke from ESP32 RTC interrupt -- ");
    printTime(now);
#endif

    Task_INA219_Worker();
    Task_ISM330DHCX_Worker();
    Task_GPS_Monitor();

#ifdef DEBUG_MAIN
    Serial.println("Running Internal RTC interrupt Loop . . . . . . ");
#endif

    while (!tasksComplete)
    {
      vTaskDelay(250);
      if (gpsComplete)
      {
        tasksComplete = true;
        while (!summarizeNow)
        {
          if (pwrComplete && imuComplete)
          {
#ifdef DEBUG_MAIN
            Serial.println("Remaining Tasks Cleaned Up");
#endif

            combine_data_buffers();
#ifdef DEBUG_MAIN
            Serial.print("Collected Data: ");
            Serial.println(fileBuffer);
#endif
            summarizeNow = true;
          }
          else
          {

#ifdef DEBUG_MAIN
            Serial.println("Processes have not Finished");
#endif
            vTaskDelay(250);
          }
        }
#ifdef DEBUG_MAIN
        Serial.println("Internal RTC interrupt Tasks Complete ");
#endif
      }
    }
    if (xSemaphoreTake(i2cSemaphore, (TickType_t)10) == pdTRUE)
    {
      now = rtc.now();
      xSemaphoreGive(i2cSemaphore);
    }
    wakeup = now.unixtime() - shortSleep;
#ifdef DEBUG_MAIN
    Serial.print("Sleeping for: ");
    Serial.print(TIME_TO_SLEEP - wakeup);
    Serial.println(" s");
#endif
    esp_sleep_enable_timer_wakeup((TIME_TO_SLEEP - wakeup) * uS_TO_S_FACTOR);
    blinkRed(1);
#ifdef DEBUG_LIGHT
    Serial.println("End Case 2");
#endif
    break;

  /*----------------------------------------------------------------------------------*/
  default:

    vTaskDelay(200);
    blinkGreen(2);
    digitalWrite(RLED, LOW);
    vTaskDelay(5);
#ifdef DEBUG_LIGHT
    Serial.println("Start Default Case");
#endif

    if (xSemaphoreTake(i2cSemaphore, (TickType_t)10) == pdTRUE)
    {
      now = rtc.now();
      xSemaphoreGive(i2cSemaphore);
    }
    shortSleep = now.unixtime();

#ifdef DEBUG_MAIN
    Serial.println(" - - - - - - - - - - - - - - - - - - - - - - - ");
    Serial.print(" -- ESP32 Booted -- ");
    printTime(now);
    Serial.println(" - - - - - - - - - - - - - - - - - - - - - - - ");
#endif

    ///*
    if (xSemaphoreTake(uartSemaphore, (TickType_t)10) == pdTRUE)
    {
#ifdef DEBUG_MAIN
      Serial.println("  Creating New DataFiles  ");
#endif

      char timeBuffer[32] = "YYYYMMDD-hhmmss";
      rtc.now().toString(timeBuffer);

      sprintf(pwrFile, "%s/pwr-data-%s.csv", pwrDir, timeBuffer);
      createDir(SD, pwrDir);
      writeFile(SD, pwrFile, "Date, Time, Voltage (V), Current (mA), Power (mW), Uptime (hrs)\n");

      sprintf(imuFile, "%s/imu-data-%s.csv", imuDir, timeBuffer);
      createDir(SD, imuDir);
      writeFile(SD, imuFile, "Date, Time, Ax (m/s^2), Ay (m/s^2), Az (m/s^2), Gx (/s), Gy (/s), Gz (/s), Mx (T), My (T), Mz (T), Temp (C), IMU Wakeup Count\n");

      sprintf(gpsFile, "%s/gps-data-%s.csv", gpsDir, timeBuffer);
      createDir(SD, gpsDir);
      writeFile(SD, gpsFile, "Date, Time, Latitude, Longitude, Location Age, Altitude (m), Satellite Count, Speed (knots), Course (deg)\n");

      sprintf(satFile, "%s/sat-data-%s.csv", satDir, timeBuffer);
      createDir(SD, satDir);
      writeFile(SD, satFile, "SatMsgSuccess?????, Date, Time, Latitude, Longitude, Location Age, Altitude (m), Satellite Count, Speed (knots), Course (deg), Date, Time, Ax (m/s^2), Ay (m/s^2), Az (m/s^2), Gx (/s), Gy (/s), Gz (/s), Mx (T), My (T), Mz (T), Temp (C), IMU Wakeup Count, Date, Time, Voltage (V), Current (mA), Power (mW), Uptime (hrs)\n");

#ifdef DEBUG_MAIN
      listDir(SD, "/", 0);
#endif
      xSemaphoreGive(uartSemaphore);
    }
    //*/

    if (TIME_TO_SLEEP < (WAIT_SECS / 1000))
    { // If the time to sleep is less than the GPS wait time, set the wait time
      WAIT_SECS = (TIME_TO_SLEEP * 1000) - 5000;
    }

    Task_INA219_Worker();
    Task_ISM330DHCX_Worker();
    Task_GPS_Monitor();

#ifdef DEBUG_MAIN
    Serial.println("Running Initial Boot Loop . . . . . . ");
#endif

    while (!tasksComplete)
    {
      vTaskDelay(250);
      if (gpsComplete)
      {
        tasksComplete = true;
        while (!summarizeNow)
        {
          if (pwrComplete && imuComplete)
          {
#ifdef DEBUG_MAIN
            Serial.println("Remaining Tasks Cleaned Up");
#endif
            summarizeNow = true;
          }
          else
          {
#ifdef DEBUG_MAIN
            Serial.println("Processes have not Finished");
#endif
            vTaskDelay(250);
          }
        }
#ifdef DEBUG_MAIN
        Serial.println("Boot Tasks Complete ");
#endif
      }
    }

    if (xSemaphoreTake(i2cSemaphore, (TickType_t)10) == pdTRUE)
    {
      now = rtc.now();
      xSemaphoreGive(i2cSemaphore);
    }
    wakeup = now.unixtime() - shortSleep;
#ifdef DEBUG_MAIN
    Serial.print("Sleeping for: ");
    Serial.print(TIME_TO_SLEEP - wakeup);
    Serial.println(" s");
#endif

    blinkRed(2);
#ifdef DEBUG_LIGHT
    Serial.println("End Default Case");
#endif
    esp_sleep_enable_timer_wakeup((TIME_TO_SLEEP - wakeup) * uS_TO_S_FACTOR);
  }

#ifdef DEBUG_MAIN
  Serial.println("::: Going to sleep now :::\n\n");
  Serial.flush();
#endif

  if (setClock)
  {
    if (xSemaphoreTake(i2cSemaphore, (TickType_t)100) == pdTRUE)
    {
      rtc.adjust(DateTime(d.year(), d.month(), d.day(), t.hour(), t.minute(), t.second()));
      xSemaphoreGive(i2cSemaphore);
    }
  }

  // esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

  // Case to handle if the RTC interupt happened while code was in another state
  if (!(EXT_RTC_INT || EXT_IMU_INT))
  {
    digitalWrite(GLED, LOW);
    digitalWrite(RLED, LOW);
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_32, 0); // Wake up when GPIO 32 goes LOW
    esp_sleep_enable_ext1_wakeup((1ULL << 14), ESP_EXT1_WAKEUP_ALL_LOW);
    esp_light_sleep_start();
  }
}

void blinkGreen(int times)
{
  int i;
  for (i = 0; i < times; i++)
  {
    digitalWrite(GLED, HIGH);
    delay(100);
    digitalWrite(GLED, LOW);
    delay(100);
  }
}

void blinkRed(int times)
{
  int i;
  for (i = 0; i < times; i++)
  {
    digitalWrite(RLED, HIGH);
    delay(100);
    digitalWrite(RLED, LOW);
    delay(100);
  }
}

void blinkALT(int times)
{
  int i;
  for (i = 0; i < times; i++)
  {
    digitalWrite(GLED, LOW);
    digitalWrite(RLED, HIGH);
    delay(100);
    digitalWrite(GLED, HIGH);
    digitalWrite(RLED, LOW);
    delay(100);
    digitalWrite(GLED, LOW);
  }
}

void syncTimeWithRTC()
{
  DateTime now = rtc.now();

  // Create time structure
  struct tm timeinfo;
  timeinfo.tm_year = now.year() - 1900; // Years since 1900
  timeinfo.tm_mon = now.month() - 1;    // Months since January (0-11)
  timeinfo.tm_mday = now.day();
  timeinfo.tm_hour = now.hour();
  timeinfo.tm_min = now.minute();
  timeinfo.tm_sec = now.second();
  timeinfo.tm_isdst = -1; // Let system determine DST

  // Convert to time_t
  time_t t = mktime(&timeinfo);

  // Set system time
  struct timeval tv = {.tv_sec = t};
  settimeofday(&tv, NULL);
}