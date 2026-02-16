#include <math.h>

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))

TaskHandle_t ACCMon;
TaskHandle_t IMUCons;

char IMUdata[128];

int ISM330DHCX_Hz = 2500;

void configureWakeupInterrupt_IMU() {
  // Configure the interrupt pin outputs
  ism330dhcx.configIntOutputs(true, true);

  // Set the interrupt to be triggered on INT1 pin for wake-up detection
  ism330dhcx.configInt1(false, false, false, false, true);
  
  // Enable wake-up detection
  ism330dhcx.enableWakeup(true, 0, 20); //Threshold (0 to 64) = 40 * 0.0313 g = 1.252 g
                                        //Threshold (0 to 64) = 20 * 0.0313 g = 0.626 g
                                        // The duration is set to 0, meaning the wake-up interrupt will be triggered immediately when the threshold is crossed                
}

void IRAM_ATTR wakeUpDetectedIMU() {
  // Serial.println("Wake-up event from the IMU detected!");
  // Add your code to handle the wake-up event here
  EXT_IMU_INT = true;
}




void printAccelRange() {
  Serial.print("Accelerometer range set to: ");
  switch (ism330dhcx.getAccelRange()) {
    case LSM6DS_ACCEL_RANGE_2_G:
      Serial.println("+-2G");  break;
    case LSM6DS_ACCEL_RANGE_4_G:
      Serial.println("+-4G");  break;
    case LSM6DS_ACCEL_RANGE_8_G:
      Serial.println("+-8G");  break;
    case LSM6DS_ACCEL_RANGE_16_G:
      Serial.println("+-16G");  break;
  }
}

void printAccelDataRate() {
  Serial.print("Accelerometer data rate set to: ");
  switch (ism330dhcx.getAccelDataRate()) {
    case LSM6DS_RATE_SHUTDOWN:
      Serial.println("0 Hz");  break;
    case LSM6DS_RATE_12_5_HZ:
      Serial.println("12.5 Hz");  break;
    case LSM6DS_RATE_26_HZ:
      Serial.println("26 Hz");  break;
    case LSM6DS_RATE_52_HZ:
      Serial.println("52 Hz");  break;
    case LSM6DS_RATE_104_HZ:
      Serial.println("104 Hz");  break;
    case LSM6DS_RATE_208_HZ:
      Serial.println("208 Hz");  break;
    case LSM6DS_RATE_416_HZ:
      Serial.println("416 Hz");  break;
    case LSM6DS_RATE_833_HZ:
      Serial.println("833 Hz");  break;
    case LSM6DS_RATE_1_66K_HZ:
      Serial.println("1.66 KHz");  break;
    case LSM6DS_RATE_3_33K_HZ:
      Serial.println("3.33 KHz");  break;
    case LSM6DS_RATE_6_66K_HZ:
      Serial.println("6.66 KHz");   break;
  }   
}

void printGyroDataRate() {
  Serial.print("Gyro data rate set to: ");
  switch (ism330dhcx.getGyroDataRate()) {
    case LSM6DS_RATE_SHUTDOWN:
      Serial.println("0 Hz");  break;
    case LSM6DS_RATE_12_5_HZ:
      Serial.println("12.5 Hz");  break;
    case LSM6DS_RATE_26_HZ:
      Serial.println("26 Hz");  break;
    case LSM6DS_RATE_52_HZ:
      Serial.println("52 Hz");  break;
    case LSM6DS_RATE_104_HZ:
      Serial.println("104 Hz");  break;
    case LSM6DS_RATE_208_HZ:
      Serial.println("208 Hz");  break;
    case LSM6DS_RATE_416_HZ:
      Serial.println("416 Hz");  break;
    case LSM6DS_RATE_833_HZ:
      Serial.println("833 Hz");  break;
    case LSM6DS_RATE_1_66K_HZ:
      Serial.println("1.66 KHz");  break;
    case LSM6DS_RATE_3_33K_HZ:
      Serial.println("3.33 KHz");  break;
    case LSM6DS_RATE_6_66K_HZ:
      Serial.println("6.66 KHz");  break;
  }
}

void printIMUData() {
  Serial.print("Ax: ");
  Serial.print(ax);
  Serial.print(" - Ay: ");
  Serial.print(ay);
  Serial.print(" - Az: ");
  Serial.println(az);

  Serial.print("Gx: ");
  Serial.print(gx);
  Serial.print(" - Gy: ");
  Serial.print(gy);
  Serial.print(" - Gz: ");
  Serial.println(gz);

  Serial.print("Mx: ");
  Serial.print(mx);
  Serial.print(" - My: ");
  Serial.print(my);
  Serial.print(" - Mz: ");
  Serial.println(mz);

  Serial.print("Temp: ");
  Serial.println(temp);
}

void ISM330DHCX_worker(void * parameter) {
  int ms;
  DateTime imuTime;

  sensors_event_t accel, g, tmp;
  sensors_event_t event;
  // not sure if we want to do running average stats

  while (!tasksComplete) {
    // ms = millis();
    bool success = false;
    while(!success) {
      if (xSemaphoreTake( i2cSemaphore, (TickType_t) 10) == pdTRUE) {
        
        // imuTime = rtc.now();
        // int MS = millis() % 1000;

        char dateBuffer[32] = "MM/DD/YYYY";
        char timeBuffer[32] = "hh:mm:ss";
        rtc.now().toString(dateBuffer);
        rtc.now().toString(timeBuffer);

        ism330dhcx.getEvent(&accel, &g, &tmp);
        lis3mdl.getEvent(&event);

        xSemaphoreGive(i2cSemaphore);

        ax = accel.acceleration.x;
        ay = accel.acceleration.y;
        az = accel.acceleration.z;

        gx = g.gyro.x;
        gy = g.gyro.y;
        gz = g.gyro.z;

        mx = event.magnetic.x;
        my = event.magnetic.y;
        mz = event.magnetic.z;

        temp = tmp.temperature;

        #ifdef DEBUG_IMU
          Serial.println("IMU Data Updated: ");
          printIMUData();
          appendFile(SD, logFile, "IMU Data Updated\n");
        #endif

        sprintf(imuBuffer, "%s,%s,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%3.1f,%i",dateBuffer, timeBuffer, 
                  ax, ay, az, gx, gy, gz, mx, my, mz, temp, imuWakeupCount);
                  
        appendFile(SD, imuFile, imuBuffer);

        success = true;
        vTaskDelay(ISM330DHCX_Hz);
      } else  {
        vTaskDelay(20);
        #ifdef DEBUG_IMU
          Serial.println("IMU stuck outer");
          appendFile(SD, logFile, "IMU stuck outer\n");
        #endif
      } 

    }
  }
  #ifdef DEBUG_IMU
    Serial.println("finished and closed");
  #endif
  imuComplete = true;
  vTaskDelete(ACCMon);
}

void Task_ISM330DHCX_Worker() {
  xTaskCreatePinnedToCore(
    ISM330DHCX_worker, /* Function to implement the task */
    "ISM330DHCXWorker", /* Name of the task */
    20000, /* Stack size in words */
    NULL, /* Task input parameter */
    1, /* Priority of the task */
    &ACCMon, /* Task handle. */
    1
  ); 
}
