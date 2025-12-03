
void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0:     Serial.println("Wakeup caused by external signal using EXT_RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1:     Serial.println("Wakeup caused by external signal using IMU"); break;
    case ESP_SLEEP_WAKEUP_TIMER:    Serial.println("Wakeup caused by ONBOARD_RTC"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD: Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP:      Serial.println("Wakeup caused by ULP program"); break;
    default:                        Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
}

int execution_state() {
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();

  int state = -1;

  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0:     state = 0; break;
    case ESP_SLEEP_WAKEUP_EXT1:     state = 1; break;
    case ESP_SLEEP_WAKEUP_TIMER:    state = 2; break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD: state = 3; break;
    case ESP_SLEEP_WAKEUP_ULP:      state = 4; break;
    default:                        break;
  }
  return state;
}

void combine_data_buffers() {

    const char* gpsN = gpsBuffer;
    const char* accN = imuBuffer;
    const char* pwrN = pwrBuffer;
    const char* comma = ",";
    // const char* lbrk = "\n";
    // const char* header = "AT+SBDWT=";

    char* tmp;
    tmp = (char*)malloc(/*strlen(header)+*/strlen(gpsN)+strlen(accN)+strlen(pwrN)+5); /* make space for the new string (should check the return value ...) */
    // strcpy(tmp, header);
    strcpy(tmp, gpsN); /* copy name into the new var */
    strcat(tmp, comma); /* add the comma */
    strcat(tmp, accN); /* add the extension */
    strcat(tmp, comma); /* add the comma */
    strcat(tmp, pwrN);
    // strcat(tmp, lbrk); /* add the line break character */

    int s = strlen(tmp);
    Serial.print("Buffer Size = ");
    Serial.println(s);
    sprintf(fileBuffer,tmp);
    
    free(tmp);

}


void listDir(fs::FS &fs, const char *dirname, uint8_t levels) {
  #ifdef DEBUG_FILE
    Serial.printf("Listing directory: %s\n", dirname);
  #endif

  File root = fs.open(dirname);
  if (!root) {
    #ifdef DEBUG_FILE
      Serial.println("Failed to open directory");
    #endif
    return;
  }
  if (!root.isDirectory()) {
    #ifdef DEBUG_FILE
      Serial.println("Not a directory");
    #endif
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      #ifdef DEBUG_FILE
        Serial.print("  DIR : ");
        Serial.println(file.name());
      #endif

      if (levels) {
        listDir(fs, file.path(), levels - 1);
      }
    } else {
      #ifdef DEBUG_FILE
        Serial.print("  FILE: ");
        Serial.print(file.name());
        Serial.print("  SIZE: ");
        Serial.println(file.size());
      #endif
    }
    file = root.openNextFile();
  }
}

void createDir(fs::FS &fs, const char *path) {
  #ifdef DEBUG_FILE
    Serial.printf("Creating Dir: %s\n", path);
  #endif

  if (fs.mkdir(path)) {
    #ifdef DEBUG_FILE
      Serial.println("Dir created");
    #endif
  } else {
    #ifdef DEBUG_FILE
      Serial.println("mkdir failed");
    #endif
  }
}

void removeDir(fs::FS &fs, const char *path) {
  #ifdef DEBUG_FILE
    Serial.printf("Removing Dir: %s\n", path);
  #endif

  if (fs.rmdir(path)) {
    #ifdef DEBUG_FILE
      Serial.println("Dir removed");
    #endif
  } else {
    #ifdef DEBUG_FILE
      Serial.println("rmdir failed");
    #endif
  }
}

void readFile(fs::FS &fs, const char *path) {
  #ifdef DEBUG_FILE
    Serial.printf("Reading file: %s\n", path);
  #endif

  File file = fs.open(path);
  if (!file) {
    #ifdef DEBUG_FILE
      Serial.println("Failed to open file for reading");
    #endif
    return;
  }
  #ifdef DEBUG_FILE
    Serial.print("Read from file: ");
  #endif
  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();
}

void writeFile(fs::FS &fs, const char *path, const char *message) {
  #ifdef DEBUG_FILE
    Serial.printf("Writing file: %s\n", path);
  #endif

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    #ifdef DEBUG_FILE
      Serial.println("Failed to open file for writing");
    #endif
    return;
  }
  if (file.print(message)) {
    #ifdef DEBUG_FILE
      Serial.println("File written");
    #endif
  } else {
    #ifdef DEBUG_FILE
      Serial.println("Write failed");
    #endif
  }
  file.close();
}

void appendFile(fs::FS &fs, const char *path, const char *message) {
  #ifdef DEBUG_FILE
    Serial.printf("Appending to file: %s\n", path);
  #endif

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    #ifdef DEBUG_FILE
      Serial.println("Failed to open file for appending");
    #endif
    return;
  }
  if (file.println(message)) {
    #ifdef DEBUG_FILE
      Serial.print("Message appended: ");
      Serial.println(message);
    #endif
  } else {
    #ifdef DEBUG_FILE
      Serial.println("Append failed");
    #endif
  }
  file.close();
}

void renameFile(fs::FS &fs, const char *path1, const char *path2) {
  #ifdef DEBUG_FILE
    Serial.printf("Renaming file %s to %s\n", path1, path2);
  #endif
  if (fs.rename(path1, path2)) {
    #ifdef DEBUG_FILE
      Serial.println("File renamed");
    #endif
  } else {
    #ifdef DEBUG_FILE
      Serial.println("Rename failed");
    #endif
  }
}

void deleteFile(fs::FS &fs, const char *path) {
  #ifdef DEBUG_FILE
    Serial.printf("Deleting file: %s\n", path);
  #endif
  if (fs.remove(path)) {
    #ifdef DEBUG_FILE
      Serial.println("File deleted");
    #endif
  } else {
    #ifdef DEBUG_FILE
      Serial.println("Delete failed");
    #endif
  }
}

void testFileIO(fs::FS &fs, const char *path) {
  File file = fs.open(path);
  static uint8_t buf[512];
  size_t len = 0;
  uint32_t start = millis();
  uint32_t end = start;
  if (file) {
    len = file.size();
    size_t flen = len;
    start = millis();
    while (len) {
      size_t toRead = len;
      if (toRead > 512) {
        toRead = 512;
      }
      file.read(buf, toRead);
      len -= toRead;
    }
    end = millis() - start;
    #ifdef DEBUG_FILE
      Serial.printf("%u bytes read for %lu ms\n", flen, end);
    #endif
    file.close();
  } else {
    #ifdef DEBUG_FILE
      Serial.println("Failed to open file for reading");
    #endif
  }

  file = fs.open(path, FILE_WRITE);
  if (!file) {
    #ifdef DEBUG_FILE
      Serial.println("Failed to open file for writing");
    #endif
    return;
  }

  size_t i;
  start = millis();
  for (i = 0; i < 2048; i++) {
    file.write(buf, 512);
  }
  end = millis() - start;
  #ifdef DEBUG_FILE
    Serial.printf("%u bytes written for %lu ms\n", 2048 * 512, end);
  #endif
  file.close();
}

void parseCSVLine(String line) {
  int startIndex = 0;
  int commaIndex = line.indexOf(',');  // Find the first comma
  int i = 0;
  // Loop through the entire line and extract the values separated by commas
  while (commaIndex != -1) {
    String value = line.substring(startIndex, commaIndex);  // Extract value before comma
    #ifdef DEBUG_FILE
    Serial.print("Value: ");
    Serial.println(value);
    Serial.println(i);
    #endif
    i++;

    // Move to the next value
    startIndex = commaIndex + 1; 
    commaIndex = line.indexOf(',', startIndex);  // Find next comma
  }

  // Print the last value after the last comma
  String lastValue = line.substring(startIndex);
  #ifdef DEBUG_FILE
  Serial.print("Value: ");
  Serial.println(lastValue);
  Serial.println(i);
  #endif
}

void parseFile(fs::FS &fs, const char *path) {
  #ifdef DEBUG_FILE
    Serial.printf("Reading file: %s\n", path);
  #endif

  File file = fs.open(path);
  if (!file) {
    #ifdef DEBUG_FILE
      Serial.println("Failed to open file for reading");
    #endif
    return;
  }
  String headerLine = file.readStringUntil('\n');
  #ifdef DEBUG_FILE
    Serial.print("Read from file: ");
    Serial.println("Reading and parsing data.csv...");
    Serial.println("Header: " + headerLine);
  #endif
 
  while (file.available()) {
    String line = file.readStringUntil('\n');  // Read a line from the CSV
    parseCSVLine(line);      
  }
  file.close();
}