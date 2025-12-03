
int moStatus = -1;
int mtStatus = -1;
int moMSN = -1;
int mtMSN = -1;
int mtLength = -1;
int mtQueued = -1;

void parseSBDResponse(String response) {
  // Look for the "+SBDIX" response
  // if (response.startsWith("+SBDIX:")) {
    // Remove the "+SBDIX: " prefix
    response.remove(0, 10);

    // Tokenize the response using commas as delimiters
    moStatus = response.substring(0, response.indexOf(',')).toInt();
    response = response.substring(response.indexOf(',') + 1);

    moMSN = response.substring(0, response.indexOf(',')).toInt();
    response = response.substring(response.indexOf(',') + 1);
    
    mtStatus = response.substring(0, response.indexOf(',')).toInt();
    response = response.substring(response.indexOf(',') + 1);

    mtMSN = response.substring(0, response.indexOf(',')).toInt();
    response = response.substring(response.indexOf(',') + 1);

    mtLength = response.substring(0, response.indexOf(',')).toInt();
    response = response.substring(response.indexOf(',') + 1);

    mtQueued = response.toInt();

    #ifdef DEBUG_SAT
      // Print parsed values for verification
      Serial.println("Parsed SBDIX Response:");
      Serial.print("MO Status: "); Serial.println(moStatus);
      Serial.print("MT Status: "); Serial.println(mtStatus);
      Serial.print("MO MSN: "); Serial.println(moMSN);
      Serial.print("MT MSN: "); Serial.println(mtMSN);
      Serial.print("MT Length: "); Serial.println(mtLength);
      Serial.print("MT Queued: "); Serial.println(mtQueued);
    #endif
}