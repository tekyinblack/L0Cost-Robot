// Bidirectional communications with Raspebery Pi platform running Python
/*
This code was introduced to facilitate the use of the L0Cost Robot architecture in the PiWars 
competition where one of the Raspeberry Pi platforms must be used. The L0Cost platform operates
in a subservient role to the Pi or Pico being used.
*/
/*
This code adds function to the command processing and response by adding the following features

The first 4 characters in a message are a serial number received from the Pi platform which are returned to the 
Pi unchanged in the response, to synchronise messages.
Any response will include a 4 character field after the serial number indicating the success of the message action.
Only OKOK indicates that the message action was successful.
*/
int picoProcessor(char picoCommand[], int source) {
  long picoTimer = millis();
  picoPinOff();

  int retVal = 0;
  int pauseValue = 0;
  long piTimer = millis();  // pi delay processing included to account for Pi synchronisation issues
  long piDelay = 0;         // with handshaking
  // store sequence number, basic but effective ;)
  sequenceNumber[0] = picoCommand[0];
  sequenceNumber[1] = picoCommand[1];
  sequenceNumber[2] = picoCommand[2];
  sequenceNumber[3] = picoCommand[3];
  // left shift command to obtain command
  {
    int i;
    for (i = 1; i < strlen(picoCommand); i++) {
      if (i > 30) break;                 // break if longer than allowed
      if (picoCommand[i] == ' ') break;  // break if blank character
      picoCommand[i - 4] = picoCommand[i];
    }
    picoCommand[i] = '\0';
  }
  if (cmdCmp(picoCommand, "WHOU") == 0) {  // command is WHOU, respond with name of controller
    Serial.printf("%sOKOK%s\r\n", sequenceNumber, configHostname);
    piDelay = millis() - piTimer;
    if (piDelay < 10) { delay(10 - piDelay); }

  } else if (cmdCmp(picoCommand, "PICOOFF") == 0) {  // exit pico processing
    execPico = 0;
    delay(10);
    if (debugSerial) { Serial.println("Pico processing exited"); }

  } else if (cmdCmp(picoCommand, "LINE") == 0) {  // respond with current line following readings
    // sequence is top line, centre line and bottom line refering to the lines in the camera image with 4 characters each and leading zeros
    Serial.printf("%sOKOK%04d%04d%04d%04d%04d%04d\r\n", sequenceNumber, lineInstancesT[1], lineInstancesT[2], lineInstancesC[1], lineInstancesC[2], lineInstancesB[1], lineInstancesB[2]);
    piDelay = millis() - piTimer;
    if (piDelay < 10) { delay(10 - piDelay); }

  } else if (cmdCmp(picoCommand, "LINC") == 0) {  // respond with current line following readings
    // sequence is top line, centre line and bottom line refering to the lines in the camera image with 4 characters each and leading zeros
    Serial.printf("%sOKOK%04d%04d\r\n", sequenceNumber, lineInstancesC[1], lineInstancesC[2]);
    piDelay = millis() - piTimer;
    if (piDelay < 10) { delay(10 - piDelay); }

  } else if (cmdCmp(picoCommand, "GETL") == 0) {  // get new line following readings and return them
    // sequence is top line, centre line and bottom line refering to the lines in the camera image with 4 characters each and leading zeros
    procGuidance(1);  // get video frame
    procGuidance(2);  // analyse frame
    procGuidance(3);  // delete frame
    Serial.printf("%sOKOK%04d%04d%04d%04d%04d%04d\r\n", sequenceNumber, lineInstancesT[1], lineInstancesT[2], lineInstancesC[1], lineInstancesC[2], lineInstancesB[1], lineInstancesB[2]);
    piDelay = millis() - piTimer;
    if (piDelay < 10) { delay(10 - piDelay); }

  } else if (cmdCmp(picoCommand, "GETC") == 0) {  // get new line following readings and return them
    // sequence is top line, centre line and bottom line refering to the lines in the camera image with 4 characters each and leading zeros
    procGuidance(1);  // get video frame
    procGuidance(2);  // analyse frame
    procGuidance(3);  // delete frame
    Serial.printf("%sOKOK%04d%04d\r\n", sequenceNumber, lineInstancesC[1], lineInstancesC[2]);
    piDelay = millis() - piTimer;
    if (piDelay < 10) { delay(10 - piDelay); }

  } else if (cmdCmp(picoCommand, "FLA+") == 0) {  // respond with current line following readings
    // sequence is top line, centre line and bottom line refering to the lines in the camera image with 4 characters each and leading zeros
    cmdProcessor("LFLASHON", 0);
    Serial.printf("%sOKOKFLASHON\r\n", sequenceNumber);
    piDelay = millis() - piTimer;
    if (piDelay < 10) { delay(10 - piDelay); }

  } else if (cmdCmp(picoCommand, "FLA-") == 0) {  // respond with current line following readings
    // sequence is top line, centre line and bottom line refering to the lines in the camera image with 4 characters each and leading zeros
    cmdProcessor("LFLASHOFF", 0);
    Serial.printf("%sOKOKFLASHOFF\r\n", sequenceNumber);
    piDelay = millis() - piTimer;
    if (piDelay < 10) { delay(10 - piDelay); }

  } else if (cmdCmp(picoCommand, "BLOB") == 0) {  // respond with current blob location
    // sequence is relative x co-ord, relative y co-ord with 4 characters each and leading zeros
    Serial.printf("%sOKOK%04d%04d%04d\r\n", sequenceNumber, lastBlobCount, lastBlobX, lastBlobY);
    piDelay = millis() - piTimer;
    if (piDelay < 10) { delay(10 - piDelay); }

  } else if (cmdCmp(picoCommand, "GETB") == 0) {  
    // set colour to BLUE and run blob search
    cmdProcessor("LGCOLOUR5", 0);
    procGuidance(1);  // get video frame
    procGuidance(2);  // analyse frame
    procGuidance(3);  // delete frame
    Serial.printf("%sOKOK%04d%04d%04d\r\n", sequenceNumber, lastBlobCount, lastBlobX, lastBlobY);
    piDelay = millis() - piTimer;
    if (piDelay < 10) { delay(10 - piDelay); }
  } else if (cmdCmp(picoCommand, "GETY") == 0) {  
    // set colour to YELLOW and run blob search
    cmdProcessor("LGCOLOUR3", 0);
    procGuidance(1);  // get video frame
    procGuidance(2);  // analyse frame
    procGuidance(3);  // delete frame
    Serial.printf("%sOKOK%04d%04d%04d\r\n", sequenceNumber, lastBlobCount, lastBlobX, lastBlobY);
    piDelay = millis() - piTimer;
    if (piDelay < 10) { delay(10 - piDelay); }
  } else if (cmdCmp(picoCommand, "GETR") == 0) {  
    // set colour to RED and run blob search
    cmdProcessor("LGCOLOUR1", 0);
    procGuidance(1);  // get video frame
    procGuidance(2);  // analyse frame
    procGuidance(3);  // delete frame
    Serial.printf("%sOKOK%04d%04d%04d\r\n", sequenceNumber, lastBlobCount, lastBlobX, lastBlobY);
    piDelay = millis() - piTimer;
    if (piDelay < 10) { delay(10 - piDelay); }
  } else if (cmdCmp(picoCommand, "GETG") == 0) { 
    // set colour to GREEN and run blob search
    cmdProcessor("LGCOLOUR4", 0);
    procGuidance(1);  // get video frame
    procGuidance(2);  // analyse frame
    procGuidance(3);  // delete frame
    Serial.printf("%sOKOK%04d%04d%04d\r\n", sequenceNumber, lastBlobCount, lastBlobX, lastBlobY);
    piDelay = millis() - piTimer;
    if (piDelay < 10) { delay(10 - piDelay); }
  } else if (cmdCmp(picoCommand, "GETW") == 0) {  
    // set colour to WHITE and run blob search
    cmdProcessor("LGCOLOUR10", 0);
    procGuidance(1);  // get video frame
    procGuidance(2);  // analyse frame
    procGuidance(3);  // delete frame
    Serial.printf("%sOKOK%04d%04d%04d\r\n", sequenceNumber, lastBlobCount, lastBlobX, lastBlobY);
    piDelay = millis() - piTimer;
    if (piDelay < 10) { delay(10 - piDelay); }
  } else if (cmdCmp(picoCommand, "PIDS") == 0) {  // respond with current pid factors
    // sequence is Proportional, Iterative and Derivative, as 2.2 decimal point numbers
    Serial.printf("%sOKOK%04d%04d%04d\r\n", sequenceNumber, factorP, factorI, factorD);
    piDelay = millis() - piTimer;
    if (piDelay < 10) { delay(10 - piDelay); }
  } else if (cmdProcessor(picoCommand, 5) == 0) {  // send to full command processor
    // only returns confirmation that command ok
    Serial.printf("%sOKOK\r\n", sequenceNumber);

  } else {  // assume its an unknown command
    Serial.printf("%sBADC - %s\r\n", sequenceNumber, picoCommand);
  }

  picoPinOn();
  if (debugSerial) { Serial.printf("%d millis\n", millis() - picoTimer); }
  return retVal;
}

void picoPinOn() {
  digitalWrite(picoPin, HIGH);
}

void picoPinOff() {
  digitalWrite(picoPin, LOW);
}
