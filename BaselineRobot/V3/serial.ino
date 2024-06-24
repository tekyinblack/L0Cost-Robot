// *********************************************************************************************
// The following routines are to implement hardware handshaking between this ESP32 and another
// robot controller card, typically an Arduino or Pico
// *********************************************************************************************
int sendHandshake(char commsString[]) {
  // check if this routine is supposed to be used
  if (!execHandshake || !execSerial) {
    return 2;
  }
  // if rtr flag set then send data, else return failure
  if (digitalRead(rtrRemote)) {
    Serial.println(commsString);
    return 0;
  } else {
    return 1;
  }
}
// *********************************************************************************************
int remoteProcessor(char commsString[], int source) {
  // handshake not implemented yet
  if (execHandshake) {
    return 2;
  }
  if (execSerial) {
    Serial.println(commsString);
    return 0;
  }
  return 1;
}
int serialProcessor(int type) {
  /*
  Reads data from the serial connection as commands
  Commands cannot be greater than 33 characters long in this definition and must
  be terminated with a \n character
  return codes
  1 = serial not active
  2 = serial data not available
  3 = serial data too long (rest of input not ignored but processing continued)
  4 = consecutive command limit exceeded (limits number of serial commands accepted before other processing)
  */
  int retVal = 0;
  if (!execSerial) return 1;

  // read serial input until end of
  if (Serial.available()) {
    char nextChar = Serial.read();
    if (nextChar == '\n') {
      commandCount++;

      retVal = cmdProcessor(serialCommand, 4);

      serialPointer = 0;  // reset serial command pointer
    } else {
      serialCommand[serialPointer] = nextChar;
      serialCommand[serialPointer + 1] = '\0';
      serialPointer++;
      if (serialPointer >= serialLimit) {
        // input exceeds command limit, input disgarded
        serialPointer = 0;
        serialCommand[0] = '\0';
        commandCount = 0;  // reset command count
        return 3;
      }
    }
  } else {
    commandCount = 0;  // reset command count
    return 2;          // serial data not available
  }
  // check that consecutive command count not exceeded
  if (commandCount >= commandLimit) {
    commandCount = 0;  // reset command count
    return 4;
  }
  return 0;
}