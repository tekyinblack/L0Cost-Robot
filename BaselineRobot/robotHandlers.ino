// contains routines that perform robot and command processing functions
//
// process received data from web page, PS3, file or internal
/*- if in BASICSTA mode, serial passthru on, no handshake, Wifi client, web page input, local drive commands processed to drive pins
  - if in BASICAP mode, serial passthru on, no handshake, Wifi Access Point, web page input, local drive commands processed to drive pins
  - if in BASICPS3 mode, serial passthru on, no handshake, PS3 input, local drive commands processed to drive pins
  - if in REMOTESTA mode, serial passthru on, handshake on, Wifi client, web page input
  - if in REMOTEAP mode, serial passthru on, handshake on, Wifi Access Point, web page input
  - if in REMOTEPS3 mode, serial passthru on, handshake on, PS3 controller input
  - if in SERVOSTA mode, serial passthru on, no handshake, Wifi client, web page input, servo commands processed to 2 pins
  - if in SERVOAP mode, serial passthru on, no handshake, Wifi Access Point, web page input, servo commands processed to 2 pins
  - if in SERVOPS3 mode, serial passthru on, no handshake, PS3 controller input, servo commands processed to 2 pins
*/
//
// the source field indicates where the command came from
// 0 = internal   internal commands are executed irrespective of status
// 1 = web page   if a web page command fails, 500 message returned, the web page may resubmit
// 2 = file       if a file command fails, it waits and resubmits
// 3 = PS3        if a PS3 command fails, it is ignored
//
int cmdProcessor(char variable[], int source) {
  int retVal = 0;
  if (debugSerial) { Serial.printf("Command =  %s  %d \n", variable, source); }
  if (lockSet(cmdLock, source)) {
    return 2;
  }

  char sendChar[32] = {
    0,
  };
  // process remote processor commands by schedule sending
  if (variable[0] == 'X') {
    int i;
    for (i = 1; i < strlen(variable); i++) {
      sendChar[i - 1] = variable[i];
    }
    sendChar[i] = '\0';
    retVal = sendHandshake(sendChar);
  }
  // process command file by schedule opening
  else if (variable[0] == 'F') {
    int i;
    for (i = 1; i < strlen(variable); i++) {
      sendChar[i - 1] = variable[i];
    }
    sendChar[i] = '\0';
    // Serial.println(sendChar);
    // set file status to 1 , process file
    // put filename in file open format
  }
  // process local comand directly ie reset
  else if (variable[0] == 'L') {
    int i;
    for (i = 1; i < strlen(variable); i++) {
      sendChar[i - 1] = variable[i];
    }
    sendChar[i] = '\0';
    // Serial.println(sendChar);
    //shiftLeft(variable,1);
    retVal = localProcessor(sendChar);
  } else {
    retVal = 99;
  }
  lockUnSet(cmdLock, source);
  return retVal;
}
// initialise PWM for direct control from the board
void initPWM() {
  //Set up PWM for motor speed
  pinMode(rightMotorPin, OUTPUT);
  pinMode(leftMotorPin, OUTPUT);
  ledcSetup(rightMotorPWMSpeedChannel, PWMFreq, PWMResolution);
  ledcSetup(leftMotorPWMSpeedChannel, PWMFreq, PWMResolution);
  ledcAttachPin(rightMotorPin, rightMotorPWMSpeedChannel);
  ledcAttachPin(leftMotorPin, leftMotorPWMSpeedChannel);
  GPIO.func_out_sel_cfg[rightMotorPin].inv_sel = 1;
  GPIO.func_out_sel_cfg[leftMotorPin].inv_sel = 1;
}
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

// process local command
int localProcessor(char localVariable[]) {

  if (strcmp(localVariable, "FLASHPEEK") == 0) {
    // switch local flash LED on for limited time of 5 seconds, prevents it being left on
    flashState = 1;
    flashTimer = millis() + FLASH_PEEK_TIME;
    digitalWrite(FLASH_PIN, HIGH);
  } else if (strcmp(localVariable, "FLASHON") == 0) {
    // switch local flash LED on
    flashState = 1;
    flashTimer = FLASH_ON_TIME;
    digitalWrite(FLASH_PIN, HIGH);
  } else if (strcmp(localVariable, "FLASHOFF") == 0) {
    // switch local flash LED off
    flashState = 0;
    digitalWrite(FLASH_PIN, LOW);
  } else if (strcmp(localVariable, "TOGGLEFLASH") == 0) {
    // toggle local flash
    if (flashState) {
      flashState = 0;
      digitalWrite(FLASH_PIN, LOW);
    } else {
      flashState = 1;
      flashTimer = FLASH_ON_TIME;
      digitalWrite(FLASH_PIN, HIGH);
    }
  } else if (strcmp(localVariable, "RESET") == 0) {    // reboot local processor
  } else if (strcmp(localVariable, "HINVERT") == 0) {  // swap horizontal video stream
  } else if (strcmp(localVariable, "VINVERT") == 0) {  // swap vertical video stream
  } else if (strcmp(localVariable, "PAUSE") == 0) {    // pause processing for defined number of milliseconds
  }


  else if (motorControl(localVariable) && servoControl(localVariable)) {  // assume its a motor control command, and if not, unknown command
    Serial.print(localVariable);
    Serial.println(" - not known");
    if (debugSerial) { Serial.printf(" %s - not known\n", localVariable); }
  }
  return 0;
}
int servoControl(char localVariable[]) {
  //int servoControl(int servoNo, char command, int value) {
  int servoNo = 0;
  if (execServo == 0) return 1;
  if (localVariable[0] != 'S') return 1;
  if (localVariable[1] != '1') return 1;
  if (localVariable[2] == '2') {
    servoNo = 12;
  } else if (localVariable[2] == '3') {
    servoNo = 13;
  } else return 1;

  int value = 0;
  char valueChar[5] = { 0, 0, 0, 0, 0 };
  for (int i = 0; i < 4; i++) {
    valueChar[i] = localVariable[i + 4];
  }
  value = atoi(valueChar);

  char command = localVariable[3];

  switch (servoNo) {
    case 12:
      if (command == 'C') {
        servo12Angle = servo12Centre;
        servo12.write(servo12Angle);
      } else if (command == 'A') {
        if ((abs(value) > maxServo12Angle) || value < 0) return 2;
        servo12Angle = value;
      } else if (command == 'I') {
        servo12Angle = servo12Angle + value;
        if (servo12Angle < 0) servo12Angle = 0;
        if (servo12Angle > maxServo12Angle) servo12Angle = maxServo12Angle;
      } else return 3;
      if (debugSerial) { Serial.printf("Servo 12 Angle %d \n", servo12Angle); }      
      servo12.write(servo12Angle);
      break;
    case 13:
      if (command == 'C') {
        servo13Angle = servo13Centre;
        servo13.write(servo13Angle);
      } else if (command == 'A') {
        if ((abs(value) > maxServo13Angle) || value < 0) return 2;
        servo13Angle = value;
      } else if (command == 'I') {
        servo13Angle = servo13Angle + value;
        if (servo13Angle < 0) servo13Angle = 0;
        if (servo13Angle > maxServo13Angle) servo13Angle = maxServo13Angle;
      } else return 3;
      if (debugSerial) { Serial.printf("Servo 13 Angle %d \n", servo13Angle); }
      servo13.write(servo13Angle);
      break;
    default:
      return 1;
  }
  return 0;
}

int motorControl(char localVariable[]) {

  if (!execMotor) return 1;

  int driveValueLY = 0;
  int driveValueRY = 0;

  char driveCharLY[5] = { 0, 0, 0, 0, 0 };
  char driveCharRY[5] = { 0, 0, 0, 0, 0 };

  // command pattern
  //  MTR left values / right values
  // MTRxxxxyyyyXXXXYYYY
  if (localVariable[0] == 'M') {
    int i;
    for (i = 0; i < 4; i++) {
      driveCharLY[i] = localVariable[i + 7];
      driveCharRY[i] = localVariable[i + 15];
    }
    driveValueLY = atoi(driveCharLY);
    driveValueRY = atoi(driveCharRY);

    if (execMotor == 1) {
      // two pin motor control only has drive forward controls with tank steer
      if (driveValueLY < 0) driveValueLY = 0;
      if (driveValueRY < 0) driveValueRY = 0;
      ledcWrite(rightMotorPWMSpeedChannel, driveValueRY);
      ledcWrite(leftMotorPWMSpeedChannel, driveValueLY);
    } else if (execMotor == 2) {
      Serial.println("4 pin control not implemented yet");
    }

  } else if (strcmp(localVariable, "STOP") == 0) {
    // stop
    // if configured for local control mode
    // switch both motors off
    ledcWrite(rightMotorPWMSpeedChannel, 0);
    ledcWrite(leftMotorPWMSpeedChannel, 0);
  } else return 1;
  return 0;
}

int fileProcessor() {
  /* File command processor
  File is opened and contents fed through common processor routine
  open file
  read line
  send to command processor
  cycle until command processor indicates command executed
  if end of file, close file and wait again
check handshake - loop until handshake high - then send command
if in pause, loop until pause ended.
  
  */
  switch (scriptStatus) {
      // routine to read a data file and parse according to the type ----------------------------------
      //int read_datafile(fs::FS &fs, const char *path, int fileType) {

    case 0:
      // file processing not scheduled
      break;
    case 1:
      // file processing scheduled
      // open file
      // if file open ok, update status to 2, and file process to 0
      // if open fails, update status to 0

      if (debugSerial) { Serial.printf("Reading script: %s\n", scriptFile); }
      script = SD_MMC.open(scriptFile);
      if (!script) {
        if (debugSerial) { Serial.println("Failed to open script for reading"); }
        scriptStatus = 0;
      } else {
        scriptStatus = 2;
      }
      break;



    case 2:
      // file read process cycle in operation
      switch (scriptProcess) {
        case 0:
          // read next file line
          // if end of file, close file, update status to 0 and break
          // update file process to 1
          if (script.available()) {
            int l = script.readBytesUntil('\n', scriptBuf, sizeof(scriptBuf));
            if (l > 0 && scriptBuf[l - 1] == '\r') {
              l--;
            }
            scriptBuf[l] = 0;
            scriptProcess = 1;
          } else {
            if (debugSerial) { Serial.printf("Script: %s Completed \n", scriptFile); }
            script.close();
            scriptProcess = 0;
            scriptStatus = 0;
          }
          break;
        case 1:
          if (!cmdProcessor(scriptBuf, 2)) {
            scriptProcess = 2;
            break;
          }
          // handover command to command processor
          // if not accepted break
          // if accepted, update file process to 2
          break;
        case 2:
          // check if command processor now free, ie command complete
          // if so, update file process to 0
          break;
      }
  }
}

int shiftLeft(char *variable[], int step) {
  int retVal = 0;
  if (*variable[0] == '\0') return 1;
  int j = strlen(*variable);
  int k = j - step;
  if (k <= 0) return 2;
  int i;
  for (i = 0; i < k; i++) {
    *variable[i] = *variable[i + step];
  }
  *variable[i] = '\0';
  return 0;
}

int initServos(void) {
  // these dummies just provide a work around for the camera pins usage
  dummy1.setPeriodHertz(50);
  dummy2.setPeriodHertz(50);
  dummy1.attach(1, 1000, 2000);
  dummy2.attach(3, 1000, 2000);

  servo12.setPeriodHertz(50);  // Standard 50hz servo
  servo13.setPeriodHertz(50);  // Standard 50hz servo
  servo12.attach(12, servo12Min, servo12Max);
  servo13.attach(13, servo13Min, servo13Max);

  cmdProcessor("LS12C0000", 0);
  cmdProcessor("LS13C0000", 0);
  return 0;
}
