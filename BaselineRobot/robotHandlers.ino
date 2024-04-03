// contains routines that perform robot and command processing functions
//
// process received data from web page, PS3, file or internal
/*  In the BASIC modes, pins 12 and 13 are used to provide drive signals to transistors switching current to motors
    The standard mode is to drive PNP transistors
  - if in BASICSTA mode, serial passthru on, no handshake, Wifi client, web page input, local drive commands processed to drive pins
  - if in BASICAP mode, serial passthru on, no handshake, Wifi Access Point, web page input, local drive commands processed to drive pins
  - if in BASICPS3 mode, serial passthru on, no handshake, PS3 input, local drive commands processed to drive pins
    The plus modes are to drive NPN transistors
  - if in BASICSTA+ mode, serial passthru on, no handshake, Wifi client, web page input, local drive commands processed to drive pins as positive
  - if in BASICAP+ mode, serial passthru on, no handshake, Wifi Access Point, web page input, local drive commands processed to drive pins as positive
  - if in BASICPS3+ mode, serial passthru on, no handshake, PS3 input, local drive commands processed to drive pins as positive

  - For SOKOBAN modes, drive pins are 3 and 12 (normally have reserved roles) and 13 is PWM output. This is a specialised robot role with minimal configuration
  - if in SOKOBANSTA mode, serial passthru off, no handshake, Wifi client, web page input, alternative drive pins, L293D decode
  - if in SOKOBANAP mode, serial passthru off, no handshake, Wifi Access Point, web page input, alternative drive pins, L293D decode
  - if in SOKOBANPS3 mode, serial passthru off, no handshake, PS3 input, alternative drive pins, L293D decode

  - if in REMOTESTA mode, serial passthru on, handshake on, Wifi client, web page input
  - if in REMOTEAP mode, serial passthru on, handshake on, Wifi Access Point, web page input
  - if in REMOTEPS3 mode, serial passthru on, handshake on, PS3 controller input
  
  - The SERVO modes make it easy to add a Pan and Tilt function to some of the other L0Cost robot builds
  - if in SERVOSTA mode, serial passthru on, no handshake, Wifi client, web page input, servo commands processed to pins 12 and 13
  - if in SERVOAP mode, serial passthru on, no handshake, Wifi Access Point, web page input, servo commands processed pins 12 and 13
  - if in SERVOPS3 mode, serial passthru on, no handshake, PS3 controller input, servo commands processed pins 12 and 13
  */




// *********************************************************************************************
// initialise pins for three pin operation, left and right drive with PWM on a single pin
// *********************************************************************************************
void initThreePin(void) {
  // setup for three pin mode used for Sokoban robots
  // change left motor pin
  leftMotorPin = leftMatrixMotorPin;
  // left and right pins are only logic 0 and 1
  pinMode(rightMotorPin, OUTPUT);
  pinMode(leftMotorPin, OUTPUT);
  // enable pin is setup for PWM
  ledcSetup(enableMotorPWMSpeedChannel, PWMFreq, PWMResolution);
  pinMode(enableMotorPin, OUTPUT);
  ledcAttachPin(enableMotorPin, enableMotorPWMSpeedChannel);
  execPWM = 1;  // flag pwm attached
}

// *********************************************************************************************
// initialise PWM for direct control from the controller
// *********************************************************************************************
void initPWM() {
  //Set up PWM for motor speed control
  ledcSetup(rightMotorPWMSpeedChannel, PWMFreq, PWMResolution);
  ledcSetup(leftMotorPWMSpeedChannel, PWMFreq, PWMResolution);
  attachPWM();  // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Remove when line follower hardware fixed
}

// *********************************************************************************************
// detach PWM for direct control from the board (typically line follower circuit)
// *********************************************************************************************
//  NOT CURRENTLY REQUIRED UNTIL HARDWARE FIX FOR LINE FOLLOWER RESOLVED
// *********************************************************************************************
void detachPWM() {
  // change pin usage to input to allow local motor control
  return;       // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Remove when line follower hardware fixed
  execPWM = 0;  // flag pwm detached
  //ledcWrite(rightMotorPWMSpeedChannel, 0);
  //ledcWrite(leftMotorPWMSpeedChannel, 0);
  ledcDetachPin(leftMotorPin);
  ledcDetachPin(rightMotorPin);
  //GPIO.func_out_sel_cfg[rightMotorPin].inv_sel = 0;
  //GPIO.func_out_sel_cfg[leftMotorPin].inv_sel = 0;
  pinMode(rightMotorPin, INPUT);
  pinMode(leftMotorPin, INPUT);
  if (debugSerial) { Serial.println("PWM Detached"); }
}

// *********************************************************************************************
// initialise PWM for direct control from the controller
// *********************************************************************************************
void attachPWM() {
  //Set up PWM for motor speed
  pinMode(rightMotorPin, OUTPUT);
  pinMode(leftMotorPin, OUTPUT);
  ledcAttachPin(rightMotorPin, rightMotorPWMSpeedChannel);
  ledcAttachPin(leftMotorPin, leftMotorPWMSpeedChannel);
  // if negative polarity set then invert pin polarity
  if (!execPolarity) {
    GPIO.func_out_sel_cfg[rightMotorPin].inv_sel = 1;
    GPIO.func_out_sel_cfg[leftMotorPin].inv_sel = 1;
  }
  execPWM = 1;                             // flag pwm attached
  directTimer = millis() + directTimeOut;  // reset direct timeout timer
  if (debugSerial) { Serial.println("PWM Attached"); }
}

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
int remoteProcessor(char commsString[]) {
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



// *********************************************************************************************
// getValue - service routine to get numeric value from command
// *********************************************************************************************
int getValue(char fullCommand[], int offset) {
  char temp[6] = { 0, 0, 0, 0, 0, 0 };
  char length = strlen(fullCommand);
  if ((length - offset >= 6) || (length - offset <= 0)) return 0;
  for (int i = offset; i < length; i++) {
    temp[i - offset] = fullCommand[i];
  }
  return atoi(temp);
}

// *********************************************************************************************
// cmdCmp - service routine to specifically test for robot commands
// *********************************************************************************************
int cmdCmp(char fullCommand[], char testCommand[]) {
  int full = strlen(fullCommand);
  int test = strlen(testCommand);
  if (test > full) return 1;
  for (int i = 0; i < test; i++) {
    if (fullCommand[i] != testCommand[i]) return 1;
  }
  return 0;
}




// *********************************************************************************************
// activate video streamimg mode
// *********************************************************************************************
int setVideoMode() {
  execVideo = 1;
  execGuide = 0;
  return 0;
}
// *********************************************************************************************
// activate video guidance mode
// *********************************************************************************************
int setGuideMode() {
  execVideo = 0;
  execGuide = 1;
  return 0;
}
// *********************************************************************************************
// servo command processing
// *********************************************************************************************
int servoControl(char localVariable[]) {
  // process servo command LSxxQ
  //   S indicates that this is a servo command
  //   xx = gpio 12 or 13, the pin the servo is attached to
  //   Q is one of the following command letters
  //
  //   C indicates that the servo moves to the set default centre position (set as 90 in the code)
  //     command layout LSxxC
  //
  //   A indicates that the servo moves to an absolute position indicated by the value zzzz
  //     command layout LSxxAzzzz
  //
  //   I indicates that the servo angle will be adjusted by adding the value zzzz to the current servo angle
  //     command layout LSxxIzzzz
  //
  //   D indicates a default value being processed
  //     command layout LSxxQyzzzzaaaa
  //   y can be one of M, where zzzz represents the default centre position
  //                   X, where zzzz represents the default maximum servo angle
  //                   T, where zzzz represnts the signal tiing for zero degress rotation
  //                         and aaaa is the signal timing for maximum rotation
  //
  int servoNo = 0;
  int value = 0;
  int value2 = 0;
  char valueChar[5] = { 0, 0, 0, 0, 0 };
  if (execServo == 0) return 1;
  if (localVariable[0] != 'S') return 1;
  if (localVariable[1] != '1') return 1;
  if (localVariable[2] == '2') {
    servoNo = 12;
  } else if (localVariable[2] == '3') {
    servoNo = 13;
  } else return 1;
  char command = localVariable[3];
  if (command == 'D') {
    // process defaults
    command = localVariable[4];
    if (command == 'M' || command == 'X') {
      // process single default value for centre or maximum
      // get the  number
      for (int i = 0; i < 4; i++) {
        valueChar[i] = localVariable[i + 5];
      }
      value = atoi(valueChar);
    } else if (command == 'T') {
      // process two values for timing      // get the  number
      for (int i = 0; i < 4; i++) {
        valueChar[i] = localVariable[i + 5];
      }
      value = atoi(valueChar);
      // get the  number
      for (int i = 0; i < 4; i++) {
        valueChar[i] = localVariable[i + 9];
      }
      value2 = atoi(valueChar);
    }
  } else if (command == 'C') {
    // we dont care
  } else if (command == 'A' || command == 'I') {
    // get the  number

    for (int i = 0; i < 4; i++) {
      valueChar[i] = localVariable[i + 4];
    }
    value = atoi(valueChar);
  } else {
    if (debugSerial) { Serial.println("Command unknown"); }
    return 1;
  }


  switch (servoNo) {
    case 12:
      if (command == 'C') {
        servo12Angle = servo12Centre;
        servo12.write(servo12Angle);
      } else if (command == 'A') {
        if ((abs(value) > maxServo12Angle) || value < 0) return 2;
        servo12Angle = value;
        servo12.write(servo12Angle);
      } else if (command == 'I') {
        servo12Angle = servo12Angle + value;
        if (servo12Angle < 0) servo12Angle = 0;
        if (servo12Angle > maxServo12Angle) servo12Angle = maxServo12Angle;
        servo12.write(servo12Angle);
      } else if (command == 'M') {
        servo12Centre = value;
      } else if (command == 'X') {
        maxServo12Angle = value;
      } else if (command == 'T') {
        servo12Min = value;
        servo12Max = value2;
      } else return 3;
      if (debugSerial) { Serial.printf("Servo 12 Angle %d \n", servo12Angle); }

      break;
    case 13:
      if (command == 'C') {
        servo13Angle = servo13Centre;
        servo13.write(servo13Angle);
      } else if (command == 'A') {
        if ((abs(value) > maxServo13Angle) || value < 0) return 2;
        servo13Angle = value;
        servo13.write(servo13Angle);
      } else if (command == 'I') {
        servo13Angle = servo13Angle + value;
        if (servo13Angle < 0) servo13Angle = 0;
        if (servo13Angle > maxServo13Angle) servo13Angle = maxServo13Angle;
        servo13.write(servo13Angle);
      } else if (command == 'M') {
        servo13Centre = value;
      } else if (command == 'X') {
        maxServo13Angle = value;
      } else if (command == 'T') {
        servo13Min = value;
        servo13Max = value2;
      } else return 3;
      if (debugSerial) { Serial.printf("Servo 13 Angle %d \n", servo13Angle); }
      break;
    default:
      return 1;
  }
  return 0;
}

// *********************************************************************************************
// Interpret and execute motor control on pins 3, 12 and 13
// *********************************************************************************************
int motorControl(char localVariable[], int source) {
  /*
  process motor control LMTR command pattern
  this can take two forms, one is general, and one specifically from a PS3 controller
  From a general source

   MTR left values / right values / runtime
  the left and right values are from 0-255
  the runtime is from 0-9999 and indcates the length of time in milliseconds the motor value is active for.
  a runtime of 0 does not timeout
  MTRxxxxyyyytttt
  or STOP command which zero's all channels

  From a PS3
  MTR leftx/lefty/rightx/righty

  in standard steering
  leftx is ignored
  left y is taken as power
  rightx is taken as direction
  righty is ignored

  in tank steering
  leftx is ignored
  lefty is left motor
  rightx is ignored
  righty is right motor

  three pin control converts any non-zero left/right value to a logic 1, and uses
  the highest of the left/right values as the PWM value for the enable pin

  The STOP command is also processed here as is the MTRD, set motor default command

  STOP sets all PWM output immediately to zero, this is more of an emergency stop rather than a controlled slow down

  MTRDxxxxyyyy sets up scaling factors for left and right motor output. The default for these is 255 and the resultant
  motor power output is command input * scaling factor/255. This allows the motrs to be balanced where one might be 
  slightly more powerful than the other leading to an imbalance in movement
  */

  if (!execMotor) return 1;  // not processing motor commands
  if (!execPWM) {
    attachPWM();
  }

  int driveValueLY = 0;
  int driveValueRY = 0;
  int runTimeValue = 0;
  int enableDrive = 0;

  char driveCharLY[5] = { 0, 0, 0, 0, 0 };
  char driveCharRY[5] = { 0, 0, 0, 0, 0 };
  char runTimeChar[5] = { 0, 0, 0, 0, 0 };



  if (cmdCmp(localVariable, "MTRD") == 0) {
    for (int i = 0; i < 4; i++) {
      driveCharLY[i] = localVariable[i + 4];
      driveCharRY[i] = localVariable[i + 8];
    }
    driveValueLY = atoi(driveCharLY);
    driveValueRY = atoi(driveCharRY);
  } else if (cmdCmp(localVariable, "MTR") == 0) {
    if (source == 3) {  // command is from PS3 so requires different processing for local control
      for (int i = 0; i < 4; i++) {
        driveCharLY[i] = localVariable[i + 7];
        driveCharRY[i] = localVariable[i + 15];
      }
      driveValueLY = -atoi(driveCharLY);
      driveValueRY = -atoi(driveCharRY);
      runTimeValue = 0;
    } else {
      for (int i = 0; i < 4; i++) {
        driveCharLY[i] = localVariable[i + 3];
        driveCharRY[i] = localVariable[i + 7];
        runTimeChar[i] = localVariable[i + 11];
      }
      driveValueLY = atoi(driveCharLY);
      driveValueRY = atoi(driveCharRY);
      runTimeValue = atoi(runTimeChar);
    }


    if (execMotor == 1) {
      // two pin motor control only has drive forward controls with tank steer
      // negative values are therefor zeroed
      if (driveValueLY < 0) driveValueLY = 0;
      if (driveValueRY < 0) driveValueRY = 0;
      if (runTimeValue < 0) runTimeValue = 0;
      ledcWrite(rightMotorPWMSpeedChannel, driveValueRY);
      ledcWrite(leftMotorPWMSpeedChannel, driveValueLY);

    } else {
      enableDrive = abs(driveValueLY);
      if (driveValueLY <= 0) {
        driveValueLY = 0;
      } else {
        driveValueLY = 1;
      }
      if (abs(driveValueRY) > enableDrive) {
        enableDrive = abs(driveValueRY);
      }
      if (driveValueRY <= 0) {
        driveValueRY = 0;
      } else {
        driveValueRY = 1;
      }
      ledcWrite(enableMotorPWMSpeedChannel, enableDrive);
      digitalWrite(leftMotorPin, driveValueLY);
      digitalWrite(rightMotorPin, driveValueRY);
      if (debugSerial) { Serial.print("Write digital"); }
    }
    if (runTimeValue == 0) {
      motorTimer = millis() + 10000;  // default timeout is 10 seconds
    } else motorTimer = millis() + runTimeValue;
    directTimer = millis() + directTimeOut;  // reset direct timeout timer
    if (debugSerial) { Serial.printf("Motor left - %d Motor right - %d Timer - %d\n", driveValueLY, driveValueRY, runTimeValue); }
    if (scriptStatus) {
      scriptTimer = millis() + runTimeValue;
      if (debugSerial) { Serial.printf("Pausing for %d milliseconds \n", runTimeValue); }
      scriptProcess = 3;
    }
  } else if (strcmp(localVariable, "STOP") == 0) {
    // stop
    // if configured for local control mode
    // switch both motors off
    motorTimer = 0;
    if (debugSerial) { Serial.println("Motors stopping"); }
    if (execMotor == 1) {
      ledcWrite(rightMotorPWMSpeedChannel, 0);
      ledcWrite(leftMotorPWMSpeedChannel, 0);
    } else if (execMotor == 2) {
      ledcWrite(enableMotorPWMSpeedChannel, 0);
    }
  } else return 1;
  return 0;
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

  if (!execSerial) return 1;

  // read serial input until end of
  if (Serial.available()) {
    char nextChar = Serial.read();
    if (nextChar == '\n') {
      commandCount++;
      if (execPico) {   // if comms with PiWars Pico processor being done, call to picoProcessor made to handle commands
        picoProcessor(serialCommand, 4);
      } else {
        cmdProcessor(serialCommand, 4);
      }
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

// *********************************************************************************************
// file processor schedules commands from script files
// *********************************************************************************************
void fileProcessor(int type) {
  /* File command processor
  type is either 0 for startup script, or 1 for normal script. PAUSE and REPEAT ignored in Startup 
  File is opened and contents fed through common processor routine
  open file
  read line
  send to command processor
  cycle until command processor indicates command executed
  if end of file, close file and wait again
check handshake - loop until handshake high - then send command
if in pause, loop until pause ended.
  
  */
  int result = 0;
  char fileName[33] = "/";

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
      if (scriptFile[0] != '/') {
        for (int i = 1; i <= 32; i++) {
          fileName[i] = scriptFile[i - 1];
        }
      } else strcpy(fileName, scriptFile);

      if (debugSerial) { Serial.printf("Reading script: %s\n", fileName); }
      script = SD_MMC.open(fileName);
      if (!script) {
        if (debugSerial) { Serial.println("Failed to open script for reading"); }
        scriptStatus = 0;
      } else {
        scriptStatus = 2;
        scriptProcess = 0;
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
            // read file line until linefeed found or buffer is full
            int l = script.readBytesUntil('\n', scriptBuf, sizeof(scriptBuf));
            // if carriage return included, reduce command length by 1
            if (l > 0 && scriptBuf[l - 1] == '\r') {
              l--;
            }
            // check for lines starting with blank or //. These are ignored.
            if (scriptBuf[0] == ' ' || (scriptBuf[0] == '/' && scriptBuf[1] == '/')) {
              scriptProcess = 0;
            } else {
              scriptBuf[l] = 0;
              scriptProcess = 1;
            }
          } else {
            if (debugSerial) { Serial.printf("Script: %s Completed \n", scriptFile); }
            scriptProcess = 4;
          }
          break;
        case 1:
          result = cmdProcessor(scriptBuf, 2);
          // if the script process state has been changed then command executed ok
          if (scriptProcess != 1) {
            break;
          } else if (result <= 91) {
            scriptProcess = 2;
          }
          // handover command to command processor
          // if not accepted break
          // if accepted, update file process to 2
          break;
        case 2:
          // check if command processor now free, ie command complete
          // if so, update file process to 0
          if (lockTest(cmdLock, 2) == 0) {
            scriptProcess = 0;
          }
          break;
        case 3:
          // this is a do nothing case where the script processing is paused
          // it isn't necessary but has been incuded so that if actions are needed then they can be added
          break;
        case 4:
          // end of file, close file and reset script processing
          script.close();
          scriptProcess = 0;
          scriptStatus = 0;
          break;
        case 5:
          // repeat script processing, close file and set script processing to start again
          script.close();
          scriptProcess = 0;
          scriptStatus = 1;
          break;
      }
  }
}

// *********************************************************************************************
// shiftLeft - service routine to shift command string left by requested number of steps
// *********************************************************************************************
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

// *********************************************************************************************
// initialise servo pins 12 and 13 usage on an esp32
// *********************************************************************************************
int initServos(void) {
  // these dummies just provide a work around for the pwm pins usage
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
