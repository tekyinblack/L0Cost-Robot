// contains routines that perform motor control
//
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
  //ledcSetup(enableMotorPWMSpeedChannel, PWMFreq, PWMResolution);
  pinMode(enableMotorPin, OUTPUT);
  //ledcAttachPin(enableMotorPin, enableMotorPWMSpeedChannel);
  ledcAttach(enableMotorPin, PWMFreq, PWMResolution);
  execPWM = 1;  // flag pwm attached
}

// *********************************************************************************************
// initialise PWM for direct control from the controller
// *********************************************************************************************
void initPWM() {
  //Set up PWM for motor speed control
  //ledcSetup(rightMotorPWMSpeedChannel, PWMFreq, PWMResolution);
  //ledcSetup(leftMotorPWMSpeedChannel, PWMFreq, PWMResolution);

  pinMode(rightMotorPin, OUTPUT);
  pinMode(leftMotorPin, OUTPUT);

  //ledcAttachPin(rightMotorPin, rightMotorPWMSpeedChannel);
  //ledcAttachPin(leftMotorPin, leftMotorPWMSpeedChannel);
   ledcAttach(rightMotorPin,PWMFreq, PWMResolution);
  ledcAttach(leftMotorPin, PWMFreq, PWMResolution);

  // if negative polarity set to invert pin polarity
  if (!execPolarity) {
    ledcOutputInvert(rightMotorPin,true);
    ledcOutputInvert(leftMotorPin,true);
  // GPIO.func_out_sel_cfg[rightMotorPin].inv_sel = 1;
   // GPIO.func_out_sel_cfg[leftMotorPin].inv_sel = 1;
  }
  execPWM = 1;                             // flag pwm attached
  directTimer = millis() + directTimeOut;  // reset direct timeout timer
  if (debugSerial) { Serial.println("PWM Attached"); }
}
// *********************************************************************************************
// process camera only commands - this is primarily to offload code complexity from the
// local command handler routine
// *********************************************************************************************
int motorControl(char localVariable[], int source) {
  int retVal = 0;
  char motorCommand[33];

  // left shift command
  retVal = motorControl2(localVariable, source);
  return retVal;


  int i = 0;
  while (localVariable[i] != 0) {
    motorCommand[i] = localVariable[i + 1];
    i++;
    if (i > 20) break;
  }
  if (cmdCmp(motorCommand, "VIDEO") == 0) {
    // activate video streamimg mode
    retVal = setVideoMode();

  } else if (cmdCmp(motorCommand, "RAWGMA") == 0) {
    // activate gamma correction 0,1
    //retVal = s->set_raw_gma(s, getValue(motorCommand, 6));
  } else if (cmdCmp(motorCommand, "LENC") == 0) {
    // activate lens correction correction 0,1
    //retVal = s->set_lenc(s, getValue(motorCommand, 4));
  } else if (cmdCmp(motorCommand, "DCW") == 0) {
    // activate direct conversion 0,1
    //retVal = s->set_dcw(s, getValue(motorCommand, 3));
  } else if (cmdCmp(motorCommand, "LUMINANCE") == 0) {
    // activate direct conversion 0,1
    Serial.printf("%04d,%08d\r\n", pixelLuminance(), millis());

  } else if (debugSerial) {
    if (debugSerial) { Serial.printf(" %s - not known\n", motorCommand); }
  }


  return 0;
}

// *********************************************************************************************
// Interpret and execute motor control on pins 3, 12 and 13
// *********************************************************************************************
int motorControl2(char localVariable[], int source) {
  /*
  process motor control LMTR command pattern
  this can take two forms, one is general, and one specifically from a PS3 controller


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
      // otherwise assume matrix motor control
      // drive power value taken as greater of left or right drive value
      // sign determines direction set by matrix as 1= positive, 0 = negative
      // stop is only set by power value
      enableDrive = max(abs(driveValueLY), abs(driveValueRY));
      if (driveValueLY > 0 && driveValueRY > 0) {
        driveValueLY = 0;
        driveValueRY = 0;
      } else if (driveValueLY <= 0 && driveValueRY <= 0) {
        driveValueLY = 1;
        driveValueRY = 1;
      } else if (driveValueLY <= 0 && driveValueRY > 0) {
        driveValueLY = 0;
        driveValueRY = 1;
      } else if (driveValueLY > 0 && driveValueRY <= 0) {
        driveValueLY = 1;
        driveValueRY = 0;
      }
      ledcWrite(enableMotorPWMSpeedChannel, enableDrive);
      digitalWrite(leftMotorPin, driveValueLY);
      digitalWrite(rightMotorPin, driveValueRY);
      if (debugSerial) { Serial.print("Write digital"); }
    }
    if (runTimeValue == 0) {
      motorTimer = millis() + 10000;  // default timeout is 10 seconds
    } else motorTimer = millis() + runTimeValue;
    // if timed motor run, then set timeout counter
    directTimer = millis() + directTimeOut;  // reset direct timeout timer
    if (debugSerial) { Serial.printf(" Power - %d Motor left - %d Motor right - %d Timer - %d\n", enableDrive, driveValueLY, driveValueRY, runTimeValue); }
    if (scriptStatus) {
      // if running a script then set the script to pause for same time as motor time to avoid overlap
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
