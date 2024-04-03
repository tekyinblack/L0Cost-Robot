/*
This section only contains routines that process commands. It has been separated out to 
enable future restructuring into a more object orientated solution.
*/
/*
*********************************************************************************************
Command processor routine - all commands routed via this code
*********************************************************************************************
the source field indicates where the command came from
0 = internal   internal commands are executed irrespective of status
1 = web page   if a web page command fails, 500 message returned, the web page may resubmit
2 = file       if a file command fails, it waits and resubmits
3 = PS3        if a PS3 command fails, it is ignored
4 = serial     if a serial port command fails, NOEXEC message returned

Command prefixes are 
L - local command for execution by this controller
LC - local command specifically for a camera if fitted
X - remote command, routed to the serial port
F - file command, executes the script in the named file 
H - where appropriate, send the command to the webpage as an unbound JSON string
P - command for PS3 controller 

return
0 command processed ok
91 file processing in progress
92 couldn't get lock
*/
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
      if (i > 30) break;              // break if longer than allowed
      if (variable[i] == ' ') break;  // break if blank character
      sendChar[i - 1] = variable[i];
    }
    sendChar[i] = '\0';
    retVal = remoteProcessor(sendChar);
  }
  // process command file by schedule opening
  else if (variable[0] == 'F') {
    int i, j;
    variable[0] = '/';
    if (scriptStatus) {
      if (debugSerial) { Serial.printf("File not processed, currently processing %s", scriptFile); }
      retVal = 1;
    } else {
      sendChar[i] = '\0';
      strcpy(scriptFile, variable);
      scriptStatus = 1;  // begin script processing
    }
  }
  // process local comand directly ie reset
  else if (variable[0] == 'L') {
    int i;
    for (i = 1; i < strlen(variable); i++) {
      sendChar[i - 1] = variable[i];
    }
    sendChar[i] = '\0';
    retVal = localProcessor(sendChar, source);
  }
  // process local comand directly ie reset
  else if (variable[0] == 'H') {
    int i;
    for (i = 1; i < strlen(variable); i++) {
      sendChar[i - 1] = variable[i];
    }
    sendChar[i] = '\0';
    retVal = htmlProcessor(sendChar, source);
  }
  // process local comand directly ie reset
  else if (variable[0] == 'P') {
    int i;
    for (i = 1; i < strlen(variable); i++) {
      sendChar[i - 1] = variable[i];
    }
    sendChar[i] = '\0';
    retVal = ps3Processor(sendChar, source);
  } else if (strcmp(variable, "0000PICO") == 0) {  // turn on PiWars Pico mode.
    execPico = 1;
    pinMode(picoPin, OUTPUT);
    picoPinOn();
    Serial.println("0000OKOK");
  } else {
    retVal = 90;
  }
  lockUnSet(cmdLock, source);
  if (source == 1) {
    return 0;
  }
  return retVal;
}

// *********************************************************************************************
// process local command - if command prefixed by L, come here
// *********************************************************************************************
int localProcessor(char localVariable[], int source) {
  int retVal = 0;
  int pauseValue = 0;
  // test if it starts with C and is thus a camera command
  if (localVariable[0] == 'C') {
    cameraControl(localVariable);
  } else if (localVariable[0] == 'G') {
    // test if it starts with G and is thus a guidance command
    guideControl(localVariable);
  } else if (strcmp(localVariable, "DEBUGOFF") == 0) {
    // switch debug off
    debugSerial = 0;
  } else if (strcmp(localVariable, "DEBUGON") == 0) {
    // switch debug on
    debugSerial = 1;
      } else if (strcmp(localVariable, "WIFION") == 0) {
    // sswitch wifi on , only applicable during startup
    execWiFi = 1;
  } else if (strcmp(localVariable, "WIFIOFF") == 0) {
    // switch wifi off , only applicable during startup
    execWiFi = 0;
  } else if (localVariable[0] == 'D') {
    // test if it starts with D and is thus a demo command
    demoControl(localVariable);
  } else if (strcmp(localVariable, "FLASHPEEK") == 0) {
    // switch local flash LED on for limited time of 5 seconds, prevents it being left on
    flashState = 1;
    flashTimer = millis() + FLASH_PEEK_TIME;
    // digitalWrite(FLASH_PIN, HIGH);
    ledcWrite(flashPWMChannel, flashIntensity);
  } else if (strcmp(localVariable, "FLASHON") == 0) {
    // switch local flash LED on
    flashState = 1;
    flashTimer = FLASH_ON_TIME;
    //digitalWrite(FLASH_PIN, HIGH);
    ledcWrite(flashPWMChannel, flashIntensity);
  } else if (strcmp(localVariable, "FLASHLEVEL") == 0) {
    // set flash LED intensity level 0-255 max
    flashIntensity = getValue(localVariable, 10);
    // if level invalid then set to default
    if (flashIntensity > 255 || flashIntensity < 0) {
      flashIntensity = 128;
    }
  } else if (strcmp(localVariable, "FLASHOFF") == 0) {
    // switch local flash LED off
    flashState = 0;
    //digitalWrite(FLASH_PIN, LOW);
    ledcWrite(flashPWMChannel, 0);
  } else if (strcmp(localVariable, "TOGGLEFLASH") == 0) {
    // toggle local flash
    if (flashState) {
      flashState = 0;
      //digitalWrite(FLASH_PIN, LOW);
      ledcWrite(flashPWMChannel, 0);
    } else {
      flashState = 1;
      flashTimer = FLASH_ON_TIME;
      //digitalWrite(FLASH_PIN, HIGH);
      ledcWrite(flashPWMChannel, flashIntensity);
    }

  } else if (strcmp(localVariable, "RESET") == 0) {  // reboot local processor
    ESP.restart();
  } else if (strcmp(localVariable, "PICO") == 0) {  // turn on PiWars Pico mode.
    execPico = 1;
  } else if (cmdCmp(localVariable, "REPEAT") == 0) {  // if script processing in operation, close and reopen script file
    // this is used to run a script continuously
    scriptProcess = 5;
  } else if (cmdCmp(localVariable, "PAUSE") == 0) {  // pause processing for defined number of milliseconds
    //get PAUSE value and add it to current cpu time, then set script processing to pause.
    pauseValue = getValue(localVariable, 5);
    scriptTimer = millis() + pauseValue;
    if (debugSerial) { Serial.printf("Pausing for %d milliseconds \n", pauseValue); }
    scriptProcess = 3;
  } else if (cmdCmp(localVariable, "MTRATTACH") == 0) {  // initialise motor processing
    // execute routine to initialise PWM to the motor control pins
    attachPWM();
  } else if (cmdCmp(localVariable, "MTRDETACH") == 0) {  // de-initialise motor processing
    // execute routine to turn off PWM to motor control pins and make them input
    detachPWM();
  } else if (cmdCmp(localVariable, "MTRTIMEOUT") == 0) {  // motor processing timeout
                                                          // timing routine to automatically deinitialise the PWM pins after number of milliseconds
    directTimeOut = getValue(localVariable, 10);
    if (directTimeOut < 0) {
      directTimeOut = defaultTimeOut;
    } else if (directTimeOut == 0) {
      directTimeOut = 9999999;  // if set as zero the set to 999 seconds
    }
    if (debugSerial) { Serial.printf("Timeout set to - %d\n", directTimeOut); }
  } else if (motorControl(localVariable, source) && servoControl(localVariable)) {  // assume its a motor control command, and if not, unknown command
    Serial.print(localVariable);
    Serial.println(" - not known");
    if (debugSerial) { Serial.printf(" %s - not known\n", localVariable); }
  }
  return retVal;
}

// *********************************************************************************************
// process camera only commands - this is primarily to offload code complexity from the
// local command handler routine
// *********************************************************************************************
int cameraControl(char localVariable[]) {
  int retVal = 0;
  char cameraCommand[33];

  // left shift command
  int i = 0;
  while (localVariable[i] != 0) {
    cameraCommand[i] = localVariable[i + 1];
    i++;
    if (i > 20) break;
  }
  if (cmdCmp(cameraCommand, "VIDEO") == 0) {
    // activate video streamimg mode
    retVal = setVideoMode();
  } else if (cmdCmp(cameraCommand, "GUIDE") == 0) {
    // activate video guidance mode
    retVal = setGuideMode();
  }
  if (execVideo || execGuide) {
    sensor_t *s = esp_camera_sensor_get();

    if (cmdCmp(cameraCommand, "HMIRROR") == 0) {
      // swap horizontal video stream 0,1
      retVal = s->set_hmirror(s, getValue(cameraCommand, 7));

    } else if (cmdCmp(cameraCommand, "VFLIP") == 0) {
      // swap vertical video stream 0,1
      retVal = s->set_vflip(s, getValue(cameraCommand, 5));

    } else if (cmdCmp(cameraCommand, "FRAMESIZE") == 0) {
      // change stream size
      // 10 UXGA(1600x1200)
      // 9  SXGA(1280x1024)
      // 8  XGA(1024x768)
      // 7  SVGA(800x600)
      // 6  VGA(640x480)
      // 5  CIF(400x296)
      // 4  QVGA(320x240)
      // 3  HQVGA(240x176)
      // 0  QQVGA(160x120)
      if (s->pixformat == PIXFORMAT_JPEG && execGuide == 0) {  // if video guidance in operation don't allow changes to frame size
        // res = s->set_framesize(s, (framesize_t)val);
        retVal = s->set_framesize(s, (framesize_t)getValue(cameraCommand, 9));
      }
    } else if (cmdCmp(cameraCommand, "QUALITY") == 0) {
      // change jpeg quality 4-64
      retVal = s->set_quality(s, getValue(cameraCommand, 7));

    } else if (cmdCmp(cameraCommand, "BRIGHT") == 0) {
      // adjust brightness -2,-1,0,1,2
      retVal = s->set_brightness(s, getValue(cameraCommand, 6));

    } else if (cmdCmp(cameraCommand, "CONTRAST") == 0) {
      // adjust contrast -2,-1,0,1,2
      retVal = s->set_contrast(s, getValue(cameraCommand, 8));

    } else if (cmdCmp(cameraCommand, "SAT") == 0) {
      // adjust saturation -2,-1,0,1,2
      retVal = s->set_saturation(s, getValue(cameraCommand, 3));

    } else if (cmdCmp(cameraCommand, "EFFECT") == 0) {
      // apply effect 0,1,2,3,4,5,6
      retVal = s->set_special_effect(s, getValue(cameraCommand, 6));
    } else if (cmdCmp(cameraCommand, "AWBGAIN") == 0) {
      // automatic white balance gain 1 or 0
      retVal = s->set_awb_gain(s, getValue(cameraCommand, 7));
    } else if (cmdCmp(cameraCommand, "AWB") == 0) {
      // activate automatic white balance 0,1
      retVal = s->set_whitebal(s, getValue(cameraCommand, 3));



    } else if (cmdCmp(cameraCommand, "WBMODE") == 0) {
      // automatic white balance mode 0 to 4
      retVal = s->set_wb_mode(s, getValue(cameraCommand, 6));
    } else if (cmdCmp(cameraCommand, "AECVALUE") == 0) {
      // adjust exposure level 0-1200
      s->set_aec_value(s, getValue(cameraCommand, 8));
    } else if (cmdCmp(cameraCommand, "AECDSP") == 0) {
      // activate automatic exposure control digital processing 0,1
      retVal = s->set_aec2(s, getValue(cameraCommand, 6));
    } else if (cmdCmp(cameraCommand, "AEC") == 0) {
      // activate automatic exposure control 0,1
      retVal = s->set_exposure_ctrl(s, getValue(cameraCommand, 3));



    } else if (cmdCmp(cameraCommand, "AELEVEL") == 0) {
      // adjust automatic exposure level -2, -1, 0,1,2
      retVal = s->set_ae_level(s, getValue(cameraCommand, 7));
    } else if (cmdCmp(cameraCommand, "AGCGAINMAX") == 0) {
      // adjust automatic gain control ceiling 0,1,2,3,4,5,6
      retVal = s->set_gainceiling(s, (gainceiling_t)getValue(cameraCommand, 10));
      //res = s->set_gainceiling(s, (gainceiling_t)val);
    } else if (cmdCmp(cameraCommand, "AGCGAIN") == 0) {
      // adjust automatic gain control level 0,1,2,3,4,5,6

      retVal = s->set_agc_gain(s, getValue(cameraCommand, 7));
    } else if (cmdCmp(cameraCommand, "AGC") == 0) {
      // activate automatic gain control 0,1
      retVal = s->set_gain_ctrl(s, getValue(cameraCommand, 3));

    } else if (cmdCmp(cameraCommand, "BPC") == 0) {
      // activate black pixel correction 0,1
      retVal = s->set_bpc(s, getValue(cameraCommand, 3));

    } else if (cmdCmp(cameraCommand, "WPC") == 0) {
      // activate white pixel correction 0,1
      retVal = s->set_wpc(s, getValue(cameraCommand, 3));

    } else if (cmdCmp(cameraCommand, "RAWGMA") == 0) {
      // activate gamma correction 0,1
      retVal = s->set_raw_gma(s, getValue(cameraCommand, 6));
    } else if (cmdCmp(cameraCommand, "LENC") == 0) {
      // activate lens correction correction 0,1
      retVal = s->set_lenc(s, getValue(cameraCommand, 4));
    } else if (cmdCmp(cameraCommand, "DCW") == 0) {
      // activate direct conversion 0,1
      retVal = s->set_dcw(s, getValue(cameraCommand, 3));

    } else if (debugSerial) {
      if (debugSerial) { Serial.printf(" %s - not known\n", cameraCommand); }
    }
  } else if (debugSerial) {
    if (debugSerial) { Serial.printf(" %s - not available - video not active\n", cameraCommand); }
  }

  return 0;
}


// *********************************************************************************************
// process guidance only commands - this is primarily to offload code complexity from the
// local command handler routine
// *********************************************************************************************
int guideControl(char localVariable[]) {
  int retVal = 0;
  char guideCommand[33];

  // left shift command
  int i = 0;
  while (localVariable[i] != 0) {
    guideCommand[i] = localVariable[i + 1];
    i++;
    if (i > 24) break;
  }
  if (cmdCmp(guideCommand, "VIDEO") == 0) {
    // activate video streamimg mode
    execVideo = getValue(guideCommand, 5);
  } else if (cmdCmp(guideCommand, "GUIDE") == 0) {
    // activate video guidance mode
    execGuide = getValue(guideCommand, 5);
  } else if (execVideo || execGuide) {
    sensor_t *s = esp_camera_sensor_get();

    if (cmdCmp(guideCommand, "GREEN") == 0) {
      // set green threshold, positve indicates must be greater, negative indicates must be less
      greenThreshold = getValue(guideCommand, 6);
    } else if (cmdCmp(guideCommand, "RED") == 0) {
      // set red threshold, positve indicates must be greater, negative indicates must be less
      redThreshold = getValue(guideCommand, 4);
    } else if (cmdCmp(guideCommand, "BLUE") == 0) {
      // set blue threshold, positve indicates must be greater, negative indicates must be less
      blueThreshold = getValue(guideCommand, 5);
    } else if (cmdCmp(guideCommand, "BRIGHT") == 0) {
      // set value for brightness adjustment
      brightnessAdjust = getValue(guideCommand, 6);
    } else if (cmdCmp(guideCommand, "COLOUR") == 0) {
      // set value for colour detection
      colourDetect = getValue(guideCommand, 6);
      if (colourDetect < 0 || colourDetect > 10) {
        colourDetect = 10;
      }
      blueThreshold = colourTranslate[0][colourDetect];
      greenThreshold = colourTranslate[1][colourDetect];
      redThreshold = colourTranslate[2][colourDetect];

    } else if (cmdCmp(guideCommand, "UPDATE") == 0) {
      // set update colour
      // find coloured line
      // #define BLACK 0
      // #define RED 1
      // #define ORANGE 2
      // #define YELLOW 3
      // #define GREEN 4
      // #define BLUE 5
      // #define VIOLET 7
      // #define WHITE 10
      updateColour = getValue(guideCommand, 6);
      // set threshold for top line detection
    } else if (cmdCmp(guideCommand, "LOOKAHEAD") == 0) {
      lookAhead = getValue(guideCommand, 10);
          } else if (cmdCmp(guideCommand, "THRESHOLDT") == 0) {
      lineThresholdT = getValue(guideCommand, 10);
      // set threshold for centre line detection
    } else if (cmdCmp(guideCommand, "THRESHOLDC") == 0) {
      lineThresholdC = getValue(guideCommand, 10);
      // set threshold for bottom line detection
    } else if (cmdCmp(guideCommand, "THRESHOLDB") == 0) {
      lineThresholdB = getValue(guideCommand, 10);
    } else if (cmdCmp(guideCommand, "THRESHOLD") == 0) {
      lineThresholdT = getValue(guideCommand, 9);
      lineThresholdC = lineThresholdT;
      lineThresholdB = lineThresholdT;
      blobThreshold = lineThresholdT;
    } else if (cmdCmp(guideCommand, "LINEPROC") == 0) {
      execLineProcessing = getValue(guideCommand, 8);
      // reset blob mask to blank
    } else if (cmdCmp(guideCommand, "BLOBMASKNONE") == 0) {
      blobMask[0] = 0;
      blobMask[1] = 0;
      blobMask[2] = 0;
      blobMask[3] = 0;
      // send last blob position
    } else if (cmdCmp(guideCommand, "BLOBDATA") == 0) {
      Serial.printf("%04d,%04d,%04d\r\n", lastBlobCount, lastBlobX, lastBlobY);
      // send last line positions
    } else if (cmdCmp(guideCommand, "LINEDATA") == 0) {
      Serial.printf("%04d,%04d,%04d,%04d,%04d,%04d\r\n", lineInstancesT[1], lineInstancesT[2],
                    lineInstancesC[1], lineInstancesC[2], lineInstancesB[1], lineInstancesB[2]);
      // set blob mask for area to be ignored by blob detection routine
      // check BLOBMASK command for string and string length
    } else if (cmdCmp(guideCommand, "BLOBMASK") == 0 && strlen(guideCommand) == 24) {
      // initialise temporary character arrays

      char maskY1[5] = { 0, 0, 0, 0, 0 };
      char maskY2[5] = { 0, 0, 0, 0, 0 };
      char maskX1[5] = { 0, 0, 0, 0, 0 };
      char maskX2[5] = { 0, 0, 0, 0, 0 };

      // load character arrays with values

      for (int i = 0; i < 4; i++) {
        maskY1[i] = guideCommand[i + 8];
        maskY2[i] = guideCommand[i + 12];
        maskX1[i] = guideCommand[i + 16];
        maskX2[i] = guideCommand[i + 20];
      }
      Serial.println(maskY1);
      Serial.println(maskY2);
      Serial.println(maskX1);
      Serial.println(maskX2);
      blobMask[0] = atoi(maskY1);
      blobMask[1] = atoi(maskY2);
      blobMask[2] = atoi(maskX1);
      blobMask[3] = atoi(maskX2);

    } else if (cmdCmp(guideCommand, "LINEDETECT") == 0) {
      // set line following in script
      execLine = getValue(guideCommand, 4);
      if (execLine) execBlob = 0;
    } else if (cmdCmp(guideCommand, "BLOBDETECT") == 0) {
      // set blob location in script
      execBlob = getValue(guideCommand, 6);
      if (execBlob) execLine = 0;
    } else if (cmdCmp(guideCommand, "OVERLAY") == 0) {
      // add overlay to guide 0 = none, 1= crosshairs, 2 = graduations, 3 = % graduations
      execOverlay = getValue(guideCommand, 7);
    } else if (cmdCmp(guideCommand, "BLOBPROC") == 0) {
      // type of blob processing for analysis 0 = standard blob location, 1 = heatmap of colour, 2 = scantest, 3 = colour detection map
      execBlobProcessing = getValue(guideCommand, 8);
    } else if (debugSerial) {
      if (debugSerial) { Serial.printf(" %s - not known\n", guideCommand); }
    }
  } else if (debugSerial) {
    if (debugSerial) { Serial.printf(" %s - not available - video not active\n", guideCommand); }
  }

  return 0;
}
// *********************************************************************************************
// process demonstration only commands - this is primarily to offload code complexity from the
// local command handler routine
// *********************************************************************************************
int demoControl(char localVariable[]) {
  int retVal = 0;
  char demoCommand[33];

  // left shift command
  int i = 0;
  while (localVariable[i] != 0) {
    demoCommand[i] = localVariable[i + 1];
    i++;
    if (i > 24) break;
  }

  if (cmdCmp(demoCommand, "FOLLOW") == 0) {
    // initiate line following, only if line following and
    greenThreshold = getValue(demoCommand, 6);
  } else if (cmdCmp(demoCommand, "CALIBRATE") == 0) {
    // set red threshold, positve indicates must be greater, negative indicates must be less
    redThreshold = getValue(demoCommand, 4);
  } else if (cmdCmp(demoCommand, "TRACK") == 0) {
    // set blue threshold, positve indicates must be greater, negative indicates must be less
    blueThreshold = getValue(demoCommand, 5);
  } else if (cmdCmp(demoCommand, "LINEFOLLOWER") == 0) {
    // set value for brightness adjustment
    brightnessAdjust = getValue(demoCommand, 6);
  } else if (cmdCmp(demoCommand, "BLOBFOLLOWER") == 0) {
    // set value for colour detection
    colourDetect = getValue(demoCommand, 6);
    if (colourDetect < 0 || colourDetect > 10) {
      colourDetect = 10;
    }
    blueThreshold = colourTranslate[0][colourDetect];
    greenThreshold = colourTranslate[1][colourDetect];
    redThreshold = colourTranslate[2][colourDetect];

  } else if (cmdCmp(demoCommand, "UPDATE") == 0) {
    // set update colour
    // find coloured line
    // #define BLACK 0
    // #define RED 1
    // #define ORANGE 2
    // #define YELLOW 3
    // #define GREEN 4
    // #define BLUE 5
    // #define VIOLET 7
    // #define WHITE 10
    updateColour = getValue(demoCommand, 6);
    // set threshold for top line detection
  } else if (cmdCmp(demoCommand, "THRESHOLDT") == 0) {
    lineThresholdT = getValue(demoCommand, 10);
    // set threshold for centre line detection
  } else if (cmdCmp(demoCommand, "THRESHOLDC") == 0) {
    lineThresholdC = getValue(demoCommand, 10);
    // set threshold for bottom line detection
  } else if (cmdCmp(demoCommand, "THRESHOLDB") == 0) {
    lineThresholdB = getValue(demoCommand, 10);
  } else if (cmdCmp(demoCommand, "THRESHOLD") == 0) {
    lineThresholdT = getValue(demoCommand, 9);
    lineThresholdC = lineThresholdT;
    lineThresholdB = lineThresholdT;
    blobThreshold = lineThresholdT;
    // reset PID factors
  } else if (cmdCmp(demoCommand, "PIDRESET") == 0) {
    factorP = 0;
    factorI = 0;
    factorD = 0;
    // send stored PID factors
  } else if (cmdCmp(demoCommand, "PIDS") == 0) {
    Serial.printf("%04d,%04d,%04d\r\n", factorP, factorI, factorD);
    // send last line positions
  } else if (cmdCmp(demoCommand, "LINEDATA") == 0) {
    Serial.printf("%04d,%04d,%04d,%04d,%04d,%04d\r\n", lineInstancesT[1], lineInstancesT[2],
                  lineInstancesC[1], lineInstancesC[2], lineInstancesB[1], lineInstancesB[2]);

    // store input PID factors
  } else if (cmdCmp(demoCommand, "PIDFACTOR") == 0 && strlen(demoCommand) == 21) {
    // initialise temporary character arrays

    char factorPC[5] = { 0, 0, 0, 0, 0 };
    char factorIC[5] = { 0, 0, 0, 0, 0 };
    char factorDC[5] = { 0, 0, 0, 0, 0 };

    // load character arrays with values

    for (int i = 0; i < 4; i++) {
      factorPC[i] = demoCommand[i + 9];
      factorIC[i] = demoCommand[i + 13];
      factorDC[i] = demoCommand[i + 17];
    }
    // Serial.println(maskY1);
    // Serial.println(maskY2);
    // Serial.println(maskX1);
    // Serial.println(maskX2);
    factorP = atoi(factorPC);
    factorI = atoi(factorIC);
    factorD = atoi(factorDC);

  } else if (cmdCmp(demoCommand, "LINEDETECT") == 0) {
    // set line following in script
    execLine = getValue(demoCommand, 4);
    if (execLine) execBlob = 0;
  } else if (cmdCmp(demoCommand, "BLOBDETECT") == 0) {
    // set blob location in script
    execBlob = getValue(demoCommand, 6);
    if (execBlob) execLine = 0;
  } else if (cmdCmp(demoCommand, "OVERLAY") == 0) {
    // add overlay to guide 0 = none, 1= crosshairs, 2 = graduations, 3 = % graduations
    execOverlay = getValue(demoCommand, 7);
  } else if (cmdCmp(demoCommand, "BLOBPROC") == 0) {
    // type of blob processing for analysis 0 = standard blob location, 1 = heatmap of colour, 2 = scantest, 3 = colour detection map
    execBlobProcessing = getValue(demoCommand, 8);
  } else if (debugSerial) {
    Serial.printf(" %s - not known\n", demoCommand);
  }

  // } else if (debugSerial) {
  //   if (debugSerial) { Serial.printf(" %s - not available - video not active\n", demoCommand); }
  return retVal;
}
