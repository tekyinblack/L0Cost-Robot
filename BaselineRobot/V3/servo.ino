// *********************************************************************************************
// process camera only commands - this is primarily to offload code complexity from the
// local command handler routine
// *********************************************************************************************
int servoControl(char localVariable[]) {
  int retVal = 0;
  char servoCommand[33];
  retVal = servoControl2(localVariable);
   return retVal;


  // left shift command
  int i = 0;
  while (localVariable[i] != 0) {
    servoCommand[i] = localVariable[i + 1];
    i++;
    if (i > 20) break;
  }




  if (cmdCmp(servoCommand, "RAWGMA") == 0) {
    // activate gamma correction 0,1
   // retVal = s->set_raw_gma(s, getValue(servoCommand, 6));
  } else if (cmdCmp(servoCommand, "LENC") == 0) {
    // activate lens correction correction 0,1
    //retVal = s->set_lenc(s, getValue(servoCommand, 4));
  } else if (cmdCmp(servoCommand, "DCW") == 0) {
    // activate direct conversion 0,1
    //retVal = s->set_dcw(s, getValue(servoCommand, 3));
  } else if (cmdCmp(servoCommand, "LUMINANCE") == 0) {
    // activate direct conversion 0,1
    Serial.printf("%04d,%08d\r\n", pixelLuminance(), millis());

  } else if (debugSerial) {
    if (debugSerial) { Serial.printf(" %s - not known\n", servoCommand); }
  }



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
// *********************************************************************************************
// servo command processing
// *********************************************************************************************
int servoControl2(char localVariable[]) {
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