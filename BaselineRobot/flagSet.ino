// This routine is included to provide a fixed set of configurations
// An option to provide a configuration file will be provided at a later date
// when this may be removed, the point being to make it simpler
void flagSet(char execType[20]) {
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
  if (strcmp(execType, "BASICSTA") == 0) {
    ///set BASICSTA mode, serial passthru on, no handshake, Wifi client, web page input, local drive commands processed to drive pins
    execWiFi = 1;       // wifi client
    execPS3 = 0;        // use PS3 input
    execSensor = 0;     // use serial pins for local sensor input
    execSerial = 1;     // configure and pass output to serial port
    execMotor = 1;      // configure motor control output
    execHandshake = 0;  // pins configured for remote controller serial handshake
    execServo = 0;      // configure servo control output
    execPolarity = 0;   // negative motor drive
    execGuide = 0;      // turn on video guidance
    execVideo = 1;      // turn on video streaming
    execLine = 0;       // line foloowing video, only active if execGuide set
    execBlob = 0;       // blob locating video, only active if execGuide set
  } else if (strcmp(execType, "BASICAP") == 0) {
    ///set BASICAP mode, serial passthru on, no handshake, Wifi Access Point, web page input, local drive commands processed to drive pins
    execWiFi = 2;       // wifi access point
    execPS3 = 0;        // use PS3 input
    execSensor = 0;     // use serial pins for local sensor input
    execSerial = 1;     // configure and pass output to serial port
    execMotor = 1;      // configure motor control output
    execHandshake = 0;  // pins configured for remote controller serial handshake
    execServo = 0;      // configure servo control output
    execPolarity = 0;   // negative motor drive
    execGuide = 0;      // turn on video guidance
    execVideo = 1;      // turn on video streaming
    execLine = 0;       // line foloowing video, only active if execGuide set
    execBlob = 0;       // blob locating video, only active if execGuide set
  } else if (strcmp(execType, "BASICPS3") == 0) {
    ///set BASICPS3 mode, serial passthru on, no handshake, PS3 input, local drive commands processed to drive pins
    execWiFi = 0;       // wifi client
    execPS3 = 1;        // use PS3 input
    execSensor = 0;     // use serial pins for local sensor input
    execSerial = 1;     // configure and pass output to serial port
    execMotor = 1;      // configure motor control output
    execHandshake = 0;  // pins configured for remote controller serial handshake
    execServo = 0;      // configure servo control output
    execPolarity = 0;   // negative motor drive
    execGuide = 0;      // turn on video guidance
    execVideo = 0;      // turn on video streaming
    execLine = 0;       // line foloowing video, only active if execGuide set
    execBlob = 0;       // blob locating video, only active if execGuide set
  } else if (strcmp(execType, "BASICSTA+") == 0) {
    ///set BASICSTA+ mode, serial passthru on, no handshake, Wifi client, web page input, local drive commands processed to drive pins as positive
    execWiFi = 1;       // wifi client
    execPS3 = 0;        // use PS3 input
    execSensor = 0;     // use serial pins for local sensor input
    execSerial = 1;     // configure and pass output to serial port
    execMotor = 1;      // configure motor control output
    execHandshake = 0;  // pins configured for remote controller serial handshake
    execServo = 0;      // configure servo control output
    execPolarity = 1;   // positive motor drive
    execGuide = 0;      // turn on video guidance
    execVideo = 1;      // turn on video streaming
    execLine = 0;       // line foloowing video, only active if execGuide set
    execBlob = 0;       // blob locating video, only active if execGuide set
  } else if (strcmp(execType, "BASICAP+") == 0) {
    ///set BASICAP+ mode, serial passthru on, no handshake, Wifi Access Point, web page input, local drive commands processed to drive pins as positive
    execWiFi = 2;       // wifi access point
    execPS3 = 0;        // use PS3 input
    execSensor = 0;     // use serial pins for local sensor input
    execSerial = 1;     // configure and pass output to serial port
    execMotor = 1;      // configure motor control output
    execHandshake = 0;  // pins configured for remote controller serial handshake
    execServo = 0;      // configure servo control output
    execPolarity = 1;   // positive motor drive
    execGuide = 0;      // turn on video guidance
    execVideo = 1;      // turn on video streaming
    execLine = 0;       // line foloowing video, only active if execGuide set
    execBlob = 0;       // blob locating video, only active if execGuide set
  } else if (strcmp(execType, "BASICPS3+") == 0) {
    ///set BASICPS3+ mode, serial passthru on, no handshake, PS3 input, local drive commands processed to drive pins as positive
    execWiFi = 0;       // wifi client
    execPS3 = 1;        // use PS3 input
    execSensor = 0;     // use serial pins for local sensor input
    execSerial = 1;     // configure and pass output to serial port
    execMotor = 1;      // configure motor control output
    execHandshake = 0;  // pins configured for remote controller serial handshake
    execServo = 0;      // configure servo control output
    execPolarity = 1;   // positive motor drive
    execGuide = 0;      // turn on video guidance
    execVideo = 0;      // turn on video streaming
    execLine = 0;       // line foloowing video, only active if execGuide set
    execBlob = 0;       // blob locating video, only active if execGuide set
  } else if (strcmp(execType, "SOKOBANSTA") == 0) {
    ///set SOKOBANSTA mode, serial passthru off, no handshake, Wifi client, web page input, local drive commands processed to alternative drive pins
    execWiFi = 1;       // wifi client
    execPS3 = 0;        // use PS3 input
    execSensor = 0;     // use serial pins for local sensor input
    execSerial = 0;     // configure and pass output to serial port
    execMotor = 2;      // configure motor control output
    execHandshake = 0;  // pins configured for remote controller serial handshake
    execServo = 0;      // configure servo control output
    execPolarity = 1;   // positive motor drive
    execGuide = 0;      // turn on video guidance
    execVideo = 1;      // turn on video streaming
    execLine = 0;       // line foloowing video, only active if execGuide set
    execBlob = 0;       // blob locating video, only active if execGuide set
  } else if (strcmp(execType, "SOKOBANAP") == 0) {
    ///set SOKOBANAP mode, serial passthru off no handshake, Wifi Access Point, web page input, local drive commands processed to alternative drive pins
    execWiFi = 2;       // wifi access point
    execPS3 = 0;        // use PS3 input
    execSensor = 0;     // use serial pins for local sensor input
    execSerial = 0;     // configure and pass output to serial port
    execMotor = 2;      // configure motor control output
    execHandshake = 0;  // pins configured for remote controller serial handshake
    execServo = 0;      // configure servo control output
    execPolarity = 1;   // positive motor drive
    execGuide = 0;      // turn on video guidance
    execVideo = 1;      // turn on video streaming
    execLine = 0;       // line foloowing video, only active if execGuide set
    execBlob = 0;       // blob locating video, only active if execGuide set
  } else if (strcmp(execType, "SOKOBANPS3") == 0) {
    ///set SOKOBANPS3 mode, serial passthru off, no handshake, PS3 input, local drive commands processed to alternative drive pins
    execWiFi = 0;       // wifi client
    execPS3 = 1;        // use PS3 input
    execSensor = 0;     // use serial pins for local sensor input
    execSerial = 0;     // configure and pass output to serial port
    execMotor = 2;      // configure motor control output
    execHandshake = 0;  // pins configured for remote controller serial handshake
    execServo = 0;      // configure servo control output
    execPolarity = 1;   // positive motor drive
    execGuide = 0;      // turn on video guidance
    execVideo = 0;      // turn on video streaming
    execLine = 0;       // line foloowing video, only active if execGuide set
    execBlob = 0;       // blob locating video, only active if execGuide set
  } else if (strcmp(execType, "REMOTESTA") == 0) {
    ///set REMOTESTA mode, serial passthru on, handshake on, Wifi client, web page input
    execWiFi = 1;       // wifi access point
    execPS3 = 0;        // use PS3 input
    execSensor = 0;     // use serial pins for local sensor input
    execSerial = 1;     // configure and pass output to serial port
    execMotor = 0;      // configure motor control output
    execHandshake = 1;  // pins configured for remote controller serial handshake
    execServo = 0;      // configure servo control output
    execGuide = 0;      // turn on video guidance
    execVideo = 1;      // turn on video streaming
    execLine = 0;       // line foloowing video, only active if execGuide set
    execBlob = 0;       // blob locating video, only active if execGuide set
  } else if (strcmp(execType, "REMOTEAP") == 0) {
    ///set REMOTEAP mode, serial passthru on, handshake on, Wifi Access Point, web page input
    execWiFi = 2;       // wifi client
    execPS3 = 0;        // use PS3 input
    execSensor = 0;     // use serial pins for local sensor input
    execSerial = 1;     // configure and pass output to serial port
    execMotor = 0;      // configure motor control output
    execHandshake = 1;  // pins configured for remote controller serial handshake
    execServo = 0;      // configure servo control output
    execGuide = 0;      // turn on video guidance
    execVideo = 1;      // turn on video streaming
    execLine = 0;       // line foloowing video, only active if execGuide set
    execBlob = 0;       // blob locating video, only active if execGuide set
  } else if (strcmp(execType, "REMOTEPS3") == 0) {
    ///set REMOTEPS3 mode, serial passthru on, handshake on, PS3 controller input
    execWiFi = 0;       // wifi client
    execPS3 = 1;        // use PS3 input
    execSensor = 0;     // use serial pins for local sensor input
    execSerial = 1;     // configure and pass output to serial port
    execMotor = 0;      // configure motor control output
    execHandshake = 1;  // pins configured for remote controller serial handshake
    execServo = 0;      // configure servo control output
    execGuide = 0;      // turn on video guidance
    execVideo = 0;      // turn on video streaming
    execLine = 0;       // line foloowing video, only active if execGuide set
    execBlob = 0;       // blob locating video, only active if execGuide set
  } else if (strcmp(execType, "SERVOSTA") == 0) {
    ///set SERVOSTA mode, serial passthru on, no handshake, Wifi client, web page input, servo commands processed to 2 pins
    execWiFi = 1;       // wifi client
    execPS3 = 0;        // use PS3 input
    execSensor = 0;     // use serial pins for local sensor input
    execSerial = 1;     // configure and pass output to serial port
    execMotor = 0;      // configure motor control output
    execHandshake = 0;  // pins configured for remote controller serial handshake
    execServo = 1;      // configure servo control output
    execGuide = 0;      // turn on video guidance
    execVideo = 1;      // turn on video streaming
    execLine = 0;       // line foloowing video, only active if execGuide set
    execBlob = 0;       // blob locating video, only active if execGuide set
  } else if (strcmp(execType, "SERVOAP") == 0) {
    ///set SERVOAP mode, serial passthru on, no handshake, Wifi Access Point, web page input, servo commands processed to 2 pins
    execWiFi = 2;       // wifi access point
    execPS3 = 0;        // use PS3 input
    execSensor = 0;     // use serial pins for local sensor input
    execSerial = 1;     // configure and pass output to serial port
    execMotor = 0;      // configure motor control output
    execHandshake = 0;  // pins configured for remote controller serial handshake
    execServo = 1;      // configure servo control output
    execGuide = 0;      // turn on video guidance
    execVideo = 1;      // turn on video streaming
    execLine = 0;       // line foloowing video, only active if execGuide set
    execBlob = 0;       // blob locating video, only active if execGuide set
  } else if (strcmp(execType, "SERVOPS3") == 0) {
    ///set SERVOPS3 mode, serial passthru on, no handshake, PS3 controller input, servo commands processed to 2 pins
    execWiFi = 0;       // wifi client
    execPS3 = 1;        // use PS3 input
    execSensor = 0;     // use serial pins for local sensor input
    execSerial = 1;     // configure and pass output to serial port
    execMotor = 0;      // configure motor control output
    execHandshake = 0;  // pins configured for remote controller serial handshake
    execServo = 1;      // configure servo control output
    execGuide = 0;      // turn on video guidance
    execVideo = 0;      // turn on video streaming
    execLine = 0;       // line foloowing video, only active if execGuide set
    execBlob = 0;       // blob locating video, only active if execGuide set
  } else if (strcmp(execType, "LINESTA") == 0) {
    ///set LINESTA mode, serial passthru on, no handshake, Wifi client, web page input, motor commands processed to 2 pins, default operation is line following
    execWiFi = 1;       // wifi client
    execPS3 = 0;        // use PS3 input
    execSensor = 0;     // use serial pins for local sensor input
    execSerial = 1;     // configure and pass output to serial port
    execMotor = 1;      // configure motor control output
    execHandshake = 0;  // pins configured for remote controller serial handshake
    execServo = 0;      // configure servo control output
    execPolarity = 0;   // negative motor drive
    execGuide = 1;      // turn on video guidance
    execVideo = 1;      // turn on video streaming
    execScript = 1;     // turn off script processing
    execLine = 1;       // line foloowing video, only active if execGuide set
    execBlob = 0;       // blob locating video, only active if execGuide set
  } else if (strcmp(execType, "LINEAP") == 0) {
    //set LINEAP mode, serial passthru on, no handshake, Wifi Access Point, web page input, motor commands processed to 2 pins, default operation is line following
    execWiFi = 2;       // wifi client
    execPS3 = 0;        // use PS3 input
    execSensor = 0;     // use serial pins for local sensor input
    execSerial = 1;     // configure and pass output to serial port
    execMotor = 1;      // configure motor control output
    execHandshake = 0;  // pins configured for remote controller serial handshake
    execServo = 0;      // configure servo control output
    execPolarity = 0;   // negative motor drive
    execGuide = 1;      // turn on video guidance
    execVideo = 1;      // turn on video streaming
    execScript = 1;     // turn off script processing
    execLine = 1;       // line following video, only active if execGuide set
    execBlob = 0;       // blob locating video, only active if execGuide set
  } else if (strcmp(execType, "LINEPS3") == 0) {
    //set LINEPS3 mode, serial passthru on, no handshake, PS3 Controller, motor commands processed to 2 pins, default operation is line following
    execWiFi = 0;       // wifi client
    execPS3 = 1;        // use PS3 input
    execSensor = 0;     // use serial pins for local sensor input
    execSerial = 1;     // configure and pass output to serial port
    execMotor = 1;      // configure motor control output
    execHandshake = 0;  // pins configured for remote controller serial handshake
    execServo = 0;      // configure servo control output
    execPolarity = 0;   // negative motor drive
    execGuide = 1;      // turn on video guidance
    execVideo = 0;      // turn on video streaming
    execScript = 1;     // turn off script processing
    execLine = 1;       // line following video, only active if execGuide set
    execBlob = 0;       // blob locating video, only active if execGuide set
      } else if (strcmp(execType, "LINESENS") == 0) {
    //set LINESENS mode, operation is sending line positional data to serial port
    execWiFi = 0;       // wifi client
    execPS3 = 1;        // use PS3 input
    execSensor = 0;     // use serial pins for local sensor input
    execSerial = 1;     // configure and pass output to serial port
    execMotor = 0;      // configure motor control output
    execHandshake = 0;  // pins configured for remote controller serial handshake
    execServo = 0;      // configure servo control output
    execPolarity = 0;   // negative motor drive
    execGuide = 1;      // turn on video guidance
    execVideo = 0;      // turn on video streaming
    execScript = 1;     // turn off script processing
    execLine = 1;       // line following video, only active if execGuide set
    execBlob = 0;       // blob locating video, only active if execGuide set
  } else if (strcmp(execType, "BLOBSTA") == 0) {
    //set BLOBSTA mode, serial passthru on, no handshake, Wifi client, web page input, motor commands processed to 2 pins, default operation is blob location
    execWiFi = 1;       // wifi client
    execPS3 = 0;        // use PS3 input
    execSensor = 0;     // use serial pins for local sensor input
    execSerial = 1;     // configure and pass output to serial port
    execMotor = 1;      // configure motor control output
    execHandshake = 0;  // pins configured for remote controller serial handshake
    execServo = 0;      // configure servo control output
    execPolarity = 0;   // negative motor drive
    execGuide = 1;      // turn on video guidance
    execVideo = 1;      // turn on video streaming
    execScript = 1;     // turn off script processing
    execLine = 0;       // line foloowing video, only active if execGuide set
    execBlob = 1;       // blob locating video, only active if execGuide set
  } else if (strcmp(execType, "BLOBAP") == 0) {
    //set BLOBAP mode, serial passthru on, no handshake, Wifi Access Point, web page input, motor commands processed to 2 pins, default operation is blob location
    execWiFi = 2;       // wifi client
    execPS3 = 0;        // use PS3 input
    execSensor = 0;     // use serial pins for local sensor input
    execSerial = 1;     // configure and pass output to serial port
    execMotor = 1;      // configure motor control output
    execHandshake = 0;  // pins configured for remote controller serial handshake
    execServo = 0;      // configure servo control output
    execPolarity = 0;   // negative motor drive
    execGuide = 1;      // turn on video guidance
    execVideo = 1;      // turn on video streaming
    execScript = 1;     // turn off script processing
    execLine = 0;       // line foloowing video, only active if execGuide set
    execBlob = 1;       // blob locating video, only active if execGuide set
  } else if (strcmp(execType, "BLOBPS3") == 0) {
    //set BLOBPS3 mode, serial passthru on, no handshake, PS3 Controller, motor commands processed to 2 pins, default operation is blob location
    execWiFi = 0;       // wifi client
    execPS3 = 1;        // use PS3 input
    execSensor = 0;     // use serial pins for local sensor input
    execSerial = 1;     // configure and pass output to serial port
    execMotor = 1;      // configure motor control output
    execHandshake = 0;  // pins configured for remote controller serial handshake
    execServo = 0;      // configure servo control output
    execPolarity = 0;   // negative motor drive
    execGuide = 1;      // turn on video guidance
    execVideo = 0;      // turn on video streaming
    execScript = 1;     // turn off script processing
    execLine = 0;       // line foloowing video, only active if execGuide set
    execBlob = 1;       // blob locating video, only active if execGuide set
      } else if (strcmp(execType, "BLOBSENS") == 0) {
    //set BLOBSENS mode, operation is sending blob positional data to serial port
    execWiFi = 0;       // wifi client
    execPS3 = 1;        // use PS3 input
    execSensor = 0;     // use serial pins for local sensor input
    execSerial = 1;     // configure and pass output to serial port
    execMotor = 0;      // configure motor control output
    execHandshake = 0;  // pins configured for remote controller serial handshake
    execServo = 0;      // configure servo control output
    execPolarity = 0;   // negative motor drive
    execGuide = 1;      // turn on video guidance
    execVideo = 0;      // turn on video streaming
    execScript = 1;     // turn off script processing
    execLine = 0;       // line foloowing video, only active if execGuide set
    execBlob = 1;       // blob locating video, only active if execGuide set
  } else if (strcmp(execType, "FILELOAD") == 0) {
    ///set FILELOAD disables all options expecting the setting to be loaded from a file
    // this is intended to be only a placeholder for the option with the future options being moved out of this routine and into
    // a file as per all the other customisations
    execWiFi = 0;       // wifi client
    execPS3 = 0;        // use PS3 input
    execSensor = 0;     // use serial pins for local sensor input
    execSerial = 0;     // configure and pass output to serial port
    execMotor = 0;      // configure motor control output, value indicates uni (two pins) or bi-directional (four pins)
    execHandshake = 0;  // pins configured for remote controller serial handshake
    execServo = 0;      // configure servo control output, value indicates either 2 or 4 pins
    execGuide = 0;      // turn on video guidance
    execVideo = 1;      // turn on video streaming
    execLine = 0;       // line foloowing video, only active if execGuide set
    execBlob = 0;       // blob locating video, only active if execGuide set
  }
}
void flagReport(void) {
  Serial.println("flagReport running)");
  if (debugSerial) { Serial.printf("execWiFi = %d \n", execWiFi); }
  if (debugSerial) { Serial.printf("execPS3 = %d \n", execPS3); }
  if (debugSerial) { Serial.printf("execSensor = %d \n", execSensor); }
  if (debugSerial) { Serial.printf("execSerial = %d \n", execSerial); }
  if (debugSerial) { Serial.printf("execMotor = %d \n", execMotor); }
  if (debugSerial) { Serial.printf("execHandshake = %d \n", execHandshake); }
  if (debugSerial) { Serial.printf("execServo = %d \n", execServo); }
  if (debugSerial) { Serial.printf("execGuide = %d \n", execGuide); }
  if (debugSerial) { Serial.printf("execVideo = %d \n", execVideo); }
  if (debugSerial) { Serial.printf("execScript = %d \n", execScript); }
  if (debugSerial) { Serial.printf("execLine = %d \n", execLine); }
  if (debugSerial) { Serial.printf("execBlob = %d \n", execBlob); }
}