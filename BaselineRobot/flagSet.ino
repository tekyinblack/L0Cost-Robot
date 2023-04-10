// This routine is included to provide a fixed set of configurations
// An option to provide a configuration file will be provided at a later date
// when this may be removed, the point being to make it simpler
void flagSet(char execType[20]) {
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
  if (strcmp(execType, "BASICSTA") == 0) {
    ///set BASICSTA mode, serial passthru on, no handshake, Wifi client, web page input, local drive commands processed to drive pins
    execWiFi = 1;       // wifi client
    execPS3 = 0;        // use PS3 input
    execSensor = 0;     // use serial pins for local sensor input
    execSerial = 1;     // configure and pass output to serial port
    execMotor = 1;      // configure motor control output 
    execHandshake = 0;  // pins configured for remote controller serial handshake
    execServo = 0;      // configure servo control output, 
  } else if (strcmp(execType, "BASICAP") == 0) {
    ///set BASICAP mode, serial passthru on, no handshake, Wifi Access Point, web page input, local drive commands processed to drive pins
    execWiFi = 2;       // wifi access point
    execPS3 = 0;        // use PS3 input
    execSensor = 0;     // use serial pins for local sensor input
    execSerial = 1;     // configure and pass output to serial port
    execMotor = 1;      // configure motor control output 
    execHandshake = 0;  // pins configured for remote controller serial handshake
    execServo = 0;      // configure servo control output
  } else if (strcmp(execType, "BASICPS3") == 0) {
    ///set BASICPS3 mode, serial passthru on, no handshake, PS3 input, local drive commands processed to drive pins
    execWiFi = 0;       // wifi client
    execPS3 = 1;        // use PS3 input
    execSensor = 0;     // use serial pins for local sensor input
    execSerial = 1;     // configure and pass output to serial port
    execMotor = 1;      // configure motor control output
    execHandshake = 0;  // pins configured for remote controller serial handshake
    execServo = 0;      // configure servo control output
  } else if (strcmp(execType, "REMOTESTA") == 0) {
    ///set REMOTESTA mode, serial passthru on, handshake on, Wifi client, web page input
    execWiFi = 1;       // wifi access point
    execPS3 = 0;        // use PS3 input
    execSensor = 0;     // use serial pins for local sensor input
    execSerial = 1;     // configure and pass output to serial port
    execMotor = 0;      // configure motor control output
    execHandshake = 1;  // pins configured for remote controller serial handshake
    execServo = 0;      // configure servo control output
  } else if (strcmp(execType, "REMOTEAP") == 0) {
    ///set REMOTEAP mode, serial passthru on, handshake on, Wifi Access Point, web page input
    execWiFi = 2;       // wifi client
    execPS3 = 0;        // use PS3 input
    execSensor = 0;     // use serial pins for local sensor input
    execSerial = 1;     // configure and pass output to serial port
    execMotor = 0;      // configure motor control output
    execHandshake = 1;  // pins configured for remote controller serial handshake
    execServo = 0;      // configure servo control output
  } else if (strcmp(execType, "REMOTEPS3") == 0) {
    ///set REMOTEPS3 mode, serial passthru on, handshake on, PS3 controller input
    execWiFi = 0;       // wifi client
    execPS3 = 1;        // use PS3 input
    execSensor = 0;     // use serial pins for local sensor input
    execSerial = 1;     // configure and pass output to serial port
    execMotor = 0;      // configure motor control output
    execHandshake = 1;  // pins configured for remote controller serial handshake
    execServo = 0;      // configure servo control output
  } else  if (strcmp(execType, "SERVOSTA") == 0) {
    ///set SERVOSTA mode, serial passthru on, no handshake, Wifi client, web page input, servo commands processed to 2 pins
    execWiFi = 1;       // wifi client
    execPS3 = 0;        // use PS3 input
    execSensor = 0;     // use serial pins for local sensor input
    execSerial = 1;     // configure and pass output to serial port
    execMotor = 0;      // configure motor control output
    execHandshake = 0;  // pins configured for remote controller serial handshake
    execServo = 1;      // configure servo control output
  } else if (strcmp(execType, "SERVOAP") == 0) {
    ///set SERVOAP mode, serial passthru on, no handshake, Wifi Access Point, web page input, servo commands processed to 2 pins
    execWiFi = 2;       // wifi access point
    execPS3 = 0;        // use PS3 input
    execSensor = 0;     // use serial pins for local sensor input
    execSerial = 1;     // configure and pass output to serial port
    execMotor = 0;      // configure motor control output
    execHandshake = 0;  // pins configured for remote controller serial handshake
    execServo = 1;      // configure servo control output
  } else if (strcmp(execType, "SERVOPS3") == 0) {
    ///set SERVOPS3 mode, serial passthru on, no handshake, PS3 controller input, servo commands processed to 2 pins
    execWiFi = 0;       // wifi client
    execPS3 = 1;        // use PS3 input
    execSensor = 0;     // use serial pins for local sensor input
    execSerial = 1;     // configure and pass output to serial port
    execMotor = 0;      // configure motor control output
    execHandshake = 0;  // pins configured for remote controller serial handshake
    execServo = 1;      // configure servo control output
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
}