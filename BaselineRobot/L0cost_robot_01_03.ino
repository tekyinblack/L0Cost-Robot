/*
Version 01_03 is a consolidation version of the software ahead of the addition of video processing and concentrating on 
auxiliary processing platforms which most of the configuration options support. 
The changes in this version are debugging and tidy up of code.
*/
/*
Version 20 of the warehosue robot project has been renamed as version 01 of the L0Cost robot project
As of version 20, the use of the serial ports for anything else other than serial comms was to be 
discontinued, it was getting to be more effort than it was worth to use those pins for
anything else, but the use of teh RX pin was found to be possible so is now in use. 
There are other ESP32-CAM boards available with spare GPIO pins but 
their price makes them out of scope for a L0Cost robot.

All references to four pin motor and servo control have been removed
Two pin servo and motor control works well with pull-down resistors on pins 12 and 13
Three pin mode uses external logic gates to implement direction and speed control on a L293D
and uses the RX pin.

Common robot controller software based on the Espressif ESP32 camerawebserver example code
Much of what has been done here is to allow rapid customisation of the operation without
having to resort to modifying the code, primarily for use in demonstration environments.

The code has been modified to do the following and in the following ways
- wifi SSID and password moved to SD card though defaults can still be hard coded
- configuration parameters held on SD card though defaults can still be hard coded
- default website index page held on SD card though defaults can still be hard coded
- selection for signing onto local wifi or running own accesss point set in configuration
- adaptation for accepting commands for local execution or passthru to additional controller card such as an Arduino or Pico
- adaptation for accepting commands from web or held on SD card
- adaptation for accepting commands from PS3 controller
- pasthru of commands via serial interface
- local commands to modify camera operation, reboot, turn on/off flash LED
- local commands to use pins to drive motors and run handshaking to additional controllers
- onboard LED blinks to show normal operation or error conditions
- control file input accepts lines beginning // as comments   
*/
/* With debug set on, no local pin assignments and serial is assumed to be used for debug output only
All other functions continue to work
*/
/*- if in BASICSTA mode, serial passthru on, no handshake, Wifi client, web page input, local drive commands processed to drive pins
  - if in BASICAP mode, serial passthru on, no handshake, Wifi Access Point, web page input, local drive commands processed to drive pins
  - if in BASICPS3 mode, serial passthru on, no handshake, PS3 input, local drive commands processed to drive pins
  - if in BASICSTA+ mode, serial passthru on, no handshake, Wifi client, web page input, local drive commands processed to drive pins, Positive drive
  - if in BASICAP+ mode, serial passthru on, no handshake, Wifi Access Point, web page input, local drive commands processed to drive pins, Positive drive
  - if in BASICPS3+ mode, serial passthru on, no handshake, PS3 input, local drive commands processed to drive pins, Positive drive
  - if in REMOTESTA mode, serial passthru on, handshake on, Wifi client, web page input
  - if in REMOTEAP mode, serial passthru on, handshake on, Wifi Access Point, web page input
  - if in REMOTEPS3 mode, serial passthru on, handshake on, PS3 controller input
  - if in SERVOSTA mode, serial passthru on, no handshake, Wifi client, web page input, servo commands processed to 2 pins
  - if in SERVOAP mode, serial passthru on, no handshake, Wifi Access Point, web page input, servo commands processed to 2 pins
  - if in SERVOPS3 mode, serial passthru on, no handshake, PS3 controller input, servo commands processed to 2 pins
*/

#include "esp_camera.h"
#include "ESPmDNS.h"
#include <WiFi.h>
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "fb_gfx.h"
#include "soc/soc.h"           // disable brownout problems
#include "soc/rtc_cntl_reg.h"  // disable brownout problems
#include "esp_http_server.h"
#include "FS.h"
#include "SD_MMC.h"
#include "SPI.h"
#include <Ps3Controller.h>
#include <ESP32Servo.h>


// ===================
// Select camera model
// ===================
//#define CAMERA_MODEL_WROVER_KIT // Has PSRAM
//#define CAMERA_MODEL_ESP_EYE // Has PSRAM
//#define CAMERA_MODEL_ESP32S3_EYE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_PSRAM // Has PSRAM
//#define CAMERA_MODEL_M5STACK_V2_PSRAM // M5Camera version B Has PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE // Has PSRAM
#define CAMERA_MODEL_AI_THINKER  // Has PSRAM


#include "camera_pins.h"

// the following are code prototypes ++++++++++++++++++++++++++++
int setup_sdcard(void);
void read_SDcard(void);
void fileParse(char *line, int fileType, int lineNo);
int read_datafile(fs::FS &fs, const char *path, int fileType);
int readHtml(fs::FS &fs, const char *htmlf);

// flagset routines
void flagSet(char execType[20]);
void flagReport(void);
// robothandler routines
int cmdProcessor(char variable[], int source);
int sendHandshake(char commsString[]);
int remoteProcessor(char commsString[]);
int localProcessor(char localVariable[]);
void initPWM();
int motorControl(char localVariable[]);
int shiftLeft(char variable[], int step);
int initServos(void);
int servoControl(char localVariable[]);
void fileProcessor(int type);
int getValue(char fullCommand[], int offset);
int cmdCmp(char fullCommand[], char testCommand[]);
int cameraControl(char localVariable[]);
void shiftFileName(char *fileName);
void detachPWM(void);
void attachPWM(void);
void initThreePin(void);

// PS3 routines
void onConnect();
void notify();

static esp_err_t index_handler(httpd_req_t *req);
static esp_err_t stream_handler(httpd_req_t *req);
static esp_err_t cmd_handler(httpd_req_t *req);
void startCameraServer();

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define WIFI 1
#define CMDFILE 2
#define CONFIG 3
#define PS3CONFIG 4


#define FLASH_PIN 4
#define LED_BUILTIN 33


// execution flags
int execWiFi = 0;       // use wifi, value indicates 0= no wifi, 1=client or 2=access point
int execPS3 = 0;        // use PS3 input
int execSensor = 0;     // use serial pins for local sensor input
int execSerial = 0;     // configure and pass output to serial port
int execMotor = 0;      // configure motor control output
int execHandshake = 0;  // pins configured for remote controller serial handshake
int execServo = 0;      // configure servo control output, valu indicates either 2 or 4 pins
int execPWM = 0;        // motor control attached
int execPolarity = 0;   // motor polarity, 0 = negative, 1 = positive
int execCamera = 1;     // utilise the camera
int execVideo = 0;      // use video processing routines, limits frame buffer size and copies frame buffer for processing



// script file processing
char scriptFile[20] = "/startup.txt";
int scriptStatus = 0;
int scriptProcess = 0;
char scriptBuf[200];
File script;

// timer counters
long loopTimer = 0;
long ledTimer = 0;
long runTimer = 0;
long scriptTimer = 0;

// timers for motor control
long motorTimer = 0;          // timeout for last motor command
long directTimer = 0;         // timeout for direct control
long defaultTimeOut = 30000;  // default set to 30 seconds
long directTimeOut = 30000;

// flash led timer definitions
#define FLASH_ON_TIME 99999999  // flash LED timer set to 10,000 seconds, could be more but just sets a practical limit
#define FLASH_PEEK_TIME 5000
long flashTimer = FLASH_ON_TIME;

char *htmlPage;

char configDebug[20] = "NODEBUG";          // << flag debug processing DEBUG or NODEBUG
char configType[20] = "BASICSTA";          // << type of processing, see list in comments earlier
char configWifi[20] = "WiFi.txt";          // << file to be used from SD card for network setup
char configConfig[20] = "config.txt";      // << file to be used from SD card for configuration
char configStartup[20] = "startup.txt";    // << file to be used from SD card for startup commands
char configMain[20] = "main.txt";          // << file to be used from SD card for initial runtime commands
char ssid[33] = "DUMMY_SSID";              // dummy ssid text indicating an error
char password[33] = "DUMMY_PASSWORD";      // dummy wifi password indicating an error
char configHostname[20] = "DummyHost";     // dummy host name
char configPS3File[20] = "PS3.txt";        // << file to be used for PS3 address
char htmlFile[20] = "notfound";            // web page to be used to command robot and view video
int configWebPort = 80;                    // port number to be used for web server
int configStreamPort = 81;                 // port number to be used for video stream
char configPS3[20] = "FF:FF:FF:FF:FF:FF";  // dummy PS3 bluetooth address

// command processing variables
//int FLASH_PIN = 4;
int flashState = 0;



// locks
int cmdLock = 0;


// serial port controls
int essentialSerial = 0;  // if 1 turns on essential serial prints
int debugSerial = 1;      // if 1 turns on only debug messages

// definitions to run locost line follower with single ESP32-CAM
// these are activated by SDcard config.txt parameter

int rightMotorPin = 12;      // definition for local driving with locost robot 2 and 3 pin mode
int leftMotorPin = 13;       // definition for local driving with locost robot 2 and 3 pin mode
int leftMatrixMotorPin = 3;  // definition for local driving with locost robot 2 and 3 pin mode
int enableMotorPin = 13;     // definition for PWM for local driving in 3 pin mode

const int PWMFreq = 1000; /* 1 KHz */
const int PWMResolution = 8;
const int PWMSpeedChannel = 4;
const int rightMotorPWMSpeedChannel = 4;
const int leftMotorPWMSpeedChannel = 5;
const int enableMotorPWMSpeedChannel = 6;

// servo definitions
Servo dummy1;               // dummy servo definitions
Servo dummy2;               // avoids using camera allocations
Servo servo12;              // servo on GPIO12
Servo servo13;              // servo on GPIO13
int servo12Min = 500;       // 0 degree default timing setting
int servo12Max = 2500;      // max rotation default timing setting
int servo13Min = 500;       // 0 degree default timing setting
int servo13Max = 2500;      // max rotation default timing setting
int maxServo12Angle = 180;  // default max rotation angle
int maxServo13Angle = 180;  // default max rotation angle
int servo12Angle = 0;       // default min rotation angle
int servo13Angle = 0;       // default min rotation angle
int servo12Centre = 90;     // default centred rotation angle
int servo13Centre = 90;     // default centred rotation angle

// the following are for transferring commands to the remote controller and receiving data back
// the ESP32 delays sending a command to the remote until it receives rtr signal. Whenever it is busy
// then the rtr is set off locally indicating that the remote should not start a transmission, after
// checking that a transmission is not in progress. This is just to make life less complicated!

int rtrLocal = 12;   // esp32 sets this high to indicate that its listening on the serial port
int rtrRemote = 13;  // remote controller sets this high to indicate its ready to recieve transmission



// the main startup routine
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  //disable brownout detector
                                              //  #if INDEX_HTML == BASIC

  // Turn off flash
  pinMode(FLASH_PIN, OUTPUT);
  digitalWrite(FLASH_PIN, LOW);
  flashState = 0;

  // initialise builtin led for status indicator
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  if (debugSerial) { Serial.begin(115200); }
  if (debugSerial) { Serial.setDebugOutput(false); }

  //delay(5000);
  if (debugSerial) { Serial.println("Starting...."); }
  if (debugSerial) { Serial.println(String(__FILE__) + " " + String(__DATE__) + " " + String(__TIME__)); }
  //delay(5000);

  // Writing a high output on pins 12 and 13 is done to resolve an "error" in 1-bit mode when SD_MMC.begin("/sdcard", true).
  pinMode(12, OUTPUT);
  digitalWrite(12, 1);
  pinMode(13, OUTPUT);
  digitalWrite(13, 1);

  // get the sd card data
  // first setup SD card access

  setup_sdcard();

  // read SD card data to get initial configuration, wifi or ps3 data and default webpage if needed

  read_SDcard();

  // reset the pin 12 and 13 fixes
  pinMode(12, INPUT_PULLDOWN);
  pinMode(13, INPUT_PULLDOWN);


  // use camera defines to setup cammera access
  if (execCamera) {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.frame_size = FRAMESIZE_SVGA;
    config.pixel_format = PIXFORMAT_JPEG;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.jpeg_quality = 12;
    config.fb_count = 2;

    // Camera init
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
      if (debugSerial) { Serial.printf("Camera init failed with error 0x%x", err); }
      execCamera = 0;
    }
  }

  // implememt 2 pin motor control
  if (execMotor == 1) {
    if (debugSerial) { Serial.printf("Implementing 2 pin motor control on pins %d and %d \n", rightMotorPin, leftMotorPin); }
    initPWM();
  }

  // implememt 3 pin motor control
  if (execMotor == 2) {
    if (debugSerial) { Serial.printf("Implementing 3 pin motor control with L293D on pins %d and %d \n", rightMotorPin, leftMotorPin, enableMotorPin); }
    initThreePin();
  }

  // initialise servos
  if (execServo == 1 and execMotor == 0) {
    // initialise pan/tilt servo mode
    // init servo12 and servo13
    initServos();
  }

  // read and process startup script file
  scriptStatus = 1;
  strcpy(scriptFile, configStartup);
  while (scriptStatus) {
    fileProcessor(0);
  }

  // if WiFi selected by option, process connection
  if (execWiFi) {
    // Wi-Fi connection
    //  WiFi.config(local_IP,gateway,subnet,primaryDNS,secondaryDNS);
    WiFi.setHostname(configHostname);
    // if not APT then setup a wifi client
    if (execWiFi == 1) {
      WiFi.begin(ssid, password);
      while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        if (debugSerial) { Serial.print("."); }
      }
      if (debugSerial) { Serial.println(""); }
      Serial.println("WiFi connected");
    } else {  // otherwise create a wifi hot spot access point
      if (debugSerial) { Serial.println("Configuring access point..."); }
      WiFi.softAP(ssid, password);
      IPAddress myIP = WiFi.softAPIP();
      if (debugSerial) {
        Serial.print("AP IP address: ");
        Serial.println(myIP);
      }
    }

    // Start streaming web server
    startCameraServer();

    if (debugSerial) {
      Serial.print("Camera Stream Ready! Go to: http://");
      Serial.println(WiFi.localIP());
    }

    // register the hostname for use with DNS
    if (!MDNS.begin(configHostname)) {
      if (debugSerial) { Serial.println("Error starting mDNS"); }
    }
  } else if (debugSerial) {
    Serial.println("WiFi and Web page services not selected");
  }

  // PS3 setup routines
  if (execPS3) {
    Ps3.attach(notify);
    Ps3.attachOnConnect(onConnect);
    Ps3.begin(configPS3);
  }

  // Setup for processing main script file

  scriptStatus = 1;
  strcpy(scriptFile, configMain);
}

// the loop function flashes the internal LED on and off when connected to Wifi and
// schedules the reading of command files
void loop() {


  loopTimer = millis();  // all functions on each iteration of the loop use this value

  delay(10);  // delay to slow/stop race contitions in testing

  // display Wifi is active via flashing LED
  if (execWiFi) {
    // wifi indicator LED
    if (WiFi.status() == WL_CONNECTED) {
      if (ledTimer <= loopTimer) {
        ledTimer = loopTimer + 1000;
        digitalWrite(LED_BUILTIN, HIGH);
      }
      if (ledTimer - 500 <= loopTimer) {
        digitalWrite(LED_BUILTIN, LOW);
      }
    } else {
      digitalWrite(LED_BUILTIN, LOW);
    }
  }

  // switch off flash led if time expired
  if (flashTimer < loopTimer) {
    cmdProcessor("LFLASHOFF", 0);
    flashTimer = FLASH_ON_TIME;
  }

  // process script file if active
  fileProcessor(1);

  // take script out of pause if timer expired
  if (scriptProcess == 3) {
    //if scriptTimer has expired, set script to process next command
    if (scriptTimer <= loopTimer) {
      if (debugSerial) { Serial.println("Script pause ended"); }
      scriptProcess = 2;
    }
  }

  // check if motor command has expired
  // if so, stop robot
  if (execMotor && execPWM) {
    if (motorTimer <= loopTimer) {
      ledcWrite(rightMotorPWMSpeedChannel, 0);
      ledcWrite(leftMotorPWMSpeedChannel, 0);
    }
  }

  // Video processing routines



  // end of loop routine
}
