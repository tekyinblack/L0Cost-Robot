/*
Version 01_06C adds enhancements to support video control for line following and target tracking
*/
/* With debug set on, no local pin assignments and serial is assumed to be used for debug output only
All other functions continue to work
*/
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

#include "all_L0cost_robot_dcl.h"
long lastconv = 0;

// the main startup routine
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  //disable brownout detector

  // Turn off flash
  pinMode(FLASH_PIN, OUTPUT);
  digitalWrite(FLASH_PIN, LOW);
  flashState = 0;
  // setup flash LED control
  ledcSetup(flashPWMChannel, PWMFreq, PWMResolution);
  ledcAttachPin(FLASH_PIN, flashPWMChannel);
  ledcWrite(flashPWMChannel, 0);

  // initialise builtin led for status indicator
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  if (debugSerial || execSerial) { Serial.begin(115200); }
  if (debugSerial) { Serial.setDebugOutput(false); }

  if (debugSerial) { Serial.println("Starting...."); }
  if (debugSerial) { Serial.println(String(__FILE__) + " " + String(__DATE__) + " " + String(__TIME__)); }

  // Writing a high output on pins 12 and 13 is done to resolve an "error" in 1-bit mode when SD_MMC.begin("/sdcard", true).
  // this may result in brief unpredictable servo or motor behaviour
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
  if (execVideo || execGuide) {
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
    config.frame_size = FRAMESIZE_SVGA;    // default for video streaming
    config.pixel_format = PIXFORMAT_JPEG;  // default for video streaming
    if (execGuide) {
      config.frame_size = FRAMESIZE_QVGA;  // default for video guidance
    }

    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    //config.jpeg_quality = 12;
    config.jpeg_quality = 2;
    config.fb_count = 2;

    // Camera init
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
      if (debugSerial) { Serial.printf("Camera init failed with error 0x%x \n", err); }
      execVideo = 0;
      execGuide = 0;
      errorTimer = 250;
    }
    if (execGuide) {
      sensor_t *s = esp_camera_sensor_get();
      s->set_framesize(s, (framesize_t)2);
      // s->set_special_effect(s, 2);
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
  if (execScript) {
    // read and process startup script file
    scriptStatus = 1;
    strcpy(scriptFile, configStartup);
    while (scriptStatus) {
      fileProcessor(0);
    }
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
  if (execScript) {
    scriptStatus = 1;
    strcpy(scriptFile, configMain);
  }



  //************ end of setup
}

// the loop function flashes the internal LED on and off when connected to Wifi and
// schedules the reading of command files
void loop() {

  if (debugSerial) {
    // delay(10);  // delay to slow/stop race contitions in testing
  }

  long startconv = 0;
  long endconv = 0;
  long enddetect = 0;
  long endoverlay = 0;
  long endloop = 0;
  long convgap = 0;
  loopTimer = millis();  // all functions on each iteration of the loop use this value

  // display Wifi is active via flashing LED
  if (execWiFi) {
    // wifi indicator LED
    if (WiFi.status() == WL_CONNECTED) {
      if (ledTimer <= loopTimer) {
        ledTimer = loopTimer + errorTimer * 2;
        digitalWrite(LED_BUILTIN, HIGH);
      }
      if (ledTimer - errorTimer <= loopTimer) {
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

  // check for and process serial input
  // this is run continuously until serial processing is complete or an exception occurs
  // out of input data, excessive input data and function not active are exceptions
  while (!serialProcessor(0)) {};

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
      if (execMotor == 1) {
        ledcWrite(rightMotorPWMSpeedChannel, 0);
        ledcWrite(leftMotorPWMSpeedChannel, 0);
      } else if (execMotor == 2) {
        ledcWrite(enableMotorPWMSpeedChannel, 0);
      }
    }
  }

  // Video processing routines
  // any incuded print statements are for development testing and will be removed in a later version
  if (execGuide) {
    // only execute this section if automatic video guidance in use
    if (!frameAvailable) {
      //     Serial.printf("Start conv: %d \n", millis());
      // get frame into working frame buffer
      startconv = millis();
      convgap = lastconv - startconv;
      lastconv = startconv;
      videoProcessing();
      endconv = millis();
      //    Serial.printf("End conv: %d \n", millis());
    }

    // if frame available, process selected detection routines
    if (frameAvailable == 1) {
      // call line detection processing. This is currently only setup for white line on black background
      // three frame positions are provided here, top, centre and bottom, and three 'lines' can detected and reported on in each
      // the position of top, centre and bottom can be defined in commands to suit the camera
      // by default, only the centre position is updated, selecting BOTTOM updates both centre and bottom, and selecting
      // TOP updates top, centre and bottom
      if (execLine) {
        // // for black processing, invert frame
        // if (colourDetect == 0) {
        //   pixelInvert();
        // }
        switch (execLineProcessing) {
          case 1:
            findLineTop(colourDetect);
          case 2:
            findLineBottom(colourDetect);
          default:
            findLineCentre(colourDetect);
        }
      }
      // call blob detection processing
      // the initial reporting is targeted on the centre of the largest blob detected
      else if (execBlob) {
        long videoTime = millis();
        switch (execBlobProcessing) {
          case 0:
            // standard blob processing, updates blob location variables
            findBlob(colourDetect);
            break;
          case 1:
            // paints a heatmap of colour for video guidance tuning
            heatMap(colourDetect, 1);
            break;
          case 2:
            // paints a colour intensity map  for video guidance tuning
            scanTest(colourDetect);
            break;
          case 3:
            // paints a colour detection map for video guidance tuning
            findColours();
            break;
          default:
            findBlob(colourDetect);
            break;
        }
        // if (debugSerial) { Serial.printf("%d \n", millis() - videoTime); }
      }

      enddetect = millis();

      // end of successful processing, set frame available for processing elsewhere
      // or flag frame processing complete and free frame buffer

      if (execVideo) {
        switch (execOverlay) {
          case 1:
            crossHairs(BLUE);
            break;
          case 2:
            graduations(BLUE);
            break;
          case 3:
            percentGrads(BLUE);
            break;
        }
        frameAvailable = 5;
      } else {
        frameAvailable = 0;
        if (guideFrame.data) {
          free(guideFrame.data);
          guideFrame.data = NULL;
        }
        // free(guideFrame.data);
        //guideFrame.data = NULL;
      }
    }
  }
  endoverlay = millis();
  // execute demo functions if selected
  if (execDemo) {
    if (execFollower) {
      // call line follower demo routine
      lineFollower();
    } else if (execBlobFollower) {
      // call bllob follower routine
      blobFollower();
    } else if (execTargetFollower) {
      // call target follower routine
      targetFollower();
    } else execDemo = 0;
  }
  endloop = millis();
  if (startconv) {
    if (debugSerial) {Serial.printf("Start loop = %d, Conversion = %d Detect = %d, Overlay = %d, Loop = %d  Interval = %d\n", loopTimer, endconv - startconv, enddetect - endconv, endoverlay - enddetect, endloop - loopTimer, convgap); }
  }
  // end of loop routine
}
