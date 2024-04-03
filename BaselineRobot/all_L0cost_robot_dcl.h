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
int localProcessor(char localVariable[], int source);
void initPWM();
int motorControl(char localVariable[], int source);
int shiftLeft(char variable[], int step);
int initServos(void);
int servoControl(char localVariable[]);
void fileProcessor(int type);
int serialProcessor(int type);
int getValue(char fullCommand[], int offset);
int cmdCmp(char fullCommand[], char testCommand[]);
int cameraControl(char localVariable[]);
int guideControl(char localVariable[]);
void shiftFileName(char *fileName);
void detachPWM(void);
void attachPWM(void);
void initThreePin(void);
int ps3Processor(char ps3Command[], int source);
int htmlProcessor(char htmlCommand[], int source);
int lockSet(int &lock, int value);
int lockUnSet(int &lock, int value);
int lockTest(int &lock, int value);
void picoPinOn();
void picoPinOff();

// video handling routines
int videoProcessing(void);
int setVideoMode();
int setGuideMode();
long findLineCentre(long colour);
long findLineTop(long colour);
long findLineBottom(long colour);
long findLine(long lineInstances[13], long colour, long startLine, long startCol, long endCol, long threshold, long adjacent, long column, long update);
long findBlobInstances(long colour);  // call findBlob routine
long findBlob(int colour);
int scanBlob(int pixLine, int pixCol);
int scanTest(long colour);
int setPixel(long location, int colour);
long heatMap(long colour, int background);
long findSquare(long colour);
long findColours(void);
int crossHairs(long colour);
int graduations(long colour);
int percentGrads(long colour);
long detectColour(long pixel, int colour);
long pixelLuminance(int type);
int procGuidance(int type);
long pixelInvert(void);
long pixelPreprocess(int type1, int type2, int type3);

// PS3 routines
void onConnect();
void notify();

// demo functions
int demoControl(char localVariable[]);
int lineFollower(void);
int blobFollower(void);
int targetFollower(void);

// html routines
static esp_err_t index_handler(httpd_req_t *req);
static esp_err_t stream_handler(httpd_req_t *req);
static esp_err_t cmd_handler(httpd_req_t *req);
static esp_err_t data_handler(httpd_req_t *req);
void startCameraServer();

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define WIFI 1
#define CMDFILE 2
#define CONFIG 3
#define PS3CONFIG 4


#define FLASH_PIN 4
#define LED_BUILTIN 33

// demo function variables
int factorP = 1;    // Proportional PID factor. This is to be interpreted as a 2.2 decimal number
int factorI = 0;    // Integral PID factor. This is to be interpreted as a 2.2 decimal number
int factorD = 0;    // Derivative PID factor. This is to be interpreted as a 2.2 decimal number

// debug variables
int webSwitch = 1;


// execution flags
int execWiFi = 0;            // use wifi, value indicates 0= no wifi, 1=client or 2=access point
int execPS3 = 0;             // use PS3 input
int execSensor = 0;          // use serial pins for local sensor input
int execSerial = 1;          // configure and pass output to serial port
int execMotor = 0;           // configure motor control output
int execHandshake = 0;       // pins configured for remote controller serial handshake
int execServo = 0;           // configure servo control output, valu indicates either 2 or 4 pins
int execPWM = 0;             // motor control attached
int execPolarity = 0;        // motor polarity, 0 = negative, 1 = positive
int execVideo = 1;           // activate video streaming
int execGuide = 0;           // activate video guidance
int execScript = 1;          // activate script processing
int execLine = 0;            // activate line following video processing
int execLineProcessing = 0;  // type of line processing to carry out 0 = centre, 1 = centre and bottom, 2 = top, centre and bottom
int execBlob = 0;            // activate blob following video processing
int execBlobProcessing = 0;  // activate type of blob processing, 0 = standard
int execOverlay = 0;         // add overlay to guide video display
int execPico = 0;            // activates bidirectional serial protocal with Raspberry Pi Pico

int execDemo = 0;           // set if demo activity selected
int execFollower = 0;       // set if line follower demonstration used
int execBlobFollower = 0;   // set if blob follower demonstration used
int execTargetFollower = 0; // set if blob tracker demonstration used

// demo processing
int demoStatus = 0;         // maintains demo status 0 = no action, 1 = calibrate, 2 = follow


// pico processing section
char sequenceNumber[5] = "    ";  // pico sends a sequence number which must be returned as header to response
int picoPin = 13;                 // hardware handshake pin


// script file processing
char scriptFile[20] = "/startup.txt";       // default script name
int scriptStatus = 0;                      // maintains status of current script execution
int scriptProcess = 0;                      // tracks current script process
char scriptBuf[200];                        // buffer for script processing
File script;                                // file definition for script processing

// timer counters
long loopTimer = 0;     // main timer variable for loop
long ledTimer = 0;
long runTimer = 0;
long scriptTimer = 0;
int errorTimer = 500;   // led flash delay 500ms is for wifi connected ok, 250ms of for camera not working

// timers for motor control
long motorTimer = 0;          // timeout for last motor command
long directTimer = 0;         // timeout for direct control
long defaultTimeOut = 30000;  // default set to 30 seconds
long directTimeOut = 30000;

// flash led timer definitions
#define FLASH_ON_TIME 99999999  // flash LED timer set to 10,000 seconds, could be more but just sets a practical limit
#define FLASH_PEEK_TIME 5000
long flashTimer = FLASH_ON_TIME;
int flashIntensity = 128;  // default PWM value for flash LED when illuminated
const int flashPWMChannel = 7;

// configuration defaults and variables
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

// HTML messaging
char msg[60] = "{\"type\":null}";
int msgLen = 60;
int htmlFlag = 2;



// locks
int cmdLock = 0;


// serial port controls
int essentialSerial = 0;  // if 1 turns on essential serial prints
int debugSerial = 1;      // if 1 turns on only debug messages
char serialCommand[40];   // command received from serial input
int serialPointer = 0;
int serialLimit = 33;  // max length of serial command
int commandCount = 0;  // number of consecutive serial commands executed
int commandLimit = 3;  // limit of consecutive sequential commands executable


// definitions to run l0cost line follower with single ESP32-CAM
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

// video guidance processing
size_t out_len, out_width, out_height;
uint8_t *out_buf;
fb_data_t guideFrame;
int frameAvailable = 0;  // if 0, all frameprocessing complete
                         // if 1, frame processing successful
                         // if 2, get frame in progress
                         // if 3, frame processing error
                         // if 4,
                         // if 5,

int lineLuminence = 1275;
long pixLuminence = 0;
int lineContrast = 2;
int greenThreshold = 125;
int redThreshold = 125;
int blueThreshold = 125;
int brightnessAdjust = 100;
int updateColour = 4;
int colourDetect = 0;
long lineThresholdT = 300;
long lineThresholdC = 300;
long lineThresholdB = 300;
long blobThreshold = 32;
long lineInstancesT[7] = { 0, 0, 0, 0, 0, 0, 0 };
long lineInstancesC[7] = { 0, 0, 0, 0, 0, 0, 0 };
long lineInstancesB[7] = { 0, 0, 0, 0, 0, 0, 0 };
long blobInstances[20][5];
long blobMask[4] = { 0, 0, 0, 0 };  // definition of frame buffer where objects ignored
// the following variables have been created as a stop gap to multi=blob detection and reporting
int lastBlobCount = 0;  // last blob pixel count
int lastBlobX = 0;      // last blob geometric centre X co-ordinate
int lastBlobY = 0;      // last blob geometric centre Y co-ordinate
int lookAhead = 20;

// find coloured line
#define BLACK 0
#define RED 1
#define ORANGE 2
#define YELLOW 3
#define GREEN 4
#define BLUE 5
#define TURQ 6
#define VIOLET 7
#define WHITE 10

int colourTranslate[3][11] = { { -10, -10, -10, -10, -10, 255, 0, 255, 0, 0, 255 },    // blue
                               { -10, -10, 165, 255, 255, -10, 0, -10, 0, 0, 255 },    // green
                               { -10, 255, 255, 255, -10, -10, 0, 127, 0, 0, 255 } };  // red

// typedef struct {
//     uint8_t * buf;              /*!< Pointer to the pixel data */
//     size_t len;                 /*!< Length of the buffer in bytes */
//     size_t width;               /*!< Width of the buffer in pixels */
//     size_t height;              /*!< Height of the buffer in pixels */
//     pixformat_t format;         /*!< Format of the pixel data */
//     struct timeval timestamp;   /*!< Timestamp since boot of the first DMA buffer of the frame */
// } camera_fb_t;

int printData = 0;  // test variable for printing line data
