// configuration processing
// these routines read the SD card files to setup the operation of the card

// file parse routine to obtain data from SD card files
void fileParse(char *line, int fileType, int lineNo) {
  String workingString;
  switch (fileType) {
    case WIFI:
      switch (lineNo) {
        case 1:
          line[32] = '\0';
          strcpy(ssid, line);
          if (debugSerial) { Serial.printf("SSID = %s\n", ssid); }
          break;
        case 2:
          line[32] = '\0';
          strcpy(password, line);
          if (debugSerial) { Serial.printf("Password = %s\n", password); }
          break;
        case 3:
          line[19] = '\0';
          strcpy(htmlFile, line);
          if (debugSerial) { Serial.printf("Web page = %s\n", htmlFile); }
          break;
        case 4:
          line[19] = '\0';
          workingString = String(line);
          configWebPort = workingString.toInt();
          if (debugSerial) { Serial.printf("Web port = %d\n", configWebPort); }
          break;
        case 5:
          line[19] = '\0';
          workingString = String(line);
          configStreamPort = workingString.toInt();
          if (debugSerial) { Serial.printf("Video stream port = %d\n", configStreamPort); }
          break;
      }
      break;
    case CONFIG:
      switch (lineNo) {
        case 1:
          line[19] = '\0';
          strcpy(configType, line);
          if (debugSerial) { Serial.printf("Execution Type = %s\n", configType); }
          flagSet(configType);
          flagReport();
          break;
        case 2:
          line[19] = '\0';
          strcpy(configHostname, line);
          if (debugSerial) { Serial.printf("Hostname = %s\n", configHostname); }
          break;
        case 3:
          line[19] = '\0';
          if (execWiFi) {
            strcpy(configWifi, line);
            if (debugSerial) { Serial.printf("WiFi config file = %s\n", configWifi); }
          }
          if (execPS3) {
            strcpy(configPS3File, line);
            if (debugSerial) { Serial.printf("PS3 config file = %s\n", configPS3File); }
          }
          break;
        case 4:
          line[19] = '\0';
          strcpy(configStartup, line);
          if (debugSerial) { Serial.printf("Startup Script file = %s\n", configStartup); }
          break;
        case 5:
          line[19] = '\0';
          strcpy(configMain, line);
          if (debugSerial) { Serial.printf("Main Script file = %s\n", configMain); }
          break;
      }
      break;
    case CMDFILE:
      cmdProcessor(line, 4);

      break;

    case PS3CONFIG:
      switch (lineNo) {
        case 1:
          line[19] = '\0';
          strcpy(configPS3, line);
          if (debugSerial) { Serial.printf("PS3 Address = %s\n", configPS3); }
          break;
        default:
          line[19] = '\0';

          lineNo = lineNo - 2;
          if (lineNo <= 47) {
            if (line[0] == '0') {
              ps3Translate[lineNo].value[0] = '\0';
            } else {
              for (int i = 0; i < 19; i++) {
                if (line[i] == ' ') {
                  ps3Translate[lineNo].value[i] = '\0';
                  break;
                } else {
                  ps3Translate[lineNo].value[i] = line[i];
                }
              }
            }
            Serial.println(ps3Translate[lineNo].value);
          }
      }
      break;
  }
}

// routine to read a data file and parse according to the type ----------------------------------
int read_datafile(fs::FS &fs, const char *path, int fileType) {
  char buf[200];
  int linecount = 0;
  char fileName[33] = "/";
  if (path[0] != '/') {
    for (int i = 1; i <= 32; i++) {
      fileName[i] = path[i - 1];
    }
  } else strcpy(fileName, path);

  if (debugSerial) { Serial.printf("Reading file: %s\n", fileName); }

  File file = fs.open(fileName);
  if (!file) {
    if (debugSerial) { Serial.println("Failed to open file for reading"); }
    if (debugSerial) { Serial.printf("Failed to open file %s for reading\n", fileName); }
    return 1;
  }

  while (file.available()) {
    int l = file.readBytesUntil('\n', buf, sizeof(buf));
    if (l > 0 && buf[l - 1] == '\r') {
      l--;
    }
    buf[l] = 0;
    // if input line starts with '//' then don't process as a line, taken as a comment.
    if (!(buf[0] == ' ' || (buf[0] == '/' && buf[1] == '/'))) {
      linecount++;
      fileParse(buf, fileType, linecount);
    }
  }

  if (debugSerial) { Serial.println("File closed"); }
  file.close();
  return 0;
}

// routine to setup SD card for file attachments ----------------------------------
int setup_sdcard() {
  if (debugSerial) { Serial.println("Mount SD card"); }
  //******1-bit mode

  if (!SD_MMC.begin("/sdcard", true)) {
    if (debugSerial) { Serial.println("Card Mount Failed"); }
    return 1;
  }

  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    if (debugSerial) { Serial.println("No SD card attached"); }
    return 1;
  }
  return 0;
}

// routine to load html page into variable
int readHtml(fs::FS &fs, const char *htmlf) {

  // get the file name
  // list the root directory checking for a filename match
  // on match, get the file size
  // allocate (or reallocate) the buffer to the size of file + 10 bytes
  // close files

  int linecount = 0;
  long htmlSize = 0;
  char htmlOpen[21] = "/";

  if (debugSerial) { Serial.printf("Reading file: %s\n", htmlf); }

  File root = fs.open("/");

  if (!root) {
    if (debugSerial) { Serial.println("Failed to open directory"); }
    return 3;
  }
  if (!root.isDirectory()) {
    if (debugSerial) { Serial.println("Not a directory"); }
    return 2;
  }

  File file = root.openNextFile();
  while (file) {
    if (!file.isDirectory()) {
      if (debugSerial) { Serial.printf("  FILE:  %s   SIZE: %d  \n", file.name(), file.size()); }
      if (strcmp(htmlf, file.name()) == 0) {
        htmlSize = file.size();
        if (debugSerial) { Serial.printf("Match found: %d\n", htmlSize); }
        break;
      }
    }
    file = root.openNextFile();
  }
  file.close();
  root.close();
  for (int i = 1; i < 20; i++) {
    htmlOpen[i] = htmlf[i - 1];
  }
  if (debugSerial) { Serial.printf("Reading file: %s\n", htmlOpen); }
  File page = fs.open(htmlOpen);
  if (!page) {
    if (debugSerial) { Serial.println("Failed to open file for reading"); }
    return 1;
  }
  htmlPage = (char *)malloc(htmlSize + 10);

  page.readBytes(htmlPage, htmlSize);
  htmlPage[htmlSize] = 0;
  if (debugSerial) { Serial.println("File closed"); }
  page.close();
  return 0;
}

// routine to align filename with / for use with SD card access
void shiftFileName(char *fileName) {
  int nameLength = strlen(fileName);
  if (fileName[0] == '/') return;
  for (int i = 20; i <= 0; i--) {
    fileName[i] = fileName[i - 1];
  }
  fileName[0] = '/';
}

// startup routine to get config details and then wifi and email details ----------------------------------
void read_SDcard(void) {
  if (debugSerial) { Serial.println("Read data file"); }
  read_datafile(SD_MMC, configConfig, CONFIG);

  if (execWiFi) {
    if (read_datafile(SD_MMC, configWifi, WIFI)) {
      execWiFi = 0;
    }
  }
  if (execPS3) {
    if (read_datafile(SD_MMC, configPS3File, PS3CONFIG)) {
      execPS3 = 0;
    }
  }
  if (execWiFi) {
    if (readHtml(SD_MMC, htmlFile)) {
      // load default basic page if SD based page doesn't load
      htmlPage = (char *)malloc(strlen(BASIC) + 10);
      strcpy(htmlPage, BASIC);
    }
  }
}

// routine to verify that filename exists on SD card
// this is a copy of the htlml load code and has not yet been customised 
// for the function
int checkFile(fs::FS &fs, const char *chkFile) {

  int linecount = 0;
  int retVal = 1;

  if (debugSerial) { Serial.printf("Seeking file: %s\n", chkFile); }

  File root = fs.open("/");
  if (!root) {
    if (debugSerial) { Serial.println("Failed to open directory"); }
    return 3;
  }
  if (!root.isDirectory()) {
    if (debugSerial) { Serial.println("Not a directory"); }
    return 2;
  }

  File file = root.openNextFile();
  while (file) {
    if (!file.isDirectory()) {
      if (debugSerial) { Serial.printf("  FILE:  %s   SIZE: %d  \n", file.name(), file.size()); }
      if (strcmp(chkFile, file.name()) == 0) {
        retVal = 0;
        if (debugSerial) { Serial.println("Match found"); }
        break;
      }
    }
    file = root.openNextFile();
  }
  file.close();
  root.close();

  return retVal;
}
