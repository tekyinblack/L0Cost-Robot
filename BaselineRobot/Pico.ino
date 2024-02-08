// Bidirectional communications with Raspebery Pi platform running Python
/*
This code was introduced to facilitate the use of the L0Cost Robot architecture in the PiWars 
competition where one of the Raspeberry Pi platforms must be used. The L0Cost platform operates
in a subservient role to the Pi or Pico being used.
*/
/*
This code adds funtion to the command processing and response by adding the following features

The first 4 characters in a message are a serial number received from the Pi platform which are returned to the 
Pi unchanged in the response, to synchronise messages.
Any response will include a 4 character field after the serial number indicating the success of the message action.
Only OKOK indicates that the message action was successful.
*/
int picoProcessor(char picoCommand[], int source) {
  int retVal = 0;
  int pauseValue = 0;
  // store sequence number, basic but effective ;)
  sequenceNumber[0] = picoCommand[0];
  sequenceNumber[1] = picoCommand[1];
  sequenceNumber[2] = picoCommand[2];
  sequenceNumber[3] = picoCommand[3];
  // left shift command to obtain command
  {
    int i;
    for (i = 1; i < strlen(picoCommand); i++) {
      if (i > 30) break;                 // break if longer than allowed
      if (picoCommand[i] == ' ') break;  // break if blank character
      picoCommand[i - 4] = picoCommand[i];
    }
    picoCommand[i] = '\0';
  }
  if (cmdCmp(picoCommand, "WHOU") == 0) {  // command is WHOU, respond with name of controller
    Serial.printf("%s%s\n", sequenceNumber, configHostname);

  } else if (cmdCmp(picoCommand, "LINT") == 0) {  // respond with current line following readings
    // sequence is top line, centre line and bottom line refering to the lines in the camera image with 4 characters each and leading zeros
    Serial.printf("%sOKOK%04d%04d\n", sequenceNumber, lineInstancesT[1],lineInstancesT[2]);

  } else if (cmdCmp(picoCommand, "LINC") == 0) {  // respond with current line following readings
    // sequence is top line, centre line and bottom line refering to the lines in the camera image with 4 characters each and leading zeros
    Serial.printf("%sOKOK%04d%04d\n", sequenceNumber, lineInstancesC[1],lineInstancesC[2]);
    
  } else if (cmdCmp(picoCommand, "LINB") == 0) {  // respond with current line following readings
    // sequence is top line, centre line and bottom line refering to the lines in the camera image with 4 characters each and leading zeros
    Serial.printf("%sOKOK%04d%04d\n", sequenceNumber, lineInstancesB[1],lineInstancesB[2]);

  } else if (cmdCmp(picoCommand, "BLOB") == 0) {  // respond with current blob location
    // sequence is relative x co-ord, relative y co-ord with 4 characters each and leading zeros
    Serial.printf("%sOKOK%04d%04d\n", sequenceNumber, configHostname);

  } else {  // assume its a motor control command, and if not, unknown command
    Serial.printf("%sBADC - %s\n", sequenceNumber, picoCommand);
  }
  return retVal;
}
