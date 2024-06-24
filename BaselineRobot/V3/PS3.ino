// PS3 control
/* The routines here wait for input from the PS3 bluetooth interface and translate them into cammands 
    to be processed. The x and y controls dupicate this function to allow these PS3 controls to be used
    both internally and externally, such as passing motor drive commands to both internal motor drivers and
    external ones where four wheel motor control is needed or ackermann steering is implemented.
    */
// PS3 signal translation table
// this provides a lookup for what each PS3 controller signal passed to the command processor
// is prefixed with
struct ps3Items {
  char value[20];
};
// default PS3 translations
struct ps3Items ps3Translate[48] = {
  { "LMTR" },  // left and right joystick values ( L is for internal only}
  { "XBJV" },  // left and right joystick values
  { "XLJP" },  // left joystick pressed
  { "XLJR" },  // left joystick released
  { "XRJP" },  // right joystick pressed
  { "XRJR" },  // right joystick released
  { "XUPV" },  // up button values
  { "XUBP" },  // up button pressed
  { "XUBR" },  // up button released
  { "XRBV" },  // right button values
  { "XRBP" },  // right button pressed
  { "XRBR" },  // right button released
  { "XDBV" },  // down button values
  { "XDBP" },  // down button pressed
  { "XDBR" },  // down button released
  { "XLBV" },  // left button values
  { "XLBP" },  // left button pressed
  { "XLBR" },  // left button released
  { "XXBV" },  // cross button values
  { "XXBP" },  // cross button pressed
  { "XXBR" },  // cross button released
  { "XSBV" },  // square button values
  { "XSBP" },  // square button pressed
  { "XSBR" },  // square button released
  { "XTBV" },  // triangle button values
  { "XTBP" },  // triangle button pressed
  { "XTBR" },  // triangle button released
  { "XCBV" },  // circle button values
  { "XCBP" },  // circle button pressed
  { "XCBR" },  // circle button released
  { "XLSV" },  // left shoulder values
  { "XLSP" },  // left shoulder pressed
  { "XLSR" },  // left shoulder released
  { "XRSV" },  // right shoulder values
  { "XRSP" },  // right shoulder pressed
  { "XRSR" },  // right shoulder released
  { "XLTV" },  // left trigger values
  { "XLTP" },  // left trigger pressed
  { "XLTR" },  // left trigger released
  { "XRTV" },  // right trigger values
  { "XRTP" },  // right trigger pressed
  { "XRTR" },  // right trigger released
  { "XSLP" },  // select button pressed
  { "XSLR" },  // select button released
  { "XSTP" },  // start button pressed
  { "XSTR" },  // start button released
  { "XPSP" },  // PS3 button pressed
  { "XPSR" }   // PS3 button released
};
// These routines provide PS3 input to the card
// PS3 callback routine
void notify() {
  char sendCommand[30];
  // if execMotor is set, then direction commands are routed to the local processor
  // if execSerial is set, then all commands routed to remote, except where execMotor holds
  // if ( abs(Ps3.event.analog_changed.stick.rx) + abs(Ps3.event.analog_changed.stick.ry) > 2 ) {

  //   if (Ps3.data.analog.stick.ry < 0) {
  //     cmdProcessor("LR000004001000001",3);
  //   }
  //   else if (Ps3.data.analog.stick.ry == 0) {
  //     cmdProcessor("LR0000000001000001",3);
  //   }
  //   else if (Ps3.data.analog.stick.ry > 0) {
  //     cmdProcessor("LR0180004001000001",3);
  //   }
  // }
  //---------------- Analog stick value events ---------------
  // These are treated differently due to their operational use. Both sticks are seen to operate in conjunction
  // even if not specified in the translate table. This generates a control string of four values which can be sent to
  // two different destinations
  if (abs(Ps3.event.analog_changed.stick.lx) + abs(Ps3.event.analog_changed.stick.ly) + abs(Ps3.event.analog_changed.stick.rx) + abs(Ps3.event.analog_changed.stick.ry) > 2 && ps3Translate[0].value[0]) {
    if (debugSerial) {
      Serial.print("Moved the left stick:");
      Serial.print(" x=");
      Serial.print(Ps3.data.analog.stick.lx, DEC);
      Serial.print(" y=");
      Serial.print(Ps3.data.analog.stick.ly, DEC);
      Serial.println();
      sprintf(sendCommand, "%s%04d%04d%04d%04d",
              ps3Translate[0].value,
              Ps3.data.analog.stick.lx,
              Ps3.data.analog.stick.ly,
              Ps3.data.analog.stick.rx,
              Ps3.data.analog.stick.ry);
      cmdProcessor(sendCommand, 3);
    }
  }

  if (abs(Ps3.event.analog_changed.stick.lx) + abs(Ps3.event.analog_changed.stick.ly) + abs(Ps3.event.analog_changed.stick.rx) + abs(Ps3.event.analog_changed.stick.ry) > 2 && ps3Translate[1].value[0]) {
    if (debugSerial) {
      Serial.print("Moved the right stick:");
      Serial.print(" x=");
      Serial.print(Ps3.data.analog.stick.rx, DEC);
      Serial.print(" y=");
      Serial.print(Ps3.data.analog.stick.ry, DEC);
      Serial.println();
      sprintf(sendCommand, "%s%04d%04d%04d%04d",
              ps3Translate[1].value,
              Ps3.data.analog.stick.lx,
              Ps3.data.analog.stick.ly,
              Ps3.data.analog.stick.rx,
              Ps3.data.analog.stick.ry);
      cmdProcessor(sendCommand, 3);
    }
  }

  //--------------- Digital stick button events --------------
  if (Ps3.event.button_down.l3 && ps3Translate[2].value[0]) {
    Serial.println("Started pressing the left stick button");
    cmdProcessor(ps3Translate[2].value, 3);
  }
  if (Ps3.event.button_up.l3 && ps3Translate[3].value[0]) {
    if (debugSerial) { Serial.println("Released the left stick button"); }
    cmdProcessor(ps3Translate[3].value, 3);
  }

  if (Ps3.event.button_down.r3 && ps3Translate[4].value[0]) {
    if (debugSerial) { Serial.println("Started pressing the right stick button"); }
    cmdProcessor(ps3Translate[4].value, 3);
  }
  if (Ps3.event.button_up.r3 && ps3Translate[5].value[0]) {
    if (debugSerial) { Serial.println("Released the right stick button"); }
    cmdProcessor(ps3Translate[5].value, 3);
  }

  //--------------- Digital and Analog D-pad button events --------------
  if (abs(Ps3.event.analog_changed.button.up) && ps3Translate[6].value[0]) {
    if (debugSerial) {
      Serial.print("Pressing the up button: ");
      Serial.println(Ps3.data.analog.button.up, DEC);
    }
    sprintf(sendCommand, "%s%04d",
            ps3Translate[6].value,
            Ps3.data.analog.button.up);
    cmdProcessor(sendCommand, 3);
  }
  if (Ps3.event.button_down.up && ps3Translate[7].value[0]) {
    if (debugSerial) { Serial.println("Started pressing the up button"); }
    cmdProcessor(ps3Translate[7].value, 3);
  }
  if (Ps3.event.button_up.up && ps3Translate[8].value[0]) {
    if (debugSerial) { Serial.println("Released the up button"); }
    cmdProcessor(ps3Translate[8].value, 3);
  }

  if (abs(Ps3.event.analog_changed.button.right) && ps3Translate[9].value[0]) {
    if (debugSerial) {
      Serial.print("Pressing the right button: ");
      Serial.println(Ps3.data.analog.button.right, DEC);
    }
    sprintf(sendCommand, "%s%04d",
            ps3Translate[9].value,
            Ps3.data.analog.button.right);
    cmdProcessor(sendCommand, 3);
  }
  if (Ps3.event.button_down.right && ps3Translate[10].value[0]) {
    if (debugSerial) { Serial.println("Started pressing the right button"); }
    cmdProcessor(ps3Translate[10].value, 3);
  }
  if (Ps3.event.button_up.right && ps3Translate[11].value[0]) {
    if (debugSerial) { Serial.println("Released the right button"); }
    cmdProcessor(ps3Translate[11].value, 3);
  }

  if (abs(Ps3.event.analog_changed.button.down) && ps3Translate[12].value[0]) {
    if (debugSerial) {
      Serial.print("Pressing the down button: ");
      Serial.println(Ps3.data.analog.button.down, DEC);
    }
    sprintf(sendCommand, "%s%04d",
            ps3Translate[12].value,
            Ps3.data.analog.button.down);
    cmdProcessor(sendCommand, 3);
  }
  if (Ps3.event.button_down.down && ps3Translate[13].value[0]) {
    if (debugSerial) { Serial.println("Started pressing the down button"); }
    cmdProcessor(ps3Translate[13].value, 3);
  }
  if (Ps3.event.button_up.down && ps3Translate[16].value[0]) {
    if (debugSerial) { Serial.println("Released the down button"); }
    cmdProcessor(ps3Translate[14].value, 3);
  }

  if (abs(Ps3.event.analog_changed.button.left) && ps3Translate[15].value[0]) {
    if (debugSerial) {
      Serial.print("Pressing the left button: ");
      Serial.println(Ps3.data.analog.button.left, DEC);
    }
    sprintf(sendCommand, "%s%04d",
            ps3Translate[15].value,
            Ps3.data.analog.button.left);
    cmdProcessor(sendCommand, 3);
  }
  if (Ps3.event.button_down.left && ps3Translate[16].value[0]) {
    if (debugSerial) { Serial.println("Started pressing the left button"); }
    cmdProcessor(ps3Translate[16].value, 3);
  }
  if (Ps3.event.button_up.left && ps3Translate[17].value[0]) {
    if (debugSerial) { Serial.println("Released the left button"); }
    cmdProcessor(ps3Translate[17].value, 3);
  }


  //--- Digital and Analog cross/square/triangle/circle button events ---

  if (abs(Ps3.event.analog_changed.button.cross) && ps3Translate[18].value[0]) {
    if (debugSerial) {
      Serial.print("Pressing the cross button: ");
      Serial.println(Ps3.data.analog.button.cross, DEC);
    }
    sprintf(sendCommand, "%s%04d",
            ps3Translate[18].value,
            Ps3.data.analog.button.cross);
    cmdProcessor(sendCommand, 3);
  }
  if (Ps3.event.button_down.cross && ps3Translate[19].value[0]) {
    if (debugSerial) { Serial.println("Started pressing the cross button"); }
    cmdProcessor(ps3Translate[19].value, 3);
  }
  if (Ps3.event.button_up.cross && ps3Translate[20].value[0]) {
    if (debugSerial) { Serial.println("Released the cross button"); }
    cmdProcessor(ps3Translate[20].value, 3);
  }

  if (abs(Ps3.event.analog_changed.button.square) && ps3Translate[21].value[0]) {
    if (debugSerial) {
      Serial.print("Pressing the square button: ");
      Serial.println(Ps3.data.analog.button.square, DEC);
    }
    sprintf(sendCommand, "%s%04d",
            ps3Translate[21].value,
            Ps3.data.analog.button.square);
    cmdProcessor(sendCommand, 3);
  }
  if (Ps3.event.button_down.square && ps3Translate[22].value[0]) {
    if (debugSerial) { Serial.println("Started pressing the square button"); }
    cmdProcessor(ps3Translate[22].value, 3);
  }
  if (Ps3.event.button_up.square && ps3Translate[23].value[0]) {
    if (debugSerial) { Serial.println("Released the square button"); }
    cmdProcessor(ps3Translate[23].value, 3);
  }

  if (abs(Ps3.event.analog_changed.button.triangle) && ps3Translate[24].value[0]) {
    if (debugSerial) {
      Serial.print("Pressing the triangle button: ");
      Serial.println(Ps3.data.analog.button.triangle, DEC);
    }
    sprintf(sendCommand, "%s%04d",
            ps3Translate[24].value,
            Ps3.data.analog.button.triangle);
    cmdProcessor(sendCommand, 3);
  }
  if (Ps3.event.button_down.triangle && ps3Translate[25].value[0]) {
    if (debugSerial) { Serial.println("Started pressing the triangle button"); }
    cmdProcessor(ps3Translate[25].value, 3);
  }
  if (Ps3.event.button_up.triangle && ps3Translate[26].value[0]) {
    if (debugSerial) { Serial.println("Released the triangle button"); }
    cmdProcessor(ps3Translate[26].value, 3);
  }

  if (abs(Ps3.event.analog_changed.button.circle && ps3Translate[27].value[0])) {
    if (debugSerial) {
      Serial.print("Pressing the circle button: ");
      Serial.println(Ps3.data.analog.button.circle, DEC);
    }
    sprintf(sendCommand, "%s%04d",
            ps3Translate[27].value,
            Ps3.data.analog.button.circle);
    cmdProcessor(sendCommand, 3);
  }
  if (Ps3.event.button_down.circle && ps3Translate[28].value[0]) {
    if (debugSerial) { Serial.println("Started pressing the circle button"); }
    cmdProcessor(ps3Translate[28].value, 3);
  }
  if (Ps3.event.button_up.circle && ps3Translate[29].value[0]) {
    if (debugSerial) { Serial.println("Released the circle button"); }
    cmdProcessor(ps3Translate[29].value, 3);
  }

  //------------- Digital and Analog shoulder button events -------------

  if (abs(Ps3.event.analog_changed.button.l1) && ps3Translate[30].value[0]) {
    if (debugSerial) {
      Serial.print("Pressing the left shoulder button: ");
      Serial.println(Ps3.data.analog.button.l1, DEC);
    }
    sprintf(sendCommand, "%s%04d",
            ps3Translate[30].value,
            Ps3.data.analog.button.l1);
    cmdProcessor(sendCommand, 3);
  }
  if (Ps3.event.button_down.l1 && ps3Translate[31].value[0]) {
    if (debugSerial) { Serial.println("Started pressing the left shoulder button"); }
    cmdProcessor(ps3Translate[31].value, 3);
  }
  if (Ps3.event.button_up.l1 && ps3Translate[32].value[0]) {
    if (debugSerial) { Serial.println("Released the left shoulder button"); }
    cmdProcessor(ps3Translate[32].value, 3);
  }

  if (abs(Ps3.event.analog_changed.button.r1) && ps3Translate[33].value[0]) {
    if (debugSerial) {
      Serial.print("Pressing the right shoulder button: ");
      Serial.println(Ps3.data.analog.button.r1, DEC);
    }
    sprintf(sendCommand, "%s%04d",
            ps3Translate[3].value,
            Ps3.data.analog.button.r1);
    cmdProcessor(sendCommand, 3);
  }
  if (Ps3.event.button_down.r1 && ps3Translate[34].value[0]) {
    if (debugSerial) { Serial.println("Started pressing the right shoulder button"); }
    cmdProcessor(ps3Translate[34].value, 3);
  }
  if (Ps3.event.button_up.r1 && ps3Translate[35].value[0]) {
    if (debugSerial) { Serial.println("Released the right shoulder button"); }
    cmdProcessor(ps3Translate[35].value, 3);
  }

  //-------------- Digital and Analog trigger button events -------------

  if (abs(Ps3.event.analog_changed.button.l2) && ps3Translate[36].value[0]) {
    if (debugSerial) {
      Serial.print("Pressing the left trigger button: ");
      Serial.println(Ps3.data.analog.button.l2, DEC);
    }
    sprintf(sendCommand, "%s%04d",
            ps3Translate[36].value,
            Ps3.data.analog.button.l2);
    cmdProcessor(sendCommand, 3);
  }
  if (Ps3.event.button_down.l2 && ps3Translate[37].value[0]) {
    if (debugSerial) { Serial.println("Started pressing the left trigger button"); }
    cmdProcessor(ps3Translate[37].value, 3);
  }
  if (Ps3.event.button_up.l2 && ps3Translate[38].value[0]) {
    if (debugSerial) { Serial.println("Released the left trigger button"); }
    cmdProcessor(ps3Translate[38].value, 3);
  }

  if (abs(Ps3.event.analog_changed.button.r2) && ps3Translate[39].value[0]) {
    if (debugSerial) {
      Serial.print("Pressing the right trigger button: ");
      Serial.println(Ps3.data.analog.button.r2, DEC);
    }
    sprintf(sendCommand, "%s%04d",
            ps3Translate[39].value,
            Ps3.data.analog.button.r2);
    cmdProcessor(sendCommand, 3);
  }
  if (Ps3.event.button_down.r2 && ps3Translate[40].value[0]) {
    if (debugSerial) { Serial.println("Started pressing the right trigger button"); }
    cmdProcessor(ps3Translate[40].value, 3);
  }
  if (Ps3.event.button_up.r2 && ps3Translate[41].value[0]) {
    if (debugSerial) { Serial.println("Released the right trigger button"); }
    cmdProcessor(ps3Translate[41].value, 3);
  }

  //---------- Digital select/start/ps button events ---------
  if (Ps3.event.button_down.select && ps3Translate[42].value[0]) {
    if (debugSerial) { Serial.println("Started pressing the select button"); }
    cmdProcessor(ps3Translate[42].value, 3);
  }
  if (Ps3.event.button_up.select && ps3Translate[43].value[0]) {
    if (debugSerial) { Serial.println("Released the select button"); }
    cmdProcessor(ps3Translate[43].value, 3);
  }

  if (Ps3.event.button_down.start && ps3Translate[44].value[0]) {
    if (debugSerial) { Serial.println("Started pressing the start button"); }
    cmdProcessor(ps3Translate[44].value, 3);
  }
  if (Ps3.event.button_up.start && ps3Translate[45].value[0]) {
    if (debugSerial) { Serial.println("Released the start button"); }
    cmdProcessor(ps3Translate[45].value, 3);
  }

  if (Ps3.event.button_down.ps && ps3Translate[46].value[0]) {
    if (debugSerial) { Serial.println("Started pressing the Playstation button"); }
    cmdProcessor(ps3Translate[46].value, 3);
  }
  if (Ps3.event.button_up.ps && ps3Translate[47].value[0]) {
    if (debugSerial) { Serial.println("Released the Playstation button"); }
    cmdProcessor(ps3Translate[47].value, 3);
  }

  //---------------------- Battery events ---------------------
  // if( battery != Ps3.data.status.battery ){
  //     battery = Ps3.data.status.battery;
  //     Serial.print("The controller battery is ");
  //     if( battery == ps3_status_battery_charging )      Serial.println("charging");
  //     else if( battery == ps3_status_battery_full )     Serial.println("FULL");
  //     else if( battery == ps3_status_battery_high )     Serial.println("HIGH");
  //     else if( battery == ps3_status_battery_low)       Serial.println("LOW");
  //     else if( battery == ps3_status_battery_dying )    Serial.println("DYING");
  //     else if( battery == ps3_status_battery_shutdown ) Serial.println("SHUTDOWN");
  //     else Serial.println("UNDEFINED");
  // }
}
//PS3 connect routine
void onConnect() {
  if (debugSerial) { Serial.println("PS3 Controller Connected."); }
}

// *********************************************************************************************
// Process PS3 command
// *********************************************************************************************
int ps3Processor(char ps3Command[], int source) {
  if (debugSerial) { Serial.printf("PS3 command ignored  %s  %d \n", ps3Command, source); }
  return 0;
}




