/*
ServoFeedbackTest2

The purpose of this code is to demonstrate the use of the feedback signal to initialise the
servo position at startup and prevent jerking

The code demonstrates the startup and then randomly repositions the arm and reboots the arduino
to repeat the demonstration

the serial speed is set to 115200
*/

// include the Servo library
#include <Servo.h>

int mapHigh = 606; // <<<<<<<<<<<<<<<<<<<<< record and update these values for your servo
int mapLow = 50;

void (*resetFunc)(void) = 0;  // reset function

Servo fbServo;  // create a servo object

int const potPin = A0;  // analog pin used to connect the potentiometer
int angle;              // variable to hold the angle for the servo motor
int error = 2;

void setup() {

  Serial.begin(115200);  // open a serial connection
  delay(50);
  Serial.println("Starting....");
  fbServo.attach(9);  // attach the servo on pin 9 to the servo object

}

void loop() {

  if (angle == 0) {  // alternate between max and min angles
    angle = 180;
  } else {
    angle = 0;
  }
  fbServo.write(angle);  // reposition servo
  delay(300);
  int v = analogRead(A0);  // read feedback value

  int actual = map(v, mapLow, mapHigh, 0, 180);  // map feedback value against arm sweep to provide actual angle

  if ( abs(actual-angle) < error )  // if difference less than error then ok
  {
    fbServo.write(actual);  // write servo to actual to prevent minor shifts
    Serial.print("Servo moved to ");
    Serial.print(angle);
    Serial.println(" successfully");
  }
  else {
    fbServo.write(actual);  // reposition servo to current angle to stop motor
    Serial.print("ERROR: Servo stuck at ");  // report error
    Serial.println(actual);
  }
    for (int i = 0; i <= 10; i++) {  // wait to reposition again
    Serial.print(".");
    delay(500);
  }
   Serial.println("");
}
