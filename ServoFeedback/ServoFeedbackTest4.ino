/*
ServoFeedbackTest4

The purpose of this code is to demonstrate the use of the feedback signal to report on the
current position of the servo arm as it moves
 
A random position is chosen for the arm to move to each time, but to make this meaningful, a new
position is always more than 60 degrees away from the previous.

the serial speed is set to 115200
*/

// include the Servo library
#include <Servo.h>

int mapHigh = 606;  // <<<<<<<<<<<<<<<<<<<<< record and update these values for your servo
int mapLow = 50;

void (*resetFunc)(void) = 0;  // reset function

Servo fbServo;  // create a servo object

int const potPin = A0;  // analog pin used to connect the potentiometer
int angle;              // variable to hold the angle for the servo motor
int error = 2;          // error value

void setup() {

  Serial.begin(115200);  // open a serial connection
  delay(50);
  Serial.println("Starting...");

  int v = analogRead(A0);  // read feedback value

  angle = map(v, mapLow, mapHigh, 0, 180);  // map feedback value against arm sweep to provide a startup angle

  fbServo.write(angle);  // write startup angle to servo

  fbServo.attach(9);  // attach the servo on pin 9 to the servo object

  randomSeed(v);

  delay(500);
}

void loop() {
  delay(2000);   //pause to allow observation

  int newangle = 0;

  while (abs(newangle - angle) < 60) { // ensure new angle at least 60 degrees from old
    newangle = random(0, 180);  // set angle to random value 0-180
  }
  Serial.print("New angle:");
  Serial.println(newangle);
  fbServo.write(newangle);

  int v = analogRead(A0);                   // read feedback value
  angle = map(v, mapLow, mapHigh, 0, 180);  // map feedback value against arm sweep to provide current angle

  while (abs(angle - newangle) > error) {
    int v = analogRead(A0);  // read feedback value

    angle = map(v, mapLow, mapHigh, 0, 180);  // map feedback value against arm sweep to provide a current angle
    Serial.print("..");
    Serial.print(angle);
    delay(20);
  }
  Serial.println("");
  Serial.println("Move complete");
}
