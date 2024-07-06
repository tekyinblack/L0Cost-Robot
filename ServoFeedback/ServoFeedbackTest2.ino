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

void setup() {

  Serial.begin(115200);  // open a serial connection
  delay(50);

  int v = analogRead(A0);  // read feedback value

  angle = map(v, mapLow, mapHigh, 0, 180);  // map feedback value against arm sweep to provide a startup angle
  Serial.println("");
  Serial.print("Startup angle:");
  Serial.println(angle);

  fbServo.write(angle);  // write startup angle to servo

  fbServo.attach(9);  // attach the servo on pin 9 to the servo object

  randomSeed(v);
  for (int i = 0; i <= 10; i++) {
    Serial.print(".");
    delay(500);
  }

  Serial.println("");
  angle = random(0, 180);  // set angle to random value 0-180

  fbServo.write(angle);  // reposition servo

  Serial.print("Next startup angle will be:");
  Serial.println(angle);
  Serial.println("Reboot in 2 seconds");

  for (int i = 0; i <= 10; i++) {
    Serial.print(".");
    delay(200);
  }
}

void loop() {
  resetFunc();
}
