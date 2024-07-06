/*
ServoFeedbackTest1

The purpose of this code is to demonstrate the signal received back from an SG90 servo
feedback pin after a take off lead has been fitted.

It also acts as a test both before and after a servo has been modified in this way

The servo is expected to be connected to pin 9 of an arduino, used on the L0Cost Robot controller and
the feed back connected to pin A0 to read the voltage

The program rotates the servo arm repeated through 180 degrees in 5 degree intervals

The output is intended to be displayed on an Arduino serial plot console

the serial speed is set to 115200
*/

// include the Servo library
#include <Servo.h>


int mapHigh = 606;  // <<<<<<<<<<<<<<<<<<<<< record and update these values for your servo
int mapLow = 50;

Servo fbServo;  // create a servo object

int angle;              // variable to hold the angle for the servo motor

void setup() {
  fbServo.attach(9);     // attach the servo on pin 9 to the servo object
  Serial.begin(115200);  // open a serial connection
  delay(500);

  angle = 0;  // set angle to 0

  fbServo.write(angle);  // update signal to servo

  delay(500);  // wait until servo has moved to 0. Takes .3 second so plenty of margin
}

void loop() {



  // Incremental loop

  for (angle = 0; angle <= 180; angle = angle + 5) {  // setup for loop to increment angle

    fbServo.write(angle);  // update signal to servo

    delay(50);  // wait for servo to settle

    int v = analogRead(A0);  // read feedback value

    int p = map(v, mapLow, mapHigh, 0, 180);  // feedback value against arm sweep to provide a prediction angle

    Serial.print("Angle:");  // outpput data format to plot command angle against feedback voltage and predicted angle
    Serial.print(angle);
    Serial.print(",Predicted:");
    Serial.print(p);
    Serial.print(",Voltage:");
    Serial.println(v);


    delay(50);
  }

  // Decremental loop

  for (angle = 180; angle >= 0; angle = angle - 5) {  // setup for loop to deccrement angle

    fbServo.write(angle);  // update signal to servo

    delay(50);  // wait for servo to settle

    int v = analogRead(A0);  // read feedback value

    int p = map(v, mapLow, mapHigh, 0, 180);  // feedback value against arm sweep to provide a prediction angle

    Serial.print("Angle:");  // outpput data format to plot command angle against feedback voltage and predicted angle
    Serial.print(angle);
    Serial.print(",Predicted:");
    Serial.print(p);
    Serial.print(",Voltage:");
    Serial.println(v);


    delay(50);
  }
}
