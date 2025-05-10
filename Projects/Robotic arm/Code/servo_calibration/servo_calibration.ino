//simple robot arm project
//servo motor calibration program

#include <Servo.h>

Servo motor;

void setup() {
  // change to the pin your motor is connected
  motor.attach(3);
  // set the angle to the starting angle you want to use
  motor.write(90);
}