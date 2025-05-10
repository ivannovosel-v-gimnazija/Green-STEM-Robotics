//simple robot arm project
//basic arm controller program
//will work but will give very twitchy behaviour
//mainly useful for debugging and initial calibration of the joystick

#include <Servo.h>

// servo motors as joints in the arm
// joint 1 is in the base
#define J1PIN 3
Servo joint_1;
// joint 2 is on the lid of the base
#define J2PIN 5
Servo joint_2;
// joint 3 is on the top of the lower link
#define J3PIN 6
Servo joint_3;
// gripper has one servo that opens and closes it
#define GJPIN 9
Servo gripper;

// sensors in the joystick controller
// potentiometer 1 is in the base
#define P1PIN A0
int pot_1 = analogRead(P1PIN);
// potentiometer 2 is on the L-link
#define P2PIN A1
int pot_2 = analogRead(P2PIN);
// potentiometer 3 is on the lower link
#define P3PIN A2
int pot_3 = analogRead(P3PIN);
// left button
#define BLPIN 10
int left_button = 0;
// right button
#define BRPIN 11
int right_button = 0;

// at 0° gripper is closed, at 90° its completely open
int gripper_angle = 45;

// input values from potentiometers
// for implementing running average
const int numReadings = 10;


void setup() {
  joint_1.attach(J1PIN);
  joint_2.attach(J2PIN);
  joint_3.attach(J3PIN);
  gripper.attach(GJPIN);
  pinMode(BLPIN, INPUT_PULLUP);
  pinMode(BRPIN, INPUT_PULLUP);
  // Serial is here for debugging
  Serial.begin(9600);
}

// for basic debugging, reads every sensor and prints to Serial
void printState(){
  pot_1 = analogRead(P1PIN);
  pot_2 = analogRead(P2PIN);
  pot_3 = analogRead(P3PIN);
  Serial.print("Pot1: ");
  Serial.print(pot_1);
  Serial.print(" Pot 2: ");
  Serial.print(pot_2);
  Serial.print(" Pot 3: ");
  Serial.println(pot_3);
  if (!digitalRead(BLPIN)){
    Serial.println("Left pressed!");
  }
  if (!digitalRead(BRPIN)){
    Serial.println("Right pressed!");
  }
}

void loop() {
  // printState(); // uncomment to print to serial, comment everything else except delay

  // read the potentiometers in the joystick
  pot_1 = analogRead(P1PIN);
  pot_2 = analogRead(P2PIN);
  pot_3 = analogRead(P3PIN)2;

  // move the servos in the arm
  // the values here were determined by moving the joystick into wanted end positions
  // measure again for your joystick!
  int j1_angle = map(pot_1,360,740,0,180);
  int j2_angle = map(pot_2,170,840,0,180);
  int j3_angle = map(pot_3,200,900,180,0);
  joint_1.write(j1_angle);
  joint_2.write(j2_angle);
  joint_3.write(j3_angle);
  gripper.write(gripper_angle);

  // left button opens the gripper
  if (!digitalRead(BLPIN) and gripper_angle < 90){
    gripper_angle = gripper_angle + 5;
  }
  // right button closes the gripper
  if (!digitalRead(BRPIN) and gripper_angle > 0){
    gripper_angle = gripper_angle - 5;
  }
  
  delay(50);
}