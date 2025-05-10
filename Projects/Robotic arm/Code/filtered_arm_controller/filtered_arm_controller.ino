//simple robot arm project
//filtered arm controller
//uses a running average to filter out noise in the potentiometers
//behaves much more stable than basic controller program

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

// input values array from potentiometers
// for implementing a running average filter
const int numReadings = 10;
const int potpins[] = {P1PIN, P2PIN, P3PIN};
int index = 0;
int potreadings[3][numReadings];
int potsum[] = {0,0,0};

// reads the value of the potentiometer, pot_i is 1, 2 or 3
// returns the running average of the potentiometer value
int readPotAvg(int pot_i){
  int i = pot_i - 1;
  potsum[i] = potsum[i] - potreadings[i][index];
  potreadings[i][index] = analogRead(potpins[i]);
  potsum[i] = potsum[i] + potreadings[i][index];
  int average = potsum[i] / numReadings;
  return average;
}

// for basic debugging, reads every sensor and prints to Serial
void printState(){
  pot_1 = readPotAvg(1);
  pot_2 = readPotAvg(2);
  pot_3 = readPotAvg(3);
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

void setup() {
  joint_1.attach(J1PIN);
  joint_2.attach(J2PIN);
  joint_3.attach(J3PIN);
  gripper.attach(GJPIN);
  pinMode(BLPIN, INPUT_PULLUP);
  pinMode(BRPIN, INPUT_PULLUP);
  //initialise the potentiometer readings
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    for(int pot = 0; pot < 3; pot++){
      potreadings[pot][thisReading] = analogRead(potpins[pot]);
      potsum[pot] = potsum[pot] + potreadings[pot][thisReading];
    }
  }
  // Serial is here for debugging
  Serial.begin(9600);
}

void loop() {
  //printState(); // uncomment to print to serial

  // read the potentiometers in the joystick and calculate a running average
  pot_1 = readPotAvg(1);
  pot_2 = readPotAvg(2);
  pot_3 = readPotAvg(3);

  // move the servos in the arm
  int j1_angle = map(pot_1,280,690,0,180);
  int j2_angle = map(pot_2,158,860,0,180);
  int j3_angle = map(pot_3,145,850,180,0);
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

  // change the index for the values arrays
  index = (index + 1) % numReadings;

  delay(50);
}