//simple robot arm project
//potentiometer calibration program

void setup() {
  Serial.begin(9600);
}

void loop() {
  // change the pin to the one where your potentiometer is connected (A0-A7)
  int position = analogRead(A0);
  // read the values in the serial monitor, aim for a value near 512 for the middle position
  Serial.println(position);
  delay(200);
}