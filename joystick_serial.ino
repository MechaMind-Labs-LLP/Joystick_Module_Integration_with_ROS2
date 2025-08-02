// Arduino code: Send joystick X and Y only

void setup() {
  Serial.begin(115200);  // Match baudrate with Python node
}

void loop() {
  int xVal = analogRead(A0);
  int yVal = analogRead(A1);

  Serial.print(xVal);
  Serial.print(",");
  Serial.println(yVal);  // Newline ends the message

  delay(50);  // Reduce spamming
}
