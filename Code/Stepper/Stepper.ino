// Pin definitions
const int dirPin = 4;  // Direction pin
const int stepPin = 3; // Step pin

void setup() {
  // Set motor control pins as outputs
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  
  // Initialize the direction (optional)
  digitalWrite(dirPin, HIGH); // Set direction to clockwise (can be LOW for counterclockwise)
}

void loop() {
  // Rotate motor one revolution clockwise
  for (int i = 0; i < 200 i++) {
    digitalWrite(stepPin, HIGH);  // Send a pulse
    delayMicroseconds(1000);      // Adjust speed (in microseconds)
    digitalWrite(stepPin, LOW);   // Complete the pulse
    delayMicroseconds(1000);      // Adjust speed (in microseconds)
  }

  delay(1000);  // Wait for 1 second

  // Reverse direction (counterclockwise)
  digitalWrite(dirPin, LOW);  // Change direction to counterclockwise

  // Rotate motor one revolution counterclockwise
  for (int i = 0; i < 200; i++) {
    digitalWrite(stepPin, HIGH);  // Send a pulse
    delayMicroseconds(1000);      // Adjust speed (in microseconds)
    digitalWrite(stepPin, LOW);   // Complete the pulse
    delayMicroseconds(1000);      // Adjust speed (in microseconds)
  }

  delay(1000);  // Wait for 1 second
}
