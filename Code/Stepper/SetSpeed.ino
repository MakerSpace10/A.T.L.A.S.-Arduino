#include <AccelStepper.h>

// Define motor control pins
#define stepPin 2
#define dirPin 3

// Create an instance of the AccelStepper class
// Using AccelStepper::DRIVER for stepper drivers that control step and direction pins
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

void setup() {
  // Start serial communication
  Serial.begin(115200);
  // Set the maximum speed and acceleration for the motor
  stepper.setMaxSpeed(1000); // Max speed (steps per second)
  stepper.setAcceleration(500); // Acceleration (steps per second^2)
}

void loop() {
  // Always keep the motors moving
  stepper.runSpeed();

  stepper.setSpeed(50);

  delay(1000);

  stepper.setSpeed(-50);

  // Wait for 1 second
  delay(1000);
}
