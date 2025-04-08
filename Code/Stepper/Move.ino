#include <AccelStepper.h>

// Define motor control pins
#define stepPin 2
#define dirPin 3

// Create an instance of the AccelStepper class
// Using AccelStepper::DRIVER for stepper drivers that control step and direction pins
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

void setup() {
  // Set the maximum speed and acceleration for the motor
  stepper.setMaxSpeed(1000); // Max speed (steps per second)
  stepper.setAcceleration(500); // Acceleration (steps per second^2)
}

void loop() {
  // Move the motor 800 steps from current position
  stepper.move(800);

  // Add a delay to ensure the motor has finished moving
  delay(1000);

  // Change direction: move to -800 steps (opposite direction)
  stepper.moveTo(-800);

  // Wait for 1 second
  delay(1000);
}
