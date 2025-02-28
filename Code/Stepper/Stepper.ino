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

  // Optionally, set the current position
  stepper.setCurrentPosition(0); // Optional: Set starting position to 0
}

void loop() {
  // Move the motor to position 800 steps in the current direction
  stepper.moveTo(800);  // Move to position 800 (relative position)

  // Run the motor until it reaches the target position
  while (stepper.distanceToGo() != 0) {
    stepper.run(); // Keep the motor moving towards the target
  }

  // Add a delay to ensure the motor has finished moving
  delay(1000); // Wait for 1 second

  // Change direction: move to -800 steps (opposite direction)
  stepper.moveTo(-800);

  // Run the motor until it reaches the new target position
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }

  // Wait for 1 second
  delay(1000);
}
