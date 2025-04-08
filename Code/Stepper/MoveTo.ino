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

  // Only set the starting position for convinience with the moveTo function
  stepper.setCurrentPosition(0);
}

void loop() {
  stepper.moveTo(800);  // Move to position 800

  // Run the motor until it reaches the target position
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }

  // Add a delay to ensure the motor has finished moving
  delay(1000);

  // Change direction: move to position -800 (opposite direction)
  stepper.moveTo(-800);

  // Run the motor until it reaches the new target position
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }

  // Wait for 1 second
  delay(1000);
}
