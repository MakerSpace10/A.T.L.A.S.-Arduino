#include <Wire.h>
#include <MPU6050.h>
#include <AccelStepper.h>
#include <PID_v1.h>

// Define motor control pins
#define stepRPin 2
#define dirRPin 3
#define stepLPin 4
#define dirLPin 5

// Create the stepper motor objects
AccelStepper stepperR(AccelStepper::DRIVER, stepRPin, dirRPin);
AccelStepper stepperL(AccelStepper::DRIVER, stepLPin, dirLPin);

// Create an MPU6050 object
MPU6050 mpu;

// PID variables
double Setpoint, Input, Output;
double Kp = 2.0, Ki = 5.0, Kd = 1.0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Moving Average Filter variables
int FILTER_SIZE = 10 // Number of readings to average
double pitchReadings[FILTER_SIZE]; // Array to store the last N readings
int currentIndex = 0; // Current index for storing readings
double pitchFiltered = 0; // The filtered pitch value

void setup() {
  // Start serial communication
  Serial.begin(9600);

  // Initialize the MPU6050 sensor
  Wire.begin();
  mpu.initialize();
  
  // Check if the MPU6050 is connected properly
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed");
  }

  // Set the maximum speed and acceleration for the motors
  stepperR.setMaxSpeed(1000); // Max speed (steps per second)
  stepperL.setMaxSpeed(1000);
  stepperR.setAcceleration(500); // Acceleration (steps per second^2)
  stepperL.setAcceleration(500);
  stepperR.setCurrentPosition(0); // Optional: Set starting position to 0
  stepperL.setCurrentPosition(0);

  // Initialize the PID controller
  Setpoint = 0; // Target angle (e.g., 0 degrees pitch)
  myPID.SetMode(AUTOMATIC);  // Set PID mode to automatic

  // Initialize the filter array to zero
  for (int i = 0; i < FILTER_SIZE; i++) {
    pitchReadings[i] = 0;
  }
}

void loop() {
  // Read the sensor data (accelerometer and gyroscope)
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Calculate the pitch angle (simplified example)
  double pitch = atan2(ay, az) * 180.0 / PI;

  // Apply the Moving Average Filter
  // Remove the oldest reading and add the new pitch reading
  pitchReadings[currentIndex] = pitch;
  
  // Move to the next index (wrap around if necessary)
  currentIndex = (currentIndex + 1) % FILTER_SIZE;

  // Calculate the average of the last N readings
  pitchFiltered = 0;
  for (int i = 0; i < FILTER_SIZE; i++) {
    pitchFiltered += pitchReadings[i];
  }
  pitchFiltered /= FILTER_SIZE;
	//Reduces sudden jumps in sensor readings caused by noise, leading to smoother control of the system.
	//This helps the PID controller perform more effectively since it's operating on a less noisy signal.
  //In dynamic systems, a filter helps the controller focus on significant changes in the input rather than reacting to minor fluctuations.
  
  // Set the input value for the PID control (filtered sensor data)
  Input = pitchFiltered;

  // Run the PID algorithm
  myPID.Compute();

  // Move the motors based on the PID output
  stepperR.moveTo(Output); // Move motor 1 to the new position
  stepperL.moveTo(-Output); // Move motor 2 to the opposite position (for example, to balance the system)

  // Step the motors
  stepperR.run();
  stepperL.run();

  // Print values for debugging
  Serial.print("Pitch: ");
  Serial.print(pitch);       // Current raw pitch angle
  Serial.print("\tFiltered Pitch: ");
  Serial.print(pitchFiltered); // Smoothed pitch angle
  Serial.print("\tPID Output: ");
  Serial.println(Output);    // PID output used to control the motors

  delay(10); // Small delay for stability
}
