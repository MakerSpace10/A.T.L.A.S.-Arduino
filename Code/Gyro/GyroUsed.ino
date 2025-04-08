#include <Wire.h>
#include <MPU6050.h>

// Create an MPU6050 object
MPU6050 mpu;

// Moving Average Filter variables
const int FILTER_SIZE = 10; // Number of readings to average
double pitchReadings[FILTER_SIZE]; // Array to store the last N readings
int currentIndex = 0; // Current index for storing readings
double pitchFiltered = 0; // The filtered pitch value

void setup() {
  // Start serial communication
  Serial.begin(115200);

  // Initialize the MPU6050 sensor
  Wire.begin();
  mpu.initialize();
  
  // Check if the MPU6050 is connected properly
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed");
  }

  // Initialize the filter array to zero
  for (int i = 0; i < FILTER_SIZE; i++) {
    pitchReadings[i] = 0;
  }
}

void loop() {
  // Get raw accelerometer values
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  // Convert raw accelerometer values to "g" (since the MPU6050 outputs in terms of gravitational force)
  float Ax = (float)ax / 16384.0;  // Sensitivity scale factor for ±2g range
  float Ay = (float)ay / 16384.0;  // Sensitivity scale factor for ±2g range
  float Az = (float)az / 16384.0;  // Sensitivity scale factor for ±2g range

  // Calculate pitch using the accelerometer data
  float pitch = atan2(Ax, sqrt(Ay * Ay + Az * Az)) * 180.0 / PI;

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
  // Reduces sudden jumps in sensor readings caused by noise, leading to smoother control of the system.
  // This helps the PID controller perform more effectively since it's operating on a less noisy signal.
  // In dynamic systems, a filter helps the controller focus on significant changes in the input rather than reacting to minor fluctuations.
  
  
  // Print values for debugging
  Serial.print("Pitch: ");
  Serial.print(pitch);       // Current raw pitch angle
  Serial.print("\tFiltered Pitch: ");
  Serial.print(pitchFiltered); // Smoothed pitch angle

  delay(100); // Small delay for stability
}