#include <Wire.h>
#include <MPU6050.h>

// Create an MPU6050 object
MPU6050 mpu;

// Accelerometer and gyroscope values
int16_t ax, ay, az;
int16_t gx, gy, gz;

void setup() {
  // Start serial communication for debugging
  Serial.begin(115200);
  
  // Initialize I2C communication
  Wire.begin();
  
  // Initialize the MPU6050
  mpu.initialize();
  
  // Check if the MPU6050 is connected properly
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed");
  }
}

void loop() {
  // Read raw accelerometer and gyroscope values
  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);
  
  // Print accelerometer values (in raw units)
  Serial.print("Accelerometer X: ");
  Serial.print(ax);
  Serial.print(" Y: ");
  Serial.print(ay);
  Serial.print(" Z: ");
  Serial.println(az);

  // Print gyroscope values (in raw units)
  Serial.print("Gyroscope X: ");
  Serial.print(gx);
  Serial.print(" Y: ");
  Serial.print(gy);
  Serial.print(" Z: ");
  Serial.println(gz);

  // Add a small delay for readability
  delay(1000);
}
