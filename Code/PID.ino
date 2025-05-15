#include <Wire.h>
#include <MPU6050.h>
#include <PID_v1.h>

// Create an MPU6050 object
MPU6050 mpu;

// PID variables
double Setpoint, Input, Output;
//     kp ~~ 0.5 - 1.0
double Kp = 0.5, Ki = 0.0, Kd = 0.0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

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

  // Initialize the PID controller
  Setpoint = 0; // Target angle (e.g., 0 degrees pitch)
  myPID.SetMode(AUTOMATIC);  // Set PID mode to automatic

  // Initialize the filter array to zero
  for (int i = 0; i < FILTER_SIZE; i++) {
    pitchReadings[i] = 0;
  }
}

unsigned long lastPIDTime = 0;
const unsigned long pidInterval = 10; // PID update interval in ms

void loop(){

  // PID loop at set interval
  unsigned long now = millis();
  if(now - lastPIDTime >= pidInterval){
    lastPIDTime = now;

    // Get raw accelerometer values
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);

    float Ax = (float)ax / 16384.0;
    float Ay = (float)ay / 16384.0;
    float Az = (float)az / 16384.0;

    float pitch = atan2(Ax, sqrt(Ay * Ay + Az * Az)) * 180.0 / PI;

    // Apply Moving Average Filter
    pitchReadings[currentIndex] = pitch;
    currentIndex = (currentIndex + 1) % FILTER_SIZE;
    pitchFiltered = 0;
    for(int i = 0; i < FILTER_SIZE; i++){
      pitchFiltered += pitchReadings[i];
    }
    pitchFiltered /= FILTER_SIZE;
    pitchFiltered += 3;

    Input = -1 * abs(pitchFiltered);
    myPID.Compute();

    Serial.print("Pitch: ");
    Serial.print(pitchFiltered);
    Serial.print("\tOutput: ");
    Serial.print(Output);
    Serial.print("\tError: ");
    Serial.print(Setpoint - pitchFiltered);
  }
}