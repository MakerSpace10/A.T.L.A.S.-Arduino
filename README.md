# A.T.L.A.S.-Arduino

The acronym 'A.T.L.A.S.' stands for 'Aboriginal Traversal Land Analysis System'. I named it this because it is my first real robotics project and it can traverse the land around it.

The goal of this project is to create a two-wheeled, self-balancing robot, that can be remote controlled by a Wii Nunckuck. This project was originally intended for a [competition](https://robogames.net/rules/balancer.php) in the [Robo Games](https://robogames.net/index.php) event, in California. The two main focuses of the project are: the Wii Nunchuck "adapter", and the robot itself. Here is a deep dive into the build process/code. The code examples can be found in their entirety in the folder above.

## Remote

### Library
The Wii Nunchuck uses the I2C protocal to comunicate with the Wii Remote, the Arduino boards also use this to communicate with sensors. If you look on the inside of the Nunchuck plug, the slot there is conviniently the same thickness as a common circut board. This will allow for the use of a [Wii Nunckuck Adapter](https://www.amazon.com/FainWan-Compatible-WiiChuck-Nunchuck-Ar-duino/dp/B09LM69T4V/ref=sr_1_4?crid=3BH2JGEV86DFC&keywords=nunchuck+adapter&qid=1701803797&s=videogames&sprefix=nunchuck+adapter%2Cvideogames%2C61&sr=1-4). For wiring ONLY, follow this [guide](https://www.youtube.com/watch?v=vhJRR_7m6z4). As for programming, user [madhephaestus](https://github.com/madhephaestus) has done the hard part and created a library to communicate with the Wii Nunchuck. This library is called [Wiichuck](https://github.com/madhephaestus/WiiChuck) and has been vetted by Arduino themselves. Please note: the Wii Nunchuck has a joystick, two buttons, a gyroscope/accelerometer; I will only use the joystick. If you want to use the other values, please look through the library documentation.

After adding the library to your IDE, you can get started. Start by using a global variable:
```
Accessory nunchuck1;
```
This is an object that will represent the Nunckuck.

```
nunchuck1.begin(); // Initialize the WiiChuck object
nunchuck1.type = NUNCHUCK;
```
The first line initializes the object. Now it is the Nunckuck object. The library can be used for any Wii accessories. The second line is there to prevent any errors that the library may have regognizing the accessory. 

```
nunchuck1.readData();
Serial.print("X: ");
Serial.println(nunchuck1.getJoyX());
Serial.print("Y: ");
Serial.println(nunchuck1.getJoyY());
```
This takes in the updated values comming from the Wii Nunchuck and prints them to the Serial Monitor. All the functions are very intuitive and self explanitory. The rest of the code is in the "Code/WiichuckStarter" folder.

### Transceiver
A transceiver is a modual that can send and recive data. For this project I used the NRF24L01 base modual. This has a range of about 100 meters. In freedom units that's over 300 feet. When adding it to the project, follow [How to Mechatronics](https://howtomechatronics.com/tutorials/arduino/arduino-wireless-communication-nrf24l01-tutorial/#:~:text=nRF24L01%20Transceiver%20Module,-Let's%20take%20a&text=It%20uses%20the%202.4%20GHz,2.4%20%E2%80%93%202.5GHz%20ISM%20band) guide. They go into much further detail than I care to do. When wiring, include a 10-100 uF capasitor. This will prevent any power supply noise and any associated issues.

```
RF24 radio(7, 8); // CE, CSN
```
Setting the radio's pins to specific ones on the board.

```
const byte address[6] = "00001";
```
This is the address of the transceiver. The transceiver will only send to / recive from this address. The prefix "const" just means it can't change.

```
struct Data_Package {
  int joyX = 0;
  int joyY = 0;
};
```
This is the data package that is sent over the transceiver. Think of it like any other package: it has contence and will only be sent to one address. The largest it can be is 32 bytes, this example is 4 bytes, so we're safe.

After the code previosly mentioned, it becomes very different based on what task it's doing. See the [How to Mechatronics](https://howtomechatronics.com/tutorials/arduino/arduino-wireless-communication-nrf24l01-tutorial/#:~:text=nRF24L01%20Transceiver%20Module,-Let's%20take%20a&text=It%20uses%20the%202.4%20GHz,2.4%20%E2%80%93%202.5GHz%20ISM%20band) guide for the full explanation.

Both the transceiver and the Nunchuck operate with 3.3v. The Arduino Uno only has one 3.3v pin; plugging both into the same pin would draw too much current (mA). This could damage the board. I found it's better to use the 5v pin and add a voltage regulator before adding the pins for the transeciver and the Nunchuck. This brings the voltage to the needed 3.3v while also making more current (mA) avalible to the curcit. When using a breadboard, select three rows (one for GND, one for input VCC, and one for output VCC). The regulator will connect to all three. Follow it [guide](https://www.youtube.com/watch?v=zMA1PjUn87g) for a visual representation. Remember to add the 10 uF capasitor after the regulator and before the transeciver/Nunchuck. The rest of the code is in the "Code/FinalRemote" and "Code/Receiver" folders.

Don't worry if the cover doesn't sit nicely on the housing. The amount of wires need to make the project function will push the cover off.

When pluging the Nunchuck into the 'adapter', be sure to have the 'dip' of the plug facing up. If an off-brand Nunchuck is used, there may not be proper contanct. I suggest sticking to the ones sold by Nintendo directly. 

## Robot

### Gyroscope
If the robot is going to balace on two wheels, it needs to detect when it is tilting/falling. That is the job of the GY-521 modual. This also uses the I2C protocall to comunicate. We will use the Wire library to do so, because we don't need to do anything fancy. The MPU6050 library will allow for easy reading of the data. 

```
MPU6050 mpu;
```
This creates the object that the code can refer to anywhere.

```
int16_t ax, ay, az;
int16_t gx, gy, gz;
```
This creates a name for all the values provided by the GY-521.

Start the I2C communication with 
```
Wire.begin();
```
and initialize the mpu object with 
```
mpu.initialize();
```

It's always good to test for proper conection. The following detects (and tells you) if everything is properly conected:
```
if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed");
  }
```

Assign the values to their variable names
```
mpu.getAcceleration(&ax, &ay, &az);
mpu.getRotation(&gx, &gy, &gz);
```

Now print them to the Serial moitor for user readability:
```
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
```

This explains how to get the raw, non-human readable values of the gyroscope. However to avoid too much complex math, and not confuse ourselves in the process, we stick to the accelerometer values. The number used calculates the value for normal gravity:
```
float Ax = (float)ax / 16384.0;
float Ay = (float)ay / 16384.0;
float Az = (float)az / 16384.0;
```

Calculate the pitch angle:
```
float pitch = atan2(Ax, sqrt(Ay * Ay + Az * Az)) * 180.0 / PI;
```

For sanity's sake, I added a sensor fillter. This will take the average of a specified number of gyroscope values, and prevent the robot from having (its equvilent to) a seizure.
```
int FILTER_SIZE = 10 // Number of readings to average
double pitchReadings[FILTER_SIZE]; // Array to store the last N readings
int currentIndex = 0; // Current index for storing readings
double pitchFiltered = 0; // The filtered pitch value
```

and the filter array to be all 0's:
```
for (int i = 0; i < FILTER_SIZE; i++) {
  pitchReadings[i] = 0;
}
```

The rest of the filter is straight forward:
```
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
```

The rest of the code is in the "Code/Gyro" folder.

### Stepper Motor
In order for the robot to corecct its balance, and move in general: I'm using two NEMA 17 stepper motors. They will provied a good balace of acuracy and weight (so the robot does't fall over). For wiring, follow this [guide](https://www.youtube.com/watch?v=7spK_BkMJys). The video is good for side by side walkthroughs, the [website](https://howtomechatronics.com/tutorials/arduino/stepper-motors-and-arduino-the-ultimate-guide/) is better for self-pacing. Make sure: everything is wired correctly, the driver is set correctly, and that the battery is charged/not weak.

If you're using the wires provided by the seller to connect the motor to the driver, the wire to pin translation is: Black to 1A, Green to 1B, Red to 2A, and Blue to 2B.

I will be using the AccelStepper library to control both motors. Lets start with one:

Define the pins that control the stepper motor:
```
#define stepPin 2
#define dirPin 3
```

Create a stepper motor object (We will refer to this when calling the functions):
```
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);
```

Set both the max speed and the acceleration of the motor:
```
stepper.setMaxSpeed(1000); // Max speed (steps per second)
stepper.setAcceleration(50); // Acceleration (steps per second^2)
```

You could set the starting position (This is so you can move it by position, instead of times rotated): 
```
stepper.setCurrentPosition(0);
```

Rotate the wheel to position 800:
```
stepper.moveTo(800);
// Run the motor until it reaches the target position
while (stepper.distanceToGo() != 0) {
  stepper.run(); // Keep the motor moving towards the target
}
```

AccelStepper allows for changing the direction to be easy, just use a negative sign (-).
```
stepper.moveTo(-800);
```

The code listed above allows for movement by absolute position, not by steps. Instead the following function is used:
```
stepper.move(800);
```
This will rotate the stepper 4 times (800 steps / 200 steps per rotation = 4 rotations), no matter what position it is at. With this method, the idea of using a negative sign still works the same way. The only difference is now the code does not need to check if the stepper has rotated enough.

However, for this project, I need the speed of the wheels to change, no the steps. In stead I use 
```
stepper.setSpeed(800);
```
The idea of the negative sign changing directions still works here. At the beginning of the loop, add
```
stepperR.runSpeed();
```
as the steppers need to run continuously.

The rest of the code (and use of the two different functions) is in the "Code/Stepper" folder.

### Balancing
For the robot to balance on it's own: it needs a special controller called a P.I.D. Controller. P.I.D. stands for Proportional-Integral-Derivative. There is a lot of complicated math to it. To put it Layman's terms: it takes the angle we are leaning at, compares it to the angle we want to be at (0 degrees for straight up), to find out how drasticly we need to move to get there and not fall over. Use these guides: [Hackster.io](https://www.hackster.io/marketingmanagerofdattabanur/arduino-self-balancing-robot-e23f9c), [What is a P.I.D. Control](https://www.youtube.com/watch?v=wkfEZmsQqiA), and the PID Explaination PDF to understand the concepts better. Each robot will be built differently, so the P.I.D. variables will need to be tweaked. The general ideas will always be present.

The P.I.D. library in this example is the [PID_v1](https://github.com/br3ttb/Arduino-PID-Library/tree/master). I found it to be simple and easy to use. Please note that you need to manually download the library from the repository, and upload it to the Arduino IDE. 

The import statment needs to look like this:
```
#include <PID_v1.h>
```

Now set all variables the P.I.D. controller uses and changes:
```
double Setpoint, Input, Output;
double Kp = 2.0, Ki = 5.0, Kd = 1.0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
```

Set the P.I.D. controller values:
```
Setpoint = 0; // Target angle (e.g., 0 degrees pitch)
myPID.SetMode(AUTOMATIC);
```

With this we can set the input for the P.I.D. controller to the output of the sensor filter:
```
// Set the input value for the PID control (filtered sensor data)
Input = pitchFiltered;
```

However: there mayby some margin of error (with the electronics or the way the robot was built), the angle might be off slightly.
To fix this: once the robot is built, stand it to be as straight up as possible. Take the angle that appears after filtering and add/subtract as needed.
Just the number before the decimal point will do. Ex.: if the robot is standing straight up and the angle is -3:
```
pitchFiltered += 3;
```
This is needed before use anywhere else. The line before setting the value of "Input" is ideal.

With that all set, we can run the P.I.D. controller:
```
myPID.Compute();
```
It's as simple as that. This will automaticly set the value of 'Output', which will be used to change the stepper motor behavior.
When it comes to controlling the stepper motors with "Output", the value may not translate perfectly to motor steps, so we multiply:
```
int steps = (int)(Output * 10);
if(pitchFiltered > 0){
  stepper.moveTo(steps);
}else if(pitchFiltered < 0){
  stepper.moveTo(-steps);
}
stepper.run();
```
This way the stepper motors will move the appropiate amount of steps to remain upright. It will move the appropiate direction based of of the direction of the pitch.

As a neat feature (for sanity's sake), it's good to stop the robot from moving if it leans too much. After the pitch passes a certain threshold, it cannot prevent itself from falling over. If it does, the wheels will keep spining, continuing to move the robot. To avoid this, "Input" is set inside a if statment:
```
// A max of 10 degree tilt
if(abs(pitchFiltered) < 10){
  // Set the input value for the PID control (filtered sensor data)
  Input = -1 * abs(pitchFiltered);
  // Run the PID algorithm
  myPID.Compute();

  int steps = (int)(Output * 5);
  if(pitchFiltered > 0){
    stepperR.moveTo(steps);
  }else if(pitchFiltered < 0){
    stepperR.moveTo(-steps);
  }
   
  // Step the motors
  stepper.run();
}else{
  Serial.println("");
  Serial.println("Please reset robot position.");
}
```

For debugging (optional):
```
Serial.print("Pitch: ");
Serial.print(pitch);       // Current raw pitch angle
Serial.print("\tFiltered Pitch: ");
Serial.print(pitchFiltered); // Smoothed pitch angle
Serial.print("\tPID Output: ");
Serial.println(Output);    // PID output used to control the motors
```

Finnally, the reciver and the balancer code combine. The balancer code will take priority, so the robot doesn't fall over.
