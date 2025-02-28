# A.T.L.A.S.-Arduino-Project
The goal of this project is to create a two-wheeled, self-balancing robot, that can be remote controlled by a Wii Nunckuck. This project was originally intended for a [competition](https://robogames.net/rules/balancer.php) in the [Robo Games](https://robogames.net/index.php) event in California. The two main portions of the project are the Wii Nunchuck "adapter" and the robot itself. Here is a deep dive in the build process/code. The code examples can be found in their entirety in the folder above.

## Remote

### Library
The Wii Nunchuck uses the I2C protocal to comunicate with the Wii Remote, the Arduino boards also use this to communicate with sensors. If you look on the inside of the Nunchuck plug, the slot there is conviniently the same thickness as a common circut board. This will allow for the use of a [Wii Nunckuck Adapter](https://www.amazon.com/FainWan-Compatible-WiiChuck-Nunchuck-Ar-duino/dp/B09LM69T4V/ref=sr_1_4?crid=3BH2JGEV86DFC&keywords=nunchuck+adapter&qid=1701803797&s=videogames&sprefix=nunchuck+adapter%2Cvideogames%2C61&sr=1-4). For wiring ONLY, follow this [guide](https://www.youtube.com/watch?v=vhJRR_7m6z4). As for programming, [madhephaestus](https://github.com/madhephaestus) has done the hard part and created a library to communicate with the Wii Nunchuck. This library is called [Wiichuck](https://github.com/madhephaestus/WiiChuck) and has been vetted by Arduino. Please note: the Wii Nunchuck has a joystick, two buttons, a gyroscope, and an accelerometer; I will only use the joystick and buttons. If you want to use the other values, please look through the library documentation. 
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
A transceiver is a modual that can send and recive data. For this project I used the NRF24L01 base modual. This has a range of about 100 meters (that's over 300 feet). When adding it to the project, follow [How to Mechatronics](https://howtomechatronics.com/tutorials/arduino/arduino-wireless-communication-nrf24l01-tutorial/#:~:text=nRF24L01%20Transceiver%20Module,-Let's%20take%20a&text=It%20uses%20the%202.4%20GHz,2.4%20%E2%80%93%202.5GHz%20ISM%20band) guide. They go into much further detail than I care to do. When wiring, the pictoral guide that in cludes a 10-100 uF capasitor is what you should use. This will prevent any power supply noise and any associated issues.

```
RF24 radio(7, 8); // CE, CSN
```
Setting the radio's pins to specific ones on the board.

```
const byte address[6] = "00001";
```
This is the address of the transceiver. The transceiver will only send to / recive from this address. "const" just means it can't change.

```
struct Data_Package {
  int joyX = 0;
  int joyY = 0;
};
```
This is the data package that is sent over the transceiver. Think of it like an Amazon package: it has contence and will only be sent to one address. The largest it can be is 32 bytes, this example is 16 bytes so, we're safe.

After the code previosly mentioned, it becomes very different based on what task it's doing. See the [How to Mechatronics](https://howtomechatronics.com/tutorials/arduino/arduino-wireless-communication-nrf24l01-tutorial/#:~:text=nRF24L01%20Transceiver%20Module,-Let's%20take%20a&text=It%20uses%20the%202.4%20GHz,2.4%20%E2%80%93%202.5GHz%20ISM%20band) guide for the full explanation.

Both the transceiver and the Nunchuck operate with 3.3v. The Arduino Uno only has one 3.3v pin; plugging both into the same pin would draw too much current (mA). This could damage the board. I found it's better to use the 5v pin and add a voltage regulator before adding the pins for the transeciver and the Nunchuck. This brings the voltage to the needed 3.3v while also making more current (mA) avalible to the curcit. When using a breadboard, select three rows (one for GND, one for input VCC, and one for output VCC). The regulator will connect to all three. Follow it [guide](https://www.youtube.com/watch?v=zMA1PjUn87g) for a visual representation. Remember to add the 10 uF capasitor after the regulator and before the transeciver/Nunchuck. The rest of the code is in the "Code/FinalRemote" and "Code/Receiver" folders.

## Robot

### Gyroscope
If the robot is going to balace on two wheels, it needs to detect when it is tilting/falling too much. That is the job of the GY-521 modual. This also uses the I2C protocall to comunicate. We will use the Wire library to do so, because we don't need to do anything fancy. The MPU6050 library will allow for easy reading of the data. 

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
The rest of the code is in the "Code/Gyro" folder.

### Stepper Motor
In order for the robot to corecct its balance, and move in general: I'm using two NEMA 17 stepper motors. They will provied a good balace of acuracy and weight (so the robot does't fall over). They have 200 steps per revolution: leading to a 1.8 degree angle per step. For wiring, follow this [guide](https://www.youtube.com/watch?v=7spK_BkMJys). The video is good for side by side walkthroughs, the [website](https://howtomechatronics.com/tutorials/arduino/stepper-motors-and-arduino-the-ultimate-guide/) is better for self-pacing.

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
stepper.setAcceleration(500); // Acceleration (steps per second^2)
```

You could also set the starting position (This is so you can move it by position, instead of times rotated): 
```
stepper.setCurrentPosition(0);
```

Rotate the wheel 4 times:
```
stepper.moveTo(800);
  // Run the motor until it reaches the target position
  while (stepper.distanceToGo() != 0) {
    stepper.run(); // Keep the motor moving towards the target
  }
```

The library I picked allows for changing the direction to be easy, just use a negative sign (-).
```
 stepper.moveTo(-800);
```
Now you can repeat the moving code above. The rest of the code is in the "Code/Stepper" folder.

### Balancing
To find out how much the robot has to ajust itself, it needs to know the angle that it is tilting at. Now time to commit a sin: using math outside of school. I am using the tan function to find the angles. Use this [guide](https://www.hackster.io/marketingmanagerofdattabanur/arduino-self-balancing-robot-e23f9c) to understand and write your own code. Each robot will be built differently, so certain numbers will need to be tweaked. The general concepts will always be present. 

By taking the values gathered by the gyro, plug them into an equation to dicover the leaning angle. The stepper motors will adjust accordingly.

Finnally, combine the reciver with the balancer code so you can control the robot. The balancer code must take priority, so the robot doesn't fall over.
