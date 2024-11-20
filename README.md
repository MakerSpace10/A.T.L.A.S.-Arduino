# A.T.L.A.S.-Arduino-Project
The goal of this project is to create a two-wheeled, self-balancing robot, that can be remote controlled by a Wii Nunckuck. This project was originally intended for a competition in the Robo Games event in California. The two main portions of the project are the Wii Nunchuck "adapter" and the robot itself. Here is a deep dive in the build process and code. The code examples can be found in the folder.

## Remote

### Wiichuck
The Wii Nunchuck uses the I2C protocal to comunicate with the Wii Remote, the most Arduino boards also use this to communicate with sensors. If you look on the inside of the Nunchuck plug, the slot there is conviniently the same thickness as a common circut board. This will allow for the use of a [Wii Nunckuck Adapter](https://www.amazon.com/FainWan-Compatible-WiiChuck-Nunchuck-Ar-duino/dp/B09LM69T4V/ref=sr_1_4?crid=3BH2JGEV86DFC&keywords=nunchuck+adapter&qid=1701803797&s=videogames&sprefix=nunchuck+adapter%2Cvideogames%2C61&sr=1-4). For wiring ONLY, follow this [guide](https://www.youtube.com/watch?v=vhJRR_7m6z4). As for programming, [madhephaestus](https://github.com/madhephaestus) has done the hard part and created a library to communicate with the Wii Nunchuck. This library is called [Wiichuck](https://github.com/madhephaestus/WiiChuck) and has been vetted by Arduino. Please note: the Wii Nunchuck has a joystick, two buttons, a gyroscope, and an accelerometer; I will only use the joystick and buttons. If you want to use the other values, please look through the library documentation. 
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
Serial.print("C btn: ");
Serial.println(nunchuck1.getButtonC());
Serial.print("Z btn: ");
Serial.println(nunchuck1.getButtonZ());
```
This takes in the updated values comming from the Wii Nunchuck and prints them to the Serial Monitor. All the functions are very intuitive and self explanitory.

### Transceiver
A transceiver is a modual that can send and recive data. For this project I used the NRF24L01 base modual. This has a range of about 100 meters (that's over 300 feet). When adding it to the project, follow [How to Mechatronics](https://howtomechatronics.com/tutorials/arduino/arduino-wireless-communication-nrf24l01-tutorial/#:~:text=nRF24L01%20Transceiver%20Module,-Let's%20take%20a&text=It%20uses%20the%202.4%20GHz,2.4%20%E2%80%93%202.5GHz%20ISM%20band) guide. They go into much further detail than I care to do. When wiring, the pictoral guide that in cludes a 10-100 uF capasitor is what you should use. This will prevent any power supply noise and any associated issues.

Both the transceiver and the Nunchuck operate with 3.3v. The Arduino Uno only has one 3.3v pin; plugging both into the same pin would draw too much current (mA). This could damage the board. Instead, I used the 5v pin and added a voltage regulator befor adding the pins for the transeciver and the Nunchuck. This brings the voltage to the needed 3.3v while also making more current (mA) avalible to the curcit. When using a breadboard, select three rows (one for GND, one for input VCC, and one for output VCC). The regulator will connect to all three. Follow it [guide](https://www.youtube.com/watch?v=zMA1PjUn87g) for a visual representation. Remember to add the 10 uF capasitor after the regulator and before the transeciver/Nunchuck.

The code for just the reciver is also provided. See the [How to Mechatronics](https://howtomechatronics.com/tutorials/arduino/arduino-wireless-communication-nrf24l01-tutorial/#:~:text=nRF24L01%20Transceiver%20Module,-Let's%20take%20a&text=It%20uses%20the%202.4%20GHz,2.4%20%E2%80%93%202.5GHz%20ISM%20band) guide for the full explanation.

## Robot

### Gyroscope
If the robot is going to balace on two wheels, it needs to detect when it is tilting/falling too much. That is the job of the GY-501 modual. This also uses the I2C protocall to comunicate. We will use the Wire library to do so, because we don't need to do anything fancy. The MPU6050 library will allow for easy reading of the data. 

```
MPU6050 mpu;
```
This creates the object that the code can refer to anywhere.

```
int16_t ax, ay, az;
int16_t gx, gy, gz;
```
This creates a name for all the values provided by the GY-501.

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
