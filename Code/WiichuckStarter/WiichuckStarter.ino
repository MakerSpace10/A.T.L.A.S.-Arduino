#include <WiiChuck.h>

//This is the Global Variable, you can use it anywhere in the program.
Accessory nunchuck1;

void setup(){
  //Wiring
  //VCC is red
  //GND is black
  //DATA is blue
  //CLOCK is white

  //Set Baud rate for Serial Monitor
  Serial.begin(115200);
  nunchuck1.begin(); // Initialize the Nunckuck object
  nunchuck1.type = NUNCHUCK; //Avoid library problems
}

void loop(){
  nunchuck1.readData();    // Read inputs and update maps
  Serial.print("X: ");
  Serial.println(nunchuck1.getJoyX());
  Serial.print("Y: ");
  Serial.println(nunchuck1.getJoyY());
  Serial.print("C btn: ");
  Serial.println(nunchuck1.getButtonC());
  Serial.print("Z btn: ");
  Serial.println(nunchuck1.getButtonZ());
  delay(1000);
}