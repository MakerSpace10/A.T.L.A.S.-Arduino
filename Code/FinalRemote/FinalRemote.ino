#include <WiiChuck.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//This is the Global Variable, you can use it anywhere in the program.
Accessory nunchuck1;

RF24 radio(7, 8); // CE pin, CSN pin

const byte address[6] = "00001";

// Max size of this struct is 32 bytes - NRF24L01 buffer limit: Do not exceed limit. Current capasity is 16 bytes.
struct Data_Package {
  int joyX = 0;
  int joyY = 0;
  int buttonC = 0;
  int buttonZ = 0;
};

Data_Package data; // Create a variable with the above structure

void setup(){
  //Wiring
  //VCC is red
  //GND is black
  //DATA is blue
  //CLOCK is white

  //Set Baud rate for Serial Monitor
  Serial.begin(115200);
  nunchuck1.begin(); // Initialize the WiiChuck object
  nunchuck1.type = NUNCHUCK;
  
  radio.begin();
  radio.openWritingPipe(address);
  radio.stopListening();
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
  
  data.joyX = nunchuck1.getJoyX();
  data.joyY = nunchuck1.getJoyY();
  data.buttonC = nunchuck1.getButtonC();
  data.buttonZ = nunchuck1.getButtonZ();
  // Send the whole data from the structure to the receiver
  radio.write(&data, sizeof(Data_Package));
  
  delay(1000);
}
