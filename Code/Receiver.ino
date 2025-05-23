#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN

const byte address[6] = "00001";

// Max size of this struct is 32 bytes - NRF24L01 buffer limit
struct Data_Package {
  int joyX = 0;
  int joyY = 0;
};

Data_Package data; //Create a variable with the above structure

void setup() {
  Serial.begin(115200);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.startListening();
}

void loop() {
  // Check whether there is data to be received
  if (radio.available()) {
    radio.read(&data, sizeof(Data_Package)); // Read the whole data and store it into the 'data' structure
  }
  Serial.println("Break");
  Serial.print("X: ");
  Serial.println(data.joyX);
  Serial.print("Y: ");
  Serial.println(data.joyY);
  
  delay(1000);
}