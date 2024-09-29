#include "SPI.h" 
#include "RF24.h" 
#include "nRF24L01.h" 

#include <HardwareSerial.h>

#define CE_PIN 5
#define CSN_PIN 26 
#define INTERVAL_MS_SIGNAL_LOST 1000 
#define INTERVAL_MS_SIGNAL_RETRY 250 
RF24 radio(CE_PIN, CSN_PIN); 
const byte address[6] = "00001"; 
//NRF24L01 buffer limit is 32 bytes (max struct size) 
struct payload { 
	 uint8_t s[32];
  // uint8_t s;
}; 
payload payload; 
unsigned long lastSignalMillis = 0; 
void setup() 
{ 
	 Serial.begin(250000); 
	 radio.begin(); 
	 //Append ACK packet from the receiving radio back to the transmitting radio 
	 radio.setAutoAck(false); //(true|false) 
	 //Set the transmission datarate 
	 radio.setDataRate(RF24_2MBPS); //(RF24_250KBPS|RF24_1MBPS|RF24_2MBPS) 
	 //Greater level = more consumption = longer distance 
	 radio.setPALevel(RF24_PA_MAX); //(RF24_PA_MIN|RF24_PA_LOW|RF24_PA_HIGH|RF24_PA_MAX) 
	 //Default value is the maximum 32 bytes1 
	 radio.setPayloadSize(sizeof(payload)); 
	 //Act as receiver 
	 radio.openReadingPipe(0, address); 
	 radio.startListening(); 
} 
void loop() 
{ 
	 unsigned long currentMillis = millis(); 
	 if (radio.available() > 0) { 
	   radio.read(&payload, sizeof(payload)); 
	   Serial.write(payload.s, sizeof(payload.s));
     Serial.flush();
     //Serial.println(payload.s[0]);
	   lastSignalMillis = currentMillis; 
	 } 
	 if (currentMillis - lastSignalMillis > INTERVAL_MS_SIGNAL_LOST) { 
	   lostConnection(); 
	 } 
} 
void lostConnection() 
{ 
	delay(INTERVAL_MS_SIGNAL_RETRY); 
} 
