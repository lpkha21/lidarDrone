#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <HardwareSerial.h>

#define QUEUE_SIZE 800  // Number of items in the queue
#define ITEM_SIZE 128     // Size of each item (32 bytes)

struct Payload{
  uint8_t s[32];
};

Payload payload;

// NRF module configuration
RF24 radio(5, 26); // CE, CSN pins
const byte address[6] = "00001"; 

QueueHandle_t lidarQueue; // FreeRTOS queue handle

// Task handles
TaskHandle_t lidarTaskHandle;
TaskHandle_t nrfTaskHandle;

HardwareSerial lidarSerial(1); // Assuming you're using Serial1 for the Lidar

void sendCommand(uint8_t* command, size_t length) {
  lidarSerial.write(command, length);
  lidarSerial.flush();
}


void setup() {
  Serial.begin(230400);
  lidarSerial.begin(230400, SERIAL_8N1, 16, 17); // Pins for Lidar TX/RX
  lidarSerial.setRxBufferSize(32768);

  // NRF24L01 setup
  radio.begin(); 
	 //Append ACK packet from the receiving radio back to the transmitting radio 
  radio.setAutoAck(false); //(true|false) 
	 //Set the transmission datarate 
	 radio.setDataRate(RF24_2MBPS); //(RF24_250KBPS|RF24_1MBPS|RF24_2MBPS) 
	 //Greater level = more consumption = longer distance 
	 radio.setPALevel(RF24_PA_MAX); //(RF24_PA_MIN|RF24_PA_LOW|RF24_PA_HIGH|RF24_PA_MAX) 
	 //Default value is the maximum 32 bytes 
	 radio.setPayloadSize(sizeof(payload)); 
	 //Act as transmitter 
	 radio.openWritingPipe(address); 


  // Create a queue to hold 32-byte data chunks
  lidarQueue = xQueueCreate(QUEUE_SIZE, ITEM_SIZE);

  if (lidarQueue == NULL) {
    while (1);
  }

  //Create tasks
  xTaskCreatePinnedToCore(
    lidarTask, "Lidar Read Task", 10000, NULL, 1, &lidarTaskHandle, 0);

  xTaskCreatePinnedToCore(
    nrfTask, "NRF Send Task", 10000, NULL, 2, &nrfTaskHandle, 1);

  uint8_t startCommand[] = {0xA5, 0x60};
  sendCommand(startCommand, sizeof(startCommand));

  uint8_t incFreq[] = {0xA5, 0x0B};
  for(int i = 0; i < 6; i++){
    sendCommand(incFreq, sizeof(incFreq));
  }

}

void lidarTask(void *parameter) {
  uint8_t lidarData[ITEM_SIZE];

  while (true) {
    if (lidarSerial.available()) {
      if (lidarSerial.available() >= ITEM_SIZE) {
        lidarSerial.readBytes(lidarData, ITEM_SIZE);

        // Add the data to the queue
        if (xQueueSend(lidarQueue, lidarData, portMAX_DELAY) != pdPASS) {
          lidarSerial.flush(); // Optional
        }
      }
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);  // Reduced delay to read more frequently
  }
}

void nrfTask(void *parameter) {
  uint8_t buffer[ITEM_SIZE];

  while (true) {
    if (xQueueReceive(lidarQueue, buffer, portMAX_DELAY) == pdPASS) {
      for (int i = 0; i < 4; i++) {
        memcpy(payload.s, buffer + i * 32, 32);
        radio.write(&payload, sizeof(payload));
      }
      vTaskDelay(1 / portTICK_PERIOD_MS);  // Reduced delay to send more quickly
    }
  }
}

void loop() {
  // nothings
}
