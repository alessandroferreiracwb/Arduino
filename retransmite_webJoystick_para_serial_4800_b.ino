// FUNCIONA COM ESP32
#include <SPI.h>
#include <string.h>

#define PACKET_SIZE 16
#define TIMEOUT_READ 100

const int ledPin = 2; 

uint8_t bufferSerial[PACKET_SIZE];
uint8_t lastReceivedBuffer[PACKET_SIZE];

void setup(){
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  Serial.begin(4800); 
  Serial2.begin(4800);
  
  memset(lastReceivedBuffer, 0, PACKET_SIZE);
}

void loop(){
  static int bufferIndex = 0;
  static unsigned long lastReceiveTime = 0;
  
  while (Serial.available()) {
    if (bufferIndex >= PACKET_SIZE) {
      bufferIndex = 0;
    }
    
    uint8_t currentByte = Serial.read();
    
    // Altera para 0xF0
    if (bufferIndex == 0 && currentByte != 0xF0) {
      continue;
    }

    bufferSerial[bufferIndex] = currentByte;
    bufferIndex++;
    lastReceiveTime = millis();
    
    if (bufferIndex == PACKET_SIZE) {
      // Altera para 0xE7
      if (bufferSerial[13] == 0xE7) {
        if (memcmp(bufferSerial, lastReceivedBuffer, PACKET_SIZE) != 0) {
          digitalWrite(ledPin, HIGH);
          
          Serial2.write(bufferSerial, PACKET_SIZE);
          
          delay(20); 
          digitalWrite(ledPin, LOW);
          
          memcpy(lastReceivedBuffer, bufferSerial, PACKET_SIZE);
        }
      }
      
      bufferIndex = 0;
    }
  }

  if (bufferIndex > 0 && (millis() - lastReceiveTime > TIMEOUT_READ)) {
    bufferIndex = 0;
  }
}