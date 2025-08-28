// funciona no esp32
#include <SPI.h>
#include <string.h>

#define PACKET_SIZE 16
#define TIMEOUT_READ 100

const int ledPin = 2; 

uint8_t bufferSerial[PACKET_SIZE];

void setup(){
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  Serial.begin(4800); 
  Serial2.begin(4800);
}

void loop(){
  static int bufferIndex = 0;
  static unsigned long lastReceiveTime = 0;
  
  while (Serial.available()) {
    if (bufferIndex >= PACKET_SIZE) {
      bufferIndex = 0;
    }
    
    uint8_t currentByte = Serial.read();
    
    if (bufferIndex == 0 && currentByte != 0xF0) {
      continue;
    }

    bufferSerial[bufferIndex] = currentByte;
    bufferIndex++;
    lastReceiveTime = millis();
    
    if (bufferIndex == PACKET_SIZE) {
      if (bufferSerial[13] == 0xE7) {
        
        // Acende o LED
        digitalWrite(ledPin, HIGH);
        
        // Envia o pacote de dados para o Arduino 2 através da Serial2
        Serial2.write(bufferSerial, PACKET_SIZE);
        
        delay(20); 
        digitalWrite(ledPin, LOW);
      }
      
      bufferIndex = 0;
    }
  }

  if (bufferIndex > 0 && (millis() - lastReceiveTime > TIMEOUT_READ)) {
    bufferIndex = 0;
  }
}
