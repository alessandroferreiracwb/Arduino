/*
  WebSerialLite Demo AP
  ------
  This example code works for both ESP8266 & ESP32 Microcontrollers
  WebSerial is accessible at 192.168.4.1/webserial URL.

  Author: HomeboyC
*/
#include <Arduino.h>
#if defined(ESP8266)
  #include <ESP8266WiFi.h>
  #include <ESPAsyncTCP.h>
#elif defined(ESP32)
  #include <WiFi.h>
  #include <AsyncTCP.h>
#endif
#include <ESPAsyncWebServer.h>
#include <WebSerialLite.h>
#include <SoftwareSerial.h>

SoftwareSerial modbusSerial(16, 17); // RX, TX
const byte slaveID = 0x02;

const int ledPin = 2; // Pino do LED (pode ser alterado)
bool ledState = LOW; // Estado inicial do LED

AsyncWebServer server(80);

//const char* ssid = ""; // Your WiFi AP SSID 
//const char* password = ""; // Your WiFi Password

const char* ssid = "AP_Ext"; // Your WiFi SSID
const char* password = "Nevil-RC"; // Your WiFi Password

/* Message callback of WebSerial */
void recvMsg(uint8_t *data, size_t len){
  WebSerial.println("Received Data...");
  String d = "";
  for(int i=0; i < len; i++){
    d += char(data[i]);
  }
  WebSerial.println(d);
}

void setup() {
    pinMode(ledPin, OUTPUT);    
    modbusSerial.begin(9600);
    delay(1000);
    Serial.begin(9600);
    WiFi.softAP(ssid, password);

    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);
    // WebSerial is accessible at "<IP Address>/webserial" in browser
    WebSerial.begin(&server);
    /* Attach Message Callback */
    WebSerial.onMessage(recvMsg);
    server.begin();
}

void loop() {
    delay(2000);

         byte request[] = {
    slaveID, 0x03,       // ID, Function
    0x00, 0x00,          // Start address high/low
    0x00, 0x02           // Quantity of registers
  };

  // Append CRC
  uint16_t crc = calculateCRC(request, 6);
  byte requestFrame[8];
  memcpy(requestFrame, request, 6);
  requestFrame[6] = crc & 0xFF;
  requestFrame[7] = (crc >> 8) & 0xFF;

  // Send request
  modbusSerial.write(requestFrame, 8);
  modbusSerial.flush();

  // Read response
  delay(500);
  if (modbusSerial.available()) {
    byte response[9];
    int i = 0;
    while (modbusSerial.available() && i < 9) {
      response[i++] = modbusSerial.read();
    }

    if (i == 9 && verifyCRC(response, 9)) {
      Serial.println("Resposta Valida do Escravo:");
      for (int j = 0; j < 9; j++) {
        Serial.print(response[j], HEX);        
        Serial.print(" ");
      }
     
      Serial.println();
       if(response[3] < 40){
          ledState = !ledState; // Toggle o estado do LED
          digitalWrite(ledPin, ledState); // Define o pino do LED com o novo estado
        }
    } else {
      Serial.println("Erro de CRC ou resposta incompleta.");
    }
    /***************************************************/
    WebSerial.printf("R ");
     for (int j = 0; j < 9; j++) {
        WebSerial.printf(" : %1x", response[j]);        
        //WebSerial.printf(" : ");
      }
      WebSerial.printf("\n");
    /***************************************************/
  }

  delay(100);
    
    //WebSerial.printf("Millis=%lu\n", millis());
    //WebSerial.printf("Free heap=[%u]\n", ESP.getFreeHeap());
}

uint16_t calculateCRC(byte *data, byte len) {
  uint16_t crc = 0xFFFF;
  for (byte i = 0; i < len; i++) {
    crc ^= data[i];
    for (byte j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

bool verifyCRC(byte *data, byte len) {
  uint16_t receivedCRC = (data[len - 2]) | (data[len - 1] << 8);
  return calculateCRC(data, len - 2) == receivedCRC;
}
