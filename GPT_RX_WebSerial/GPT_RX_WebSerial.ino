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

AsyncWebServer server(80);

const char* ssid = "ESP32";
const char* password = "admin1234";

#define RXD2 16
#define TXD2 17

String uartBuffer = "";  // Buffer de dados recebidos

void recvMsg(uint8_t *data, size_t len) {
  String msg = "";
  for (size_t i = 0; i < len; i++) {
    msg += (char)data[i];
  }
  WebSerial.println("Recebido do navegador:");
  WebSerial.println(msg);
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.println("WebSerial em: http://" + IP.toString() + "/webserial");
  Serial.println("Valor inicial da UART2 = 9600 bps");

  WebSerial.begin(&server);
  WebSerial.onMessage(recvMsg);
  server.begin();

  WebSerial.println("🟢 WebSerial iniciado. Aguardando UART2...");
}

void loop() {
  // Lê UART2 e acumula no buffer até nova linha
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\n') {
      WebSerial.println(uartBuffer);  // Envia a linha inteira
      uartBuffer = "";                // Limpa buffer
    } else {
      uartBuffer += c;
    }
  }
}
