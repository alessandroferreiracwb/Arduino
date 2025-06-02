/*********************************************************/

// Para mudar a velocidade da Serial envie:
// baud=xxxx Ex: baud=9600


/********************************************************/
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerialLite.h>

AsyncWebServer server(80);

const char* ssid = "ESP32";
const char* password = "admin1234";

#define RXD2 16
#define TXD2 17

String uartBuffer = "";
unsigned long currentBaud = 9600;  // Valor inicial da UART2

// 🔄 Atualiza a taxa de transmissão da Serial2
void updateBaudRate(unsigned long newBaud) {
  Serial2.end();  // Encerra a UART atual
  delay(100);     // Pequeno delay para segurança
  Serial2.begin(newBaud, SERIAL_8N1, RXD2, TXD2);
  currentBaud = newBaud;
  WebSerial.printf("🔁 Taxa da UART2 atualizada para %lu bps\n", newBaud);
}

// 🔤 Processa comandos recebidos pelo WebSerial
void recvMsg(uint8_t *data, size_t len) {
  String msg = "";
  for (size_t i = 0; i < len; i++) {
    msg += (char)data[i];
  }

  msg.trim();
  WebSerial.println("📩 Comando recebido: " + msg);

  if (msg.startsWith("baud=")) {
    String baudStr = msg.substring(5);
    unsigned long newBaud = baudStr.toInt();
    if (newBaud > 0) {
      updateBaudRate(newBaud);
    } else {
      WebSerial.println("❌ Valor inválido para baud rate.");
    }
  } else {
    WebSerial.println("❔ Comando desconhecido.");
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(currentBaud, SERIAL_8N1, RXD2, TXD2);

  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.println("WebSerial em: http://" + IP.toString() + "/webserial");
  Serial.println("Valor inicial da UART2 = 9600 bps");

  WebSerial.begin(&server);
  WebSerial.onMessage(recvMsg);
  server.begin();

  WebSerial.println("🟢 WebSerial iniciado. Envie `baud=xxxxx` para mudar velocidade da UART2.");
}

void loop() {
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\n') {
      WebSerial.println(uartBuffer);
      uartBuffer = "";
    } else {
      uartBuffer += c;
    }
  }
}
