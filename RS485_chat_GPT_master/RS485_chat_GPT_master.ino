#include <SoftwareSerial.h>

SoftwareSerial modbusSerial(16, 17); // RX, TX
const byte slaveID = 0x02;

void setup() {
  Serial.begin(9600);
  modbusSerial.begin(9600);
  delay(1000);
}

void loop() {
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
      Serial.println("Resposta válida:");
      for (int j = 0; j < 9; j++) {
        Serial.print(response[j], HEX);
        Serial.print(" ");
      }
      Serial.println();
    } else {
      Serial.println("Erro de CRC ou resposta incompleta.");
    }
  }

  delay(2000);
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
