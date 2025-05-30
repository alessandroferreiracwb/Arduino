#include <SoftwareSerial.h>

SoftwareSerial modbusSerial(16, 17); // RX, TX
const byte slaveID = 0x02;
byte Reg1;

void setup() {
  Serial.begin(9600);
  modbusSerial.begin(9600);
}

void loop() {
  // Verifica se há pelo menos 8 bytes disponíveis
  if (modbusSerial.available() >= 8) {
    byte request[8];
    
    // Lê os 8 bytes
    for (int i = 0; i < 8; i++) {
      request[i] = modbusSerial.read();
    }

    // Imprime os bytes recebidos (requisição do mestre)
    Serial.print("Requisição recebida do mestre: ");
    for (int i = 0; i < 8; i++) {
      if (request[i] < 0x10) Serial.print('0'); // padding zero à esquerda
      Serial.print(request[i], HEX);
      Serial.print(' ');
    }
    Serial.println();
    Reg1 = random(100);
    // Verifica se é para este slave e função 0x03 com CRC válido
    if (request[0] == slaveID && request[1] == 0x03 && verifyCRC(request, 8)) {
      // Simula resposta com dois registradores: 0x01F4 (500), 0x02BC (700)
      byte response[] = {
        slaveID, 0x03, 0x04,
        0x01, 0xF4, // Reg 1
        Reg1, 0xBC  // Reg 2
      };

      uint16_t crc = calculateCRC(response, 7);
      byte fullResponse[9];
      memcpy(fullResponse, response, 7);
      fullResponse[7] = crc & 0xFF;
      fullResponse[8] = (crc >> 8) & 0xFF;

      modbusSerial.write(fullResponse, 9);
    } else {
      Serial.println("Requisição inválida ou erro de CRC.");
    }
  }
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
