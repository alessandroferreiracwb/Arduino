/* Este código envia um frame de 16 bytes via Serial para um receptor. */

#include <WiFi.h>              // ESP32 V. 3.0.0
#include <WebSocketsServer.h>  // by Markus Sattler 2.16.1
#include <ArduinoJson.h>       // by Benoit Blanchon 7.4.2

// Substitua com suas credenciais de Wi-Fi
char* ssid = "NRC-AP_Ext";
char* password = "Nevil-RC";

WebSocketsServer webSocket = WebSocketsServer(81);

// --- Nova Estrutura para o frame de 16 bytes ---
typedef struct cockpitNevil {
  uint8_t specialMessage;
  uint8_t addressCode;
  uint8_t leftJoystickY;
  uint8_t leftJoystickX;
  uint8_t rightJoystickY;
  uint8_t rightJoystickX;
  uint8_t pedal1;
  uint8_t pedal2;
  uint8_t buttonsByte;  // 8 botões
  uint8_t keypadByte1;  // 8 teclas
  uint8_t keypadByte2;  // 8 teclas
  uint8_t keypadByte3;  // 8 teclas
} cockpitNevil_t;

cockpitNevil_t myCockpitData;
// Buffer para o novo frame de 16 bytes
uint8_t transmitBuffer[16];

// --- Função para calcular CRC16 (CRC-16-MODBUS) ---
uint16_t calculateCRC16(const byte *data_p, size_t len) {
  uint16_t _crc = 0xFFFF;
  char bit2 = 0;

  for (uint16_t i = 0; i < len; i++) {
    _crc ^= data_p[i];
    for (bit2 = 0; bit2 < 8; bit2++) {
      if (_crc & 0x0001) {
        _crc >>= 1;
        _crc ^= 0xA001;
      } else {
        _crc >>= 1;
      }
    }
  }
  return _crc;
}

/**
 * @brief Empacota os dados da struct cockpitNevil_t em um array de bytes para transmissão.
 * @param _cockpit Ponteiro para a struct cockpitNevil_t com os dados a serem transmitidos.
 * @param data Array de bytes onde o frame de transmissão será montado (16 bytes).
 */
void buildCockpitNevilFrame(const cockpitNevil_t *_cockpit, uint8_t *data) {
  // Byte 0: Início do frame
  data[0] = 0xF0;

  // Bytes 1-2
  data[1] = _cockpit->specialMessage;
  data[2] = _cockpit->addressCode;

  // Bytes 3-9: Eixos e Pedais
  data[3] = _cockpit->leftJoystickY;
  data[4] = _cockpit->leftJoystickX;
  data[5] = _cockpit->rightJoystickY;
  data[6] = _cockpit->rightJoystickX;
  data[7] = _cockpit->pedal1;
  data[8] = _cockpit->pedal2;

  // Bytes 9-12: Botões e Teclas
  data[9] = _cockpit->buttonsByte;
  data[10] = _cockpit->keypadByte1;
  data[11] = _cockpit->keypadByte2;
  data[12] = _cockpit->keypadByte3;

  // Byte 13: Fim do frame
  data[13] = 0xE7;

  // Bytes 14 e 15: CRC16 Modbus
  uint16_t calculatedCrc = calculateCRC16(data, 14); // Calcula CRC dos bytes 0 a 13
  data[14] = (uint8_t)(calculatedCrc >> 8);   // Byte alto do CRC (MSB)
  data[15] = (uint8_t)(calculatedCrc & 0xFF); // Byte baixo do CRC (LSB)
}

// --- Função principal para manipular eventos WebSocket ---
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_TEXT) {
    //Serial.println("\n--- Enviando Frame de 16 bytes ---");
   /* for (int i = 0; i < 16; i++) {
        // Imprime o byte em formato hexadecimal com 2 dígitos
        if (transmitBuffer[i] < 0x10) {
            Serial.print("0");
        }
        Serial.print(transmitBuffer[i], HEX);
        Serial.print(" ");
    }
    Serial.println(""); */
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, payload);
    if (error) return;

    // Função auxiliar para mapear valor float (-100 a 100) para ponto (0-31) e flags de direção
    auto mapAxisToStruct = [&](float value, uint8_t &targetByte) {
      int absolute_value = abs(round(value));
      uint8_t point = map(absolute_value, 0, 100, 0, 31);
      point = constrain(point, 0, 31);
      
      bool positiveFlag = (value > 0);
      bool negativeFlag = (value < 0);
      
      // Combina flags e ponto em um único byte
      targetByte = (point & 0x1F) | ((positiveFlag ? 1 : 0) << 5) | ((negativeFlag ? 1 : 0) << 6);
    };

    // Mapeamento dos Eixos
    mapAxisToStruct(doc["eixo_esq"]["y"].as<float>(), myCockpitData.leftJoystickY);
    mapAxisToStruct(doc["eixo_esq"]["x"].as<float>(), myCockpitData.leftJoystickX);
    mapAxisToStruct(doc["eixo_dir"]["y"].as<float>(), myCockpitData.rightJoystickY);
    mapAxisToStruct(doc["eixo_dir"]["x"].as<float>(), myCockpitData.rightJoystickX);

    // Mapeamento dos Pedais
    mapAxisToStruct(doc["pedais"]["pedal1"].as<float>(), myCockpitData.pedal1);
    mapAxisToStruct(doc["pedais"]["pedal2"].as<float>(), myCockpitData.pedal2);

    // Mapeamento dos Botões (8 botões)
    myCockpitData.buttonsByte = 0;
    for (int i = 0; i < 8; i++) {
        String btnName = "btn" + String(i + 1);
        if (doc["botoes"].containsKey(btnName) && doc["botoes"][btnName].as<bool>()) {
            myCockpitData.buttonsByte |= (1 << i);
        }
    }
    
    // Mapeamento das Teclas (24 teclas em 3 bytes)
    myCockpitData.keypadByte1 = 0;
    for (int i = 0; i < 8; i++) {
        String keyName = "key" + String(i + 1);
        if (doc["teclas"].containsKey(keyName) && doc["teclas"][keyName].as<bool>()) {
            myCockpitData.keypadByte1 |= (1 << i);
        }
    }
    myCockpitData.keypadByte2 = 0;
    for (int i = 8; i < 16; i++) {
        String keyName = "key" + String(i + 1);
        if (doc["teclas"].containsKey(keyName) && doc["teclas"][keyName].as<bool>()) {
            myCockpitData.keypadByte2 |= (1 << (i - 8));
        }
    }
    myCockpitData.keypadByte3 = 0;
    for (int i = 16; i < 24; i++) {
        String keyName = "key" + String(i + 1);
        if (doc["teclas"].containsKey(keyName) && doc["teclas"][keyName].as<bool>()) {
            myCockpitData.keypadByte3 |= (1 << (i - 16));
        }
    }
    
    // Define valores padrão
    myCockpitData.specialMessage = 0x7F;
    myCockpitData.addressCode = 0x00;

    // Monta e envia o frame
    buildCockpitNevilFrame(&myCockpitData, transmitBuffer);
    Serial.write(transmitBuffer, 16);
  }
}

void setup() {
  Serial.begin(4800); 

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
  }
  Serial.println("\nWiFi Conectado! ✅");
  Serial.print("Endereço IP: ");
  Serial.println(WiFi.localIP());

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  memset(&myCockpitData, 0, sizeof(myCockpitData));
}

void loop() {
  webSocket.loop();
}