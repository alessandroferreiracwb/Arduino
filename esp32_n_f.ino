// recebe dados via websockt e envia pela serial 
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h> // Certifique-se de ter a biblioteca ArduinoJson instalada

// Substitua com suas credenciais de Wi-Fi
char* ssid = "NRC-AP_Ext";         // Nome da sua rede Wi-Fi
char* password = "Nevil-RC";    // Senha da sua rede Wi-Fi

// Cria um objeto WebSocketsServer na porta 81 (porta padrão para WebSockets)
WebSocketsServer webSocket = WebSocketsServer(81);

// --- Definição da mesma estrutura do receptor ---
typedef struct cockpitNevil {
  /*byte 0: init of frame, default value is 0xF1 (handled in buildCockpitNevilFrame)*/

  /*byte 1*/
  uint8_t specialMessage;
  /*byte 2*/
  uint8_t addressCode;

  /*byte 3*/ /*JOYS ESQUERDO (Y)*/
  bool leftJoystickFowardFlag;
  bool leftJoystickBackwardFlag;
  uint8_t leftJoystickYPoint; // 5 bits (0-31)

  /*byte 4*/ /*JOYS ESQUERDO (X)*/
  bool leftJoystickRightFlag;
  bool leftJoystickLeftFlag;
  uint8_t leftJoystickXPoint; // 5 bits (0-31)

  /*byte 5*/ /*JOYS ESQUERDO ANALOGICO (1)*/
  bool leftAnalogic1UpFlag;
  bool leftAnalogic1DwFlag;
  uint8_t leftAnalogic1Point; // 5 bits (0-31)

  /*byte 6*/ /*JOYS ESQUERDO ANALOGICO (2)*/
  bool leftAnalogic2UpFlag;
  bool leftAnalogic2DwFlag;
  uint8_t leftAnalogic2Point; // 5 bits (0-31)

  /*byte 7*/ /*JOYS ESQUERDO ANALOGICO (3)*/
  bool leftAnalogic3UpFlag;
  bool leftAnalogic3DwFlag;
  uint8_t leftAnalogic3Point; // 5 bits (0-31)
/*------------------------------------------*/
  /*byte 8*/ /*JOYS DIREITO (Y)*/
  bool rightJoystickFowardFlag;
  bool rightJoystickBackwardFlag;
  uint8_t rightJoystickYPoint; // 5 bits (0-31)

  /*byte 9*/ /*JOYS DIREITO (X)*/
  bool rightJoystickRightFlag;
  bool rightJoystickLeftFlag;
  uint8_t rightJoystickXPoint; // 5 bits (0-31)

  /*byte 10*/ /*JOYS DIREITO ANALOGICO (1)*/
  bool rightAnalogic1UpFlag;
  bool rightAnalogic1DwFlag;
  uint8_t rightAnalogic1Point; // 5 bits (0-31)

  /*byte 11*/ /*JOYS DIREITO ANALOGICO (2)*/
  bool rightAnalogic2UpFlag;
  bool rightAnalogic2DwFlag;
  uint8_t rightAnalogic2Point; // 5 bits (0-31)

  /*byte 12*/ /*JOYS DIREITO ANALOGICO (3)*/
  bool rightAnalogic3UpFlag;
  bool rightAnalogic3DwFlag;
  uint8_t rightAnalogic3Point; // 5 bits (0-31)

  /*byte 13*/ /*PEDAL ESQUERDO*/
  bool leftPedalFowardFlag;
  bool leftPedalBackwardFlag;
  uint8_t leftPedalYPoint; // 5 bits (0-31)

  /*byte 14*/
  bool rightPedalFowardFlag;
  bool rightPedalBackwardFlag;
  uint8_t rightPedalYPoint; // 5 bits (0-31)

  /*byte 15*/
  bool AddPedalFowardFlag;
  bool AddPedalBackwardFlag;
  uint8_t AddPedalYPoint; // 5 bits (0-31)

  /*byte 16 - 17*/
  uint8_t rightJoystickButtons[6]; // 6 bits
  uint8_t leftJoystickButtons[6];    // 6 bits

  /*byte 18 - 20*/
  uint8_t keypadButtonStatus[24]; // Assuming 24 bits as per the parsing (0-23)

  /*byte 21: end of frame, default value is 0xE7 (handled in buildCockpitNevilFrame)*/

  /*byte 22 and byte 23: CRC16 Modbus (handled in buildCockpitNevilFrame)*/
} cockpitNevil_t;

// Variável global para armazenar os dados do cockpit
cockpitNevil_t myCockpitData;
// Buffer para armazenar o frame de transmissão
uint8_t transmitBuffer[24];

// --- Função para calcular CRC16 (CRC-16-MODBUS) ---
uint16_t calculateCRC16(const byte *data_p, size_t len) {
  uint16_t _crc = 0xFFFF; // Valor inicial para CRC-16-MODBUS
  char bit2 = 0;

  for (uint16_t i = 0; i < len; i++) {
    _crc ^= data_p[i]; // XOR com o byte de dados

    for (bit2 = 0; bit2 < 8; bit2++) {
      if (_crc & 0x0001) { // Verifica o bit menos significativo
        _crc >>= 1;       // Desloca um bit para a direita
        _crc ^= 0xA001;   // XOR com o polinômio (invertido) para CRC-16-MODBUS
      } else {
        _crc >>= 1;       // Desloca um bit para a direita
      }
    }
  }
  return _crc;
}

/**
 * @brief Empacota os dados da struct cockpitNevil_t em um array de bytes para transmissão.
 * @param _cockpit Ponteiro para a struct cockpitNevil_t com os dados a serem transmitidos.
 * @param data Array de bytes onde o frame de transmissão será montado (deve ter pelo menos 24 bytes).
 */
void buildCockpitNevilFrame(const cockpitNevil_t *_cockpit, uint8_t *data) {
  // Byte 0: Início do frame
  data[0] = 0xF1;

  // Byte 1: specialMessage
  data[1] = _cockpit->specialMessage;

  // Byte 2: addressCode
  data[2] = _cockpit->addressCode;

  // Bytes 3-7: Joysticks Esquerdos
  // As flags são os 2 bits menos significativos, o ponto são os 5 bits seguintes.
  data[3] = (_cockpit->leftJoystickFowardFlag ? (1 << 0) : 0) |
            ((_cockpit->leftJoystickBackwardFlag ? (1 << 1) : 0)) |
            ((_cockpit->leftJoystickYPoint & 0x1F) << 2);

  data[4] = (_cockpit->leftJoystickRightFlag ? (1 << 0) : 0) |
            ((_cockpit->leftJoystickLeftFlag ? (1 << 1) : 0)) |
            ((_cockpit->leftJoystickXPoint & 0x1F) << 2);

  data[5] = (_cockpit->leftAnalogic1UpFlag ? (1 << 0) : 0) |
            ((_cockpit->leftAnalogic1DwFlag ? (1 << 1) : 0)) |
            ((_cockpit->leftAnalogic1Point & 0x1F) << 2);

  data[6] = (_cockpit->leftAnalogic2UpFlag ? (1 << 0) : 0) |
            ((_cockpit->leftAnalogic2DwFlag ? (1 << 1) : 0)) |
            ((_cockpit->leftAnalogic2Point & 0x1F) << 2);

  data[7] = (_cockpit->leftAnalogic3UpFlag ? (1 << 0) : 0) |
            ((_cockpit->leftAnalogic3DwFlag ? (1 << 1) : 0)) |
            ((_cockpit->leftAnalogic3Point & 0x1F) << 2);

  // Bytes 8-12: Joysticks Direitos
  data[8] = (_cockpit->rightJoystickFowardFlag ? (1 << 0) : 0) |
            ((_cockpit->rightJoystickBackwardFlag ? (1 << 1) : 0)) |
            ((_cockpit->rightJoystickYPoint & 0x1F) << 2);

  data[9] = (_cockpit->rightJoystickRightFlag ? (1 << 0) : 0) |
            ((_cockpit->rightJoystickLeftFlag ? (1 << 1) : 0)) |
            ((_cockpit->rightJoystickXPoint & 0x1F) << 2);

  data[10] = (_cockpit->rightAnalogic1UpFlag ? (1 << 0) : 0) |
             ((_cockpit->rightAnalogic1DwFlag ? (1 << 1) : 0)) |
             ((_cockpit->rightAnalogic1Point & 0x1F) << 2);

  data[11] = (_cockpit->rightAnalogic2UpFlag ? (1 << 0) : 0) |
             ((_cockpit->rightAnalogic2DwFlag ? (1 << 1) : 0)) |
             ((_cockpit->rightAnalogic2Point & 0x1F) << 2);

  data[12] = (_cockpit->rightAnalogic3UpFlag ? (1 << 0) : 0) |
             ((_cockpit->rightAnalogic3DwFlag ? (1 << 1) : 0)) |
             ((_cockpit->rightAnalogic3Point & 0x1F) << 2);

  // Bytes 13-15: Pedais
  data[13] = (_cockpit->leftPedalFowardFlag ? (1 << 0) : 0) |
             ((_cockpit->leftPedalBackwardFlag ? (1 << 1) : 0)) |
             ((_cockpit->leftPedalYPoint & 0x1F) << 2);

  data[14] = (_cockpit->rightPedalFowardFlag ? (1 << 0) : 0) |
             ((_cockpit->rightPedalBackwardFlag ? (1 << 1) : 0)) |
             ((_cockpit->rightPedalYPoint & 0x1F) << 2);

  data[15] = (_cockpit->AddPedalFowardFlag ? (1 << 0) : 0) |
             ((_cockpit->AddPedalBackwardFlag ? (1 << 1) : 0)) |
             ((_cockpit->AddPedalYPoint & 0x1F) << 2);

  // Byte 16: Botões do Joystick Direito (6 bits)
  data[16] = (_cockpit->rightJoystickButtons[0] ? (1 << 0) : 0) |
             ((_cockpit->rightJoystickButtons[1] ? (1 << 1) : 0)) |
             ((_cockpit->rightJoystickButtons[2] ? (1 << 2) : 0)) |
             ((_cockpit->rightJoystickButtons[3] ? (1 << 3) : 0)) |
             ((_cockpit->rightJoystickButtons[4] ? (1 << 4) : 0)) |
             ((_cockpit->rightJoystickButtons[5] ? (1 << 5) : 0));

  // Byte 17: Botões do Joystick Esquerdo (6 bits)
  data[17] = (_cockpit->leftJoystickButtons[0] ? (1 << 0) : 0) |
             ((_cockpit->leftJoystickButtons[1] ? (1 << 1) : 0)) |
             ((_cockpit->leftJoystickButtons[2] ? (1 << 2) : 0)) |
             ((_cockpit->leftJoystickButtons[3] ? (1 << 3) : 0)) |
             ((_cockpit->leftJoystickButtons[4] ? (1 << 4) : 0)) |
             ((_cockpit->leftJoystickButtons[5] ? (1 << 5) : 0));

  // Bytes 18-20: Botões do Teclado (24 bits)
  // Byte 18 (keypadButtonStatus[0] a [7])
  data[18] = (_cockpit->keypadButtonStatus[0] ? (1 << 0) : 0) |
             ((_cockpit->keypadButtonStatus[1] ? (1 << 1) : 0)) |
             ((_cockpit->keypadButtonStatus[2] ? (1 << 2) : 0)) |
             ((_cockpit->keypadButtonStatus[3] ? (1 << 3) : 0)) |
             ((_cockpit->keypadButtonStatus[4] ? (1 << 4) : 0)) |
             ((_cockpit->keypadButtonStatus[5] ? (1 << 5) : 0)) |
             ((_cockpit->keypadButtonStatus[6] ? (1 << 6) : 0)) |
             ((_cockpit->keypadButtonStatus[7] ? (1 << 7) : 0));

  // Byte 19 (keypadButtonStatus[8] a [15])
  data[19] = (_cockpit->keypadButtonStatus[8] ? (1 << 0) : 0) |
             ((_cockpit->keypadButtonStatus[9] ? (1 << 1) : 0)) |
             ((_cockpit->keypadButtonStatus[10] ? (1 << 2) : 0)) |
             ((_cockpit->keypadButtonStatus[11] ? (1 << 3) : 0)) |
             ((_cockpit->keypadButtonStatus[12] ? (1 << 4) : 0)) |
             ((_cockpit->keypadButtonStatus[13] ? (1 << 5) : 0)) |
             ((_cockpit->keypadButtonStatus[14] ? (1 << 6) : 0)) |
             ((_cockpit->keypadButtonStatus[15] ? (1 << 7) : 0));

  // Byte 20 (keypadButtonStatus[16] a [23])
  data[20] = (_cockpit->keypadButtonStatus[16] ? (1 << 0) : 0) |
             ((_cockpit->keypadButtonStatus[17] ? (1 << 1) : 0)) |
             ((_cockpit->keypadButtonStatus[18] ? (1 << 2) : 0)) |
             ((_cockpit->keypadButtonStatus[19] ? (1 << 3) : 0)) |
             ((_cockpit->keypadButtonStatus[20] ? (1 << 4) : 0)) |
             ((_cockpit->keypadButtonStatus[21] ? (1 << 5) : 0)) |
             ((_cockpit->keypadButtonStatus[22] ? (1 << 6) : 0)) |
             ((_cockpit->keypadButtonStatus[23] ? (1 << 7) : 0)); // Assuming 24th button is index 23

  // Byte 21: Fim do frame
  data[21] = 0xE7;

  // Bytes 22 e 23: CRC16 Modbus (MSB primeiro, LSB depois)
  uint16_t calculatedCrc = calculateCRC16(data, 22); // Calcula CRC dos bytes 0 a 21
  data[22] = (uint8_t)(calculatedCrc >> 8);    // Byte alto do CRC (MSB)
  data[23] = (uint8_t)(calculatedCrc & 0xFF);  // Byte baixo do CRC (LSB)
}

// --- Função principal para manipular eventos WebSocket ---
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      break;
    case WStype_CONNECTED: {
      webSocket.sendTXT(num, "Conectado ao ESP32 via WebSocket!");
    }
      break;
    case WStype_TEXT:
      {
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, payload);

        if (error) {
          return;
        }

        // --- Mapeamento dos Eixos (JSON para struct cockpitNevil_t) ---
        // Função auxiliar para mapear valor float (-100 a 100) para ponto (0-31) e flags de direção
        auto mapAxisToStruct = [&](float value, uint8_t &point, bool &positiveFlag, bool &negativeFlag) {
          int absolute_value = abs(round(value));
          point = map(absolute_value, 0, 100, 0, 31);
          point = constrain(point, 0, 31); // Garante que esteja entre 0 e 31

          positiveFlag = (value > 0);
          negativeFlag = (value < 0);
        };

        // Mapeamento para Joystick Principal
        if (doc["joystickPrincipal"]["eixos"].containsKey("eixo1")) { // Y Esquerdo
          mapAxisToStruct(doc["joystickPrincipal"]["eixos"]["eixo1"]["x"].as<float>(),
                          myCockpitData.leftJoystickYPoint,
                          myCockpitData.leftJoystickFowardFlag,
                          myCockpitData.leftJoystickBackwardFlag);
        }
        if (doc["joystickPrincipal"]["eixos"].containsKey("eixo2")) { // X Esquerdo
          mapAxisToStruct(doc["joystickPrincipal"]["eixos"]["eixo2"]["x"].as<float>(),
                          myCockpitData.leftJoystickXPoint,
                          myCockpitData.leftJoystickRightFlag,
                          myCockpitData.leftJoystickLeftFlag);
        }
        if (doc["joystickPrincipal"]["eixos"].containsKey("eixo3")) { // Analógico Esquerdo 1
          mapAxisToStruct(doc["joystickPrincipal"]["eixos"]["eixo3"]["x"].as<float>(),
                          myCockpitData.leftAnalogic1Point,
                          myCockpitData.leftAnalogic1UpFlag,
                          myCockpitData.leftAnalogic1DwFlag);
        }
        if (doc["joystickPrincipal"]["eixos"].containsKey("eixo4")) { // Analógico Esquerdo 2
          mapAxisToStruct(doc["joystickPrincipal"]["eixos"]["eixo4"]["x"].as<float>(),
                          myCockpitData.leftAnalogic2Point,
                          myCockpitData.leftAnalogic2UpFlag,
                          myCockpitData.leftAnalogic2DwFlag);
        }
        if (doc["joystickPrincipal"]["eixos"].containsKey("eixo5")) { // Analógico Esquerdo 3
          mapAxisToStruct(doc["joystickPrincipal"]["eixos"]["eixo5"]["x"].as<float>(),
                          myCockpitData.leftAnalogic3Point,
                          myCockpitData.leftAnalogic3UpFlag,
                          myCockpitData.leftAnalogic3DwFlag);
        }
        if (doc["joystickPrincipal"]["eixos"].containsKey("eixo6")) { // Y Direito
          mapAxisToStruct(doc["joystickPrincipal"]["eixos"]["eixo6"]["x"].as<float>(),
                          myCockpitData.rightJoystickYPoint,
                          myCockpitData.rightJoystickFowardFlag,
                          myCockpitData.rightJoystickBackwardFlag);
        }
        if (doc["joystickPrincipal"]["eixos"].containsKey("eixo7")) { // X Direito
          mapAxisToStruct(doc["joystickPrincipal"]["eixos"]["eixo7"]["x"].as<float>(),
                          myCockpitData.rightJoystickXPoint,
                          myCockpitData.rightJoystickRightFlag,
                          myCockpitData.rightJoystickLeftFlag);
        }
        if (doc["joystickPrincipal"]["eixos"].containsKey("eixo8")) { // Analógico Direito 1
          mapAxisToStruct(doc["joystickPrincipal"]["eixos"]["eixo8"]["x"].as<float>(),
                          myCockpitData.rightAnalogic1Point,
                          myCockpitData.rightAnalogic1UpFlag,
                          myCockpitData.rightAnalogic1DwFlag);
        }

        // Mapeamento para Joystick 2
        if (doc["joystick2"]["eixos"].containsKey("eixo1")) { // Analógico Direito 2
          mapAxisToStruct(doc["joystick2"]["eixos"]["eixo1"]["x"].as<float>(),
                          myCockpitData.rightAnalogic2Point,
                          myCockpitData.rightAnalogic2UpFlag,
                          myCockpitData.rightAnalogic2DwFlag);
        }
        if (doc["joystick2"]["eixos"].containsKey("eixo2")) { // Analógico Direito 3
          mapAxisToStruct(doc["joystick2"]["eixos"]["eixo2"]["x"].as<float>(),
                          myCockpitData.rightAnalogic3Point,
                          myCockpitData.rightAnalogic3UpFlag,
                          myCockpitData.rightAnalogic3DwFlag);
        }
        if (doc["joystick2"]["eixos"].containsKey("eixo3")) { // Pedal Esquerdo
          mapAxisToStruct(doc["joystick2"]["eixos"]["eixo3"]["x"].as<float>(),
                          myCockpitData.leftPedalYPoint,
                          myCockpitData.leftPedalFowardFlag,
                          myCockpitData.leftPedalBackwardFlag);
        }
        if (doc["joystick2"]["eixos"].containsKey("eixo4")) { // Pedal Direito
          mapAxisToStruct(doc["joystick2"]["eixos"]["eixo4"]["x"].as<float>(),
                          myCockpitData.rightPedalYPoint,
                          myCockpitData.rightPedalFowardFlag,
                          myCockpitData.rightPedalBackwardFlag);
        }
        if (doc["joystick2"]["eixos"].containsKey("eixo5")) { // Pedal Adicional
          mapAxisToStruct(doc["joystick2"]["eixos"]["eixo5"]["x"].as<float>(),
                          myCockpitData.AddPedalYPoint,
                          myCockpitData.AddPedalFowardFlag,
                          myCockpitData.AddPedalBackwardFlag);
        }

        // --- Mapeamento dos Botões (JSON para struct cockpitNevil_t) ---
        if (doc["joystickPrincipal"].containsKey("botoes")) {
          for (int i = 0; i < 6; i++) { // Botões do Joystick Direito (btn1 a btn6)
            String btnName = "btn" + String(i + 1);
            if (doc["joystickPrincipal"]["botoes"].containsKey(btnName)) {
              myCockpitData.rightJoystickButtons[i] = doc["joystickPrincipal"]["botoes"][btnName].as<bool>();
            }
          }
          for (int i = 0; i < 6; i++) { // Botões do Joystick Esquerdo (btn7 a btn12)
            String btnName = "btn" + String(i + 7); // Começa do btn7
            if (doc["joystickPrincipal"]["botoes"].containsKey(btnName)) {
              myCockpitData.leftJoystickButtons[i] = doc["joystickPrincipal"]["botoes"][btnName].as<bool>();
            }
          }
        }

        if (doc["joystick2"].containsKey("botoes")) {
          for (int i = 0; i < 24; i++) { // Teclas do Teclado (btn1 a btn24)
            String btnName = "btn" + String(i + 1);
            if (doc["joystick2"]["botoes"].containsKey(btnName)) {
              myCockpitData.keypadButtonStatus[i] = doc["joystick2"]["botoes"][btnName].as<bool>();
            }
          }
        }

        // --- Definir specialMessage e addressCode (se não vierem do JSON) ---
        myCockpitData.specialMessage = 0xAA; // Exemplo de valor padrão
        myCockpitData.addressCode = 0x01;    // Exemplo de valor padrão

        // --- Monta o frame UART usando a struct preenchida ---
        buildCockpitNevilFrame(&myCockpitData, transmitBuffer);

        // --- Envia o frame via Serial (TX0/RX0) ---
        Serial.write(transmitBuffer, 24); // Alterado para Serial.write
      }
      break;
    case WStype_BIN:
    case WStype_ERROR:
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:
      break;
  }
}

void setup() {
  // Configura Serial (TX0/RX0) para comunicação com o receptor
  // Baud rate para o receptor
  Serial.begin(9600); 

  // Não precisamos configurar Serial1, pois não a usaremos para o frame.
  // Serial1.begin(9600); // Removido

  // Mensagens de inicialização de Wi-Fi.
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
  }

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  // Inicializa os dados da struct myCockpitData com valores padrão ou zero
  memset(&myCockpitData, 0, sizeof(myCockpitData));
  myCockpitData.specialMessage = 0xAA; // Valor padrão
  myCockpitData.addressCode = 0x01;    // Valor padrão
}

void loop() {
  webSocket.loop(); // Processa eventos WebSocket continuamente
}
