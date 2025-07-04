#include <Arduino.h>

// Definição da mesma estrutura do receptor
typedef struct cockpitNevil
{
  /*byte 0: init of frame, default value is 0xF1 (changed to 0xF1 for transmitter side)*/

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
  uint8_t leftJoystickButtons[6];  // 6 bits

  /*byte 18 - 20*/
  uint8_t keypadButtonStatus[24]; // Assuming 24 bits as per the parsing (0-23)

  /*byte 21: end of frame, default value is 0xE7*/

  /*byte 22 and byte 23: CRC16 Modbus*/
} cockpitNevil_t;

// Função para calcular CRC16 (a mesma que você já tem)
uint16_t crc16(unsigned char* data_p, int len)
{
  uint16_t _crc = 0xFFFF;
  char bit2 = 0;

  for (uint16_t i = 0; i < len; i++)
  {
    _crc ^= data_p[i];

    for (bit2 = 0; bit2 < 8; bit2++)
    {
      if (_crc & 0x0001)
      {
        _crc >>= 1;
        _crc ^= 0xA001;
      }
      else
      {
        _crc >>= 1;
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
void buildCockpitNevilFrame(const cockpitNevil_t *_cockpit, uint8_t *data)
{
  // Byte 0: Início do frame
  data[0] = 0xF1;

  // Byte 1: specialMessage
  data[1] = _cockpit->specialMessage;

  // Byte 2: addressCode
  data[2] = _cockpit->addressCode;

  // Bytes 3-7: Joysticks Esquerdos
  data[3] = (_cockpit->leftJoystickFowardFlag) |
            ((_cockpit->leftJoystickBackwardFlag) << 1) |
            ((_cockpit->leftJoystickYPoint & 0x1F) << 2); // 5 bits para o ponto

  data[4] = (_cockpit->leftJoystickRightFlag) |
            ((_cockpit->leftJoystickLeftFlag) << 1) |
            ((_cockpit->leftJoystickXPoint & 0x1F) << 2);

  data[5] = (_cockpit->leftAnalogic1UpFlag) |
            ((_cockpit->leftAnalogic1DwFlag) << 1) |
            ((_cockpit->leftAnalogic1Point & 0x1F) << 2);

  data[6] = (_cockpit->leftAnalogic2UpFlag) |
            ((_cockpit->leftAnalogic2DwFlag) << 1) |
            ((_cockpit->leftAnalogic2Point & 0x1F) << 2);

  data[7] = (_cockpit->leftAnalogic3UpFlag) |
            ((_cockpit->leftAnalogic3DwFlag) << 1) |
            ((_cockpit->leftAnalogic3Point & 0x1F) << 2);

  // Bytes 8-12: Joysticks Direitos
  data[8] = (_cockpit->rightJoystickFowardFlag) |
            ((_cockpit->rightJoystickBackwardFlag) << 1) |
            ((_cockpit->rightJoystickYPoint & 0x1F) << 2);

  data[9] = (_cockpit->rightJoystickRightFlag) |
            ((_cockpit->rightJoystickLeftFlag) << 1) |
            ((_cockpit->rightJoystickXPoint & 0x1F) << 2);

  data[10] = (_cockpit->rightAnalogic1UpFlag) |
             ((_cockpit->rightAnalogic1DwFlag) << 1) |
             ((_cockpit->rightAnalogic1Point & 0x1F) << 2);

  data[11] = (_cockpit->rightAnalogic2UpFlag) |
             ((_cockpit->rightAnalogic2DwFlag) << 1) |
             ((_cockpit->rightAnalogic2Point & 0x1F) << 2);

  data[12] = (_cockpit->rightAnalogic3UpFlag) |
             ((_cockpit->rightAnalogic3DwFlag) << 1) |
             ((_cockpit->rightAnalogic3Point & 0x1F) << 2);

  // Bytes 13-15: Pedais
  data[13] = (_cockpit->leftPedalFowardFlag) |
             ((_cockpit->leftPedalBackwardFlag) << 1) |
             ((_cockpit->leftPedalYPoint & 0x1F) << 2);

  data[14] = (_cockpit->rightPedalFowardFlag) |
             ((_cockpit->rightPedalBackwardFlag) << 1) |
             ((_cockpit->rightPedalYPoint & 0x1F) << 2);

  data[15] = (_cockpit->AddPedalFowardFlag) |
             ((_cockpit->AddPedalBackwardFlag) << 1) |
             ((_cockpit->AddPedalYPoint & 0x1F) << 2);

  // Byte 16: Botões do Joystick Direito (6 bits)
  data[16] = (_cockpit->rightJoystickButtons[0]) |
             ((_cockpit->rightJoystickButtons[1]) << 1) |
             ((_cockpit->rightJoystickButtons[2]) << 2) |
             ((_cockpit->rightJoystickButtons[3]) << 3) |
             ((_cockpit->rightJoystickButtons[4]) << 4) |
             ((_cockpit->rightJoystickButtons[5]) << 5);

  // Byte 17: Botões do Joystick Esquerdo (6 bits)
  data[17] = (_cockpit->leftJoystickButtons[0]) |
             ((_cockpit->leftJoystickButtons[1]) << 1) |
             ((_cockpit->leftJoystickButtons[2]) << 2) |
             ((_cockpit->leftJoystickButtons[3]) << 3) |
             ((_cockpit->leftJoystickButtons[4]) << 4) |
             ((_cockpit->leftJoystickButtons[5]) << 5);

  // Bytes 18-20: Botões do Teclado (24 bits)
  // Byte 18 (keypadButtonStatus[0] a [7])
  data[18] = (_cockpit->keypadButtonStatus[0]) |
             ((_cockpit->keypadButtonStatus[1]) << 1) |
             ((_cockpit->keypadButtonStatus[2]) << 2) |
             ((_cockpit->keypadButtonStatus[3]) << 3) |
             ((_cockpit->keypadButtonStatus[4]) << 4) |
             ((_cockpit->keypadButtonStatus[5]) << 5) |
             ((_cockpit->keypadButtonStatus[6]) << 6) |
             ((_cockpit->keypadButtonStatus[7]) << 7);

  // Byte 19 (keypadButtonStatus[8] a [15])
  data[19] = (_cockpit->keypadButtonStatus[8]) |
             ((_cockpit->keypadButtonStatus[9]) << 1) |
             ((_cockpit->keypadButtonStatus[10]) << 2) |
             ((_cockpit->keypadButtonStatus[11]) << 3) |
             ((_cockpit->keypadButtonStatus[12]) << 4) |
             ((_cockpit->keypadButtonStatus[13]) << 5) |
             ((_cockpit->keypadButtonStatus[14]) << 6) |
             ((_cockpit->keypadButtonStatus[15]) << 7);

  // Byte 20 (keypadButtonStatus[16] a [23])
  data[20] = (_cockpit->keypadButtonStatus[16]) |
             ((_cockpit->keypadButtonStatus[17]) << 1) |
             ((_cockpit->keypadButtonStatus[18]) << 2) |
             ((_cockpit->keypadButtonStatus[19]) << 3) |
             ((_cockpit->keypadButtonStatus[20]) << 4) |
             ((_cockpit->keypadButtonStatus[21]) << 5) |
             ((_cockpit->keypadButtonStatus[22]) << 6) |
             ((_cockpit->keypadButtonStatus[23]) << 7); // Assuming 24th button is index 23

  // Byte 21: Fim do frame
  data[21] = 0xE7;

  // Bytes 22 e 23: CRC16 Modbus
  uint16_t calculatedCrc = crc16(data, 22); // Calcula CRC dos bytes 0 a 21
  data[22] = (uint8_t)(calculatedCrc >> 8);   // Byte alto do CRC
  data[23] = (uint8_t)(calculatedCrc & 0xFF); // Byte baixo do CRC
}

// Exemplo de uso
cockpitNevil_t myCockpitData;
uint8_t transmitBuffer[24]; // Buffer para armazenar o frame de transmissão

void setup() {
  Serial.begin(9600); // Para depuração
  Serial1.begin(9600); // Para comunicação com o receptor

  // Inicializa os dados do seu "cockpit" (apenas um exemplo)
  myCockpitData.specialMessage = 0xAA;
  myCockpitData.addressCode = 0x01;

  myCockpitData.leftJoystickFowardFlag = true;
  myCockpitData.leftJoystickBackwardFlag = false;
  myCockpitData.leftJoystickYPoint = 15; // Valor entre 0-31

  myCockpitData.leftJoystickRightFlag = false;
  myCockpitData.leftJoystickLeftFlag = true;
  myCockpitData.leftJoystickXPoint = 20; // Valor entre 0-31

  myCockpitData.leftAnalogic1UpFlag = true;
  myCockpitData.leftAnalogic1DwFlag = false;
  myCockpitData.leftAnalogic1Point = 10;

  myCockpitData.leftAnalogic2UpFlag = false;
  myCockpitData.leftAnalogic2DwFlag = true;
  myCockpitData.leftAnalogic2Point = 25;

  myCockpitData.leftAnalogic3UpFlag = true;
  myCockpitData.leftAnalogic3DwFlag = false;
  myCockpitData.leftAnalogic3Point = 5;

  myCockpitData.rightJoystickFowardFlag = false;
  myCockpitData.rightJoystickBackwardFlag = true;
  myCockpitData.rightJoystickYPoint = 30;

  myCockpitData.rightJoystickRightFlag = true;
  myCockpitData.rightJoystickLeftFlag = false;
  myCockpitData.rightJoystickXPoint = 7;

  myCockpitData.rightAnalogic1UpFlag = false;
  myCockpitData.rightAnalogic1DwFlag = true;
  myCockpitData.rightAnalogic1Point = 12;

  myCockpitData.rightAnalogic2UpFlag = true;
  myCockpitData.rightAnalogic2DwFlag = false;
  myCockpitData.rightAnalogic2Point = 28;

  myCockpitData.rightAnalogic3UpFlag = false;
  myCockpitData.rightAnalogic3DwFlag = true;
  myCockpitData.rightAnalogic3Point = 3;

  myCockpitData.leftPedalFowardFlag = true;
  myCockpitData.leftPedalBackwardFlag = false;
  myCockpitData.leftPedalYPoint = 22;

  myCockpitData.rightPedalFowardFlag = false;
  myCockpitData.rightPedalBackwardFlag = true;
  myCockpitData.rightPedalYPoint = 8;

  myCockpitData.AddPedalFowardFlag = true;
  myCockpitData.AddPedalBackwardFlag = false;
  myCockpitData.AddPedalYPoint = 18;

  // Botões do Joystick Direito (exemplo: botão 0 e 2 pressionados)
  myCockpitData.rightJoystickButtons[0] = 1;
  myCockpitData.rightJoystickButtons[1] = 0;
  myCockpitData.rightJoystickButtons[2] = 1;
  myCockpitData.rightJoystickButtons[3] = 0;
  myCockpitData.rightJoystickButtons[4] = 0;
  myCockpitData.rightJoystickButtons[5] = 0;

  // Botões do Joystick Esquerdo (exemplo: botão 1 e 3 pressionados)
  myCockpitData.leftJoystickButtons[0] = 0;
  myCockpitData.leftJoystickButtons[1] = 1;
  myCockpitData.leftJoystickButtons[2] = 0;
  myCockpitData.leftJoystickButtons[3] = 1;
  myCockpitData.leftJoystickButtons[4] = 0;
  myCockpitData.leftJoystickButtons[5] = 0;

  // Botões do Teclado (exemplo: botões 0, 8 e 16 pressionados)
  for (int i = 0; i < 24; i++) {
    myCockpitData.keypadButtonStatus[i] = 0; // Zera todos
  }
  myCockpitData.keypadButtonStatus[0] = 1;
  myCockpitData.keypadButtonStatus[8] = 1;
  myCockpitData.keypadButtonStatus[16] = 1;
}

void loop() {
  // Monta o frame de transmissão
  buildCockpitNevilFrame(&myCockpitData, transmitBuffer);

  // Envia o frame via Serial1
  Serial1.write(transmitBuffer, 24);

  // Opcional: Imprime o frame enviado no Serial para depuração
  Serial.print("Frame enviado (BIN): ");
  for (int i = 0; i < 24; i++) {
    Serial.print(transmitBuffer[i], BIN);
    Serial.print(" ");
  }
  Serial.println();

  Serial.print("Frame enviado (HEX): ");
  for (int i = 0; i < 24; i++) {
    if (transmitBuffer[i] < 0x10) Serial.print("0");
    Serial.print(transmitBuffer[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  delay(100); // Envia a cada 100ms, ajuste conforme a necessidade
}