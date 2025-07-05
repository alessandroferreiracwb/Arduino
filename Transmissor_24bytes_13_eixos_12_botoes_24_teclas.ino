#include <Arduino.h>

// Definição da mesma estrutura do receptor (mantida inalterada)
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
  uint8_t leftJoystickButtons[6];   // 6 bits

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
  // As flags são os 2 bits menos significativos, o ponto são os 5 bits seguintes.
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

// Variáveis globais para emulação
cockpitNevil_t myCockpitData;
uint8_t transmitBuffer[24]; // Buffer para armazenar o frame de transmissão

long lastEmulationMillis = 0;
const long EMULATION_INTERVAL_MS = 200; // Intervalo para cada botão/tecla piscar e para os eixos se moverem

// Total de botões: 6 (joystick direito) + 6 (joystick esquerdo) + 24 (teclado) = 36
const int TOTAL_BUTTONS_AND_KEYS = 6 + 6 + 24;
int currentSequenceIndex = 0; // Índice do botão/tecla atual na sequência

// Variáveis para emulação dos eixos
// Direção: 1 para aumentar, -1 para diminuir
int leftJoyYDirection = 1;
int leftJoyXDirection = 1;
int leftAnalogic1Direction = 1;
int leftAnalogic2Direction = 1;
int leftAnalogic3Direction = 1;

int rightJoyYDirection = 1;
int rightJoyXDirection = 1;
int rightAnalogic1Direction = 1;
int rightAnalogic2Direction = 1;
int rightAnalogic3Direction = 1;

int leftPedalDirection = 1;
int rightPedalDirection = 1;
int addPedalDirection = 1;

// Constantes para os valores dos eixos
const int MAX_POINT_VALUE = 31; // Valor máximo para os pontos dos joysticks/analógicos (0-31)
const int JOYSTICK_CENTER = 15; // Ponto central para joysticks e analógicos (0-31)
const int PEDAL_MIN = 0;        // Ponto mínimo para pedais
const int PEDAL_MAX = 31;       // Ponto máximo para pedais
const int AXIS_STEP = 1;        // Quanto o valor do eixo muda a cada intervalo

/**
 * @brief Simula o movimento de um eixo analógico (joystick ou analógico).
 * @param point O valor atual do ponto do eixo (0-31).
 * @param direction A direção do movimento (1 para aumentar, -1 para diminuir).
 * @param positiveFlag Flag de movimento para frente/cima/direita.
 * @param negativeFlag Flag de movimento para trás/baixo/esquerda.
 * @param center O valor central do eixo. (Não usado diretamente na lógica de limite, mas útil para referência)
 */
void simulateAnalogAxis(uint8_t &point, int &direction, bool &positiveFlag, bool &negativeFlag, int center) {
  point += (uint8_t)direction * AXIS_STEP;

  // Ajusta a direção e as flags quando atinge os limites
  if (direction == 1) { // Indo para o máximo
    positiveFlag = true;
    negativeFlag = false;
    if (point >= MAX_POINT_VALUE) {
      point = MAX_POINT_VALUE;
      direction = -1; // Inverte a direção
    }
  } else { // Indo para o mínimo
    positiveFlag = false;
    negativeFlag = true;
    if (point <= 0) {
      point = 0;
      direction = 1; // Inverte a direção
    }
  }
}

/**
 * @brief Simula o movimento de um eixo de pedal (0-31).
 * @param point O valor atual do ponto do pedal (0-31).
 * @param direction A direção do movimento (1 para aumentar, -1 para diminuir).
 * @param forwardFlag Flag de movimento para frente.
 * @param backwardFlag Flag de movimento para trás.
 */
void simulatePedalAxis(uint8_t &point, int &direction, bool &forwardFlag, bool &backwardFlag) {
  point += (uint8_t)direction * AXIS_STEP;

  // Ajusta a direção e as flags quando atinge os limites
  if (direction == 1) { // Indo para o máximo (pedal pressionado)
    forwardFlag = true;
    backwardFlag = false;
    if (point >= PEDAL_MAX) {
      point = PEDAL_MAX;
      direction = -1; // Inverte a direção
    }
  } else { // Indo para o mínimo (pedal solto)
    forwardFlag = false;
    backwardFlag = true;
    if (point <= PEDAL_MIN) {
      point = PEDAL_MIN;
      direction = 1; // Inverte a direção
    }
    // Para pedais, se o ponto estiver no mínimo, ambas as flags podem ser falsas se não houver movimento.
    // No entanto, para fins de simulação de movimento contínuo, a flag "backward" é ativada ao ir para 0.
  }
}


void setup() {
  Serial.begin(9600); // Para depuração
  Serial1.begin(9600); // Para comunicação com o receptor

  // Inicializa os dados da struct
  myCockpitData.specialMessage = 0xAA;
  myCockpitData.addressCode = 0x01;

  // Zera todos os botões no início
  for (int i = 0; i < 6; i++) {
    myCockpitData.rightJoystickButtons[i] = 0;
    myCockpitData.leftJoystickButtons[i] = 0;
  }
  for (int i = 0; i < 24; i++) {
    myCockpitData.keypadButtonStatus[i] = 0;
  }

  // Inicializa os joysticks e pedais para o centro/zero
  myCockpitData.leftJoystickYPoint = JOYSTICK_CENTER;
  myCockpitData.leftJoystickXPoint = JOYSTICK_CENTER;
  myCockpitData.leftAnalogic1Point = JOYSTICK_CENTER;
  myCockpitData.leftAnalogic2Point = JOYSTICK_CENTER;
  myCockpitData.leftAnalogic3Point = JOYSTICK_CENTER;

  myCockpitData.rightJoystickYPoint = JOYSTICK_CENTER;
  myCockpitData.rightJoystickXPoint = JOYSTICK_CENTER;
  myCockpitData.rightAnalogic1Point = JOYSTICK_CENTER;
  myCockpitData.rightAnalogic2Point = JOYSTICK_CENTER;
  myCockpitData.rightAnalogic3Point = JOYSTICK_CENTER;

  myCockpitData.leftPedalYPoint = PEDAL_MIN;
  myCockpitData.rightPedalYPoint = PEDAL_MIN;
  myCockpitData.AddPedalYPoint = PEDAL_MIN;
}

void loop() {
  // Controle de tempo para a sequência de botões/teclas E para os movimentos dos eixos
  if (millis() - lastEmulationMillis > EMULATION_INTERVAL_MS) {
    lastEmulationMillis = millis();

    // --- Simulação dos Botões/Teclas ---
    // Desliga o botão/tecla anterior (se houver)
    if (currentSequenceIndex > 0) {
      int prevIndex = currentSequenceIndex - 1;
      if (prevIndex < 6) { // Botões do Joystick Direito
        myCockpitData.rightJoystickButtons[prevIndex] = 0;
      } else if (prevIndex < 12) { // Botões do Joystick Esquerdo
        myCockpitData.leftJoystickButtons[prevIndex - 6] = 0;
      } else { // Teclas do Teclado
        myCockpitData.keypadButtonStatus[prevIndex - 12] = 0;
      }
    } else { // Se for o início de um novo ciclo, desliga o último botão do ciclo anterior
      myCockpitData.keypadButtonStatus[23] = 0; // Último botão do teclado
    }

    // Liga o botão/tecla atual
    if (currentSequenceIndex < 6) { // Botões do Joystick Direito (0-5)
      myCockpitData.rightJoystickButtons[currentSequenceIndex] = 1;
      //Serial.print("RJ Button: "); Serial.println(currentSequenceIndex);
    } else if (currentSequenceIndex < 12) { // Botões do Joystick Esquerdo (6-11)
      myCockpitData.leftJoystickButtons[currentSequenceIndex - 6] = 1;
      //Serial.print("LJ Button: "); Serial.println(currentSequenceIndex - 6);
    } else { // Teclas do Teclado (12-35)
      myCockpitData.keypadButtonStatus[currentSequenceIndex - 12] = 1;
      //Serial.print("Keypad Button: "); Serial.println(currentSequenceIndex - 12);
    }

    // Avança para o próximo botão/tecla na sequência
    currentSequenceIndex++;
    if (currentSequenceIndex >= TOTAL_BUTTONS_AND_KEYS) {
      currentSequenceIndex = 0; // Reinicia a sequência
    }

    // --- Simulação dos Eixos ---
    // Joysticks e Analógicos (movimento de 0 a 31 e volta)
    simulateAnalogAxis(myCockpitData.leftJoystickYPoint, leftJoyYDirection, myCockpitData.leftJoystickFowardFlag, myCockpitData.leftJoystickBackwardFlag, JOYSTICK_CENTER);
    simulateAnalogAxis(myCockpitData.leftJoystickXPoint, leftJoyXDirection, myCockpitData.leftJoystickRightFlag, myCockpitData.leftJoystickLeftFlag, JOYSTICK_CENTER);
    simulateAnalogAxis(myCockpitData.leftAnalogic1Point, leftAnalogic1Direction, myCockpitData.leftAnalogic1UpFlag, myCockpitData.leftAnalogic1DwFlag, JOYSTICK_CENTER);
    simulateAnalogAxis(myCockpitData.leftAnalogic2Point, leftAnalogic2Direction, myCockpitData.leftAnalogic2UpFlag, myCockpitData.leftAnalogic2DwFlag, JOYSTICK_CENTER);
    simulateAnalogAxis(myCockpitData.leftAnalogic3Point, leftAnalogic3Direction, myCockpitData.leftAnalogic3UpFlag, myCockpitData.leftAnalogic3DwFlag, JOYSTICK_CENTER);

    simulateAnalogAxis(myCockpitData.rightJoystickYPoint, rightJoyYDirection, myCockpitData.rightJoystickFowardFlag, myCockpitData.rightJoystickBackwardFlag, JOYSTICK_CENTER);
    simulateAnalogAxis(myCockpitData.rightJoystickXPoint, rightJoyXDirection, myCockpitData.rightJoystickRightFlag, myCockpitData.rightJoystickLeftFlag, JOYSTICK_CENTER);
    simulateAnalogAxis(myCockpitData.rightAnalogic1Point, rightAnalogic1Direction, myCockpitData.rightAnalogic1UpFlag, myCockpitData.rightAnalogic1DwFlag, JOYSTICK_CENTER);
    simulateAnalogAxis(myCockpitData.rightAnalogic2Point, rightAnalogic2Direction, myCockpitData.rightAnalogic2UpFlag, myCockpitData.rightAnalogic2DwFlag, JOYSTICK_CENTER);
    simulateAnalogAxis(myCockpitData.rightAnalogic3Point, rightAnalogic3Direction, myCockpitData.rightAnalogic3UpFlag, myCockpitData.rightAnalogic3DwFlag, JOYSTICK_CENTER);

    // Pedais (movimento de 0 a 31 e volta)
    simulatePedalAxis(myCockpitData.leftPedalYPoint, leftPedalDirection, myCockpitData.leftPedalFowardFlag, myCockpitData.leftPedalBackwardFlag);
    simulatePedalAxis(myCockpitData.rightPedalYPoint, rightPedalDirection, myCockpitData.rightPedalFowardFlag, myCockpitData.rightPedalBackwardFlag);
    simulatePedalAxis(myCockpitData.AddPedalYPoint, addPedalDirection, myCockpitData.AddPedalFowardFlag, myCockpitData.AddPedalBackwardFlag);
/*
    // Opcional: Imprime os valores dos joysticks para depuração
    Serial.print("LJ Y: "); Serial.print(myCockpitData.leftJoystickYPoint);
    Serial.print(" (F:"); Serial.print(myCockpitData.leftJoystickFowardFlag);
    Serial.print(" B:"); Serial.print(myCockpitData.leftJoystickBackwardFlag);
    Serial.print(") | RJ X: "); Serial.print(myCockpitData.rightJoystickXPoint);
    Serial.print(" (R:"); Serial.print(myCockpitData.rightJoystickRightFlag);
    Serial.print(" L:"); Serial.print(myCockpitData.rightJoystickLeftFlag);
    Serial.print(") | LP: "); Serial.println(myCockpitData.leftPedalYPoint);*/
  }

  // Monta o frame de transmissão com os dados atualizados
  buildCockpitNevilFrame(&myCockpitData, transmitBuffer);

  // Envia o frame via Serial1
  Serial1.write(transmitBuffer, 24);

  // Pequeno delay para não sobrecarregar a comunicação.
  // O EMULATION_INTERVAL_MS já controla a velocidade da sequência.
  delay(50);
}
