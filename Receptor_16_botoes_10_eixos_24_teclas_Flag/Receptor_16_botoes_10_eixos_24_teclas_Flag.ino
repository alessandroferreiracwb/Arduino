// ==== CÓDIGO FINAL DO RECEPTOR (COM LÓGICA DE RESSINCRONIZAÇÃO) ====
// Este código recebe um pacote de dados de 24 bytes, valida a sua integridade e
// usa os dados para controlar dois joysticks virtuais distintos.
// Ele é capaz de se recuperar de erros de comunicação procurando ativamente pelo
// byte de início de frame (0xF1).

#include <stdint.h>
#include <Joystick.h>

// --- Definições do Protocolo e do Frame ---
const int FRAME_SIZE = 24;
const int DATA_PAYLOAD_LEN = 22;
const byte EXPECTED_INIT_FRAME = 0xF1;
const byte EXPECTED_END_FRAME = 0xE7;

// --- Variáveis Globais ---
byte receivedFrame[FRAME_SIZE];
byte payload[DATA_PAYLOAD_LEN];

// --- Criação dos Dispositivos Joystick Virtuais ---
Joystick_ joystick1(
  JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_GAMEPAD,
  16, 0, true, true, true, true, true, true, true, true, true, true, false);
Joystick_ joystick2(
  JOYSTICK_DEFAULT_REPORT_ID + 1, JOYSTICK_TYPE_GAMEPAD,
  24, 0, false, false, false, false, false, false, false, false, false, false, false);

// --- Função de Cálculo CRC-16 ---
uint16_t crc16(const byte *data, int len) {
  uint16_t crc = 0x0000;
  while (len--) {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; i++) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0x8408;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

// --- Configuração Inicial ---
void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);

  joystick1.setXAxisRange(-512, 512); joystick1.setYAxisRange(-512, 512); joystick1.setZAxisRange(-512, 512);
  joystick1.setRxAxisRange(-512, 512); joystick1.setRyAxisRange(-512, 512); joystick1.setRzAxisRange(-512, 512);
  joystick1.setRudderRange(-512, 512); joystick1.setThrottleRange(-512, 512); joystick1.setAcceleratorRange(-512, 512); joystick1.setBrakeRange(-512, 512);
  
  joystick1.begin(false);
  joystick2.begin(false);
  
  Serial.println("Receptor Robusto (com Ressincronizacao) pronto.");
}

// --- Loop Principal ---
void loop() {
  // --- MUDANÇA PRINCIPAL: LÓGICA DE "CAÇA" AO FRAME ---
  // Verifica se há pelo menos UM byte disponível para ler.
  if (Serial1.available() > 0) {
    // Lê um byte e verifica se é o nosso byte de início de frame.
    if (Serial1.read() == EXPECTED_INIT_FRAME) {
      
      // Byte de início encontrado! Agora, tenta ler o resto do frame (23 bytes).
      // A função readBytes tem um timeout padrão (1 segundo), então ela esperará um pouco
      // pelos bytes restantes antes de desistir.
      int bytesRead = Serial1.readBytes(receivedFrame + 1, FRAME_SIZE - 1);

      // Verifica se recebemos todos os 23 bytes restantes.
      if (bytesRead == FRAME_SIZE - 1) {
        // --- FRAME COMPLETO RECEBIDO ---
        // Colocamos o byte de início de volta na primeira posição do nosso array.
        receivedFrame[0] = EXPECTED_INIT_FRAME;

        // ETAPA 1: Organizar e Validar o Frame
        for (int i = 0; i < DATA_PAYLOAD_LEN; i++) {
          payload[i] = receivedFrame[i];
        }
        uint16_t receivedCrc = (receivedFrame[22] << 8) | receivedFrame[23];
        uint16_t calculatedCrc = crc16(payload, DATA_PAYLOAD_LEN);

        // Validação tripla. Se falhar, simplesmente ignora e volta a caçar.
        if (payload[0] == EXPECTED_INIT_FRAME && payload[21] == EXPECTED_END_FRAME && calculatedCrc == receivedCrc) {
          // --- SUCESSO! O FRAME É VÁLIDO ---
          // Atualiza os dois joysticks com os dados do frame.
          
          // Joystick 1: Eixos
          int finalAxisValues[10];
          for (int i = 0; i < 10; i++) {
            byte rawAxisByte = payload[i + 3];
            int magnitude = rawAxisByte & 0x3F;
            bool goLeft = bitRead(rawAxisByte, 6);
            bool goRight = bitRead(rawAxisByte, 7);
            int mappedMagnitude = map(magnitude, 0, 63, 0, 512);
            int finalValue = 0;
            if (goRight && !goLeft) { finalValue = mappedMagnitude; } 
            else if (goLeft && !goRight) { finalValue = -mappedMagnitude; }
            finalAxisValues[i] = finalValue;
          }
          joystick1.setXAxis(finalAxisValues[0]); joystick1.setYAxis(finalAxisValues[1]); joystick1.setZAxis(finalAxisValues[2]);
          joystick1.setRxAxis(finalAxisValues[3]); joystick1.setRyAxis(finalAxisValues[4]); joystick1.setRzAxis(finalAxisValues[5]);
          joystick1.setRudder(finalAxisValues[6]); joystick1.setThrottle(finalAxisValues[7]); joystick1.setAccelerator(finalAxisValues[8]); joystick1.setBrake(finalAxisValues[9]);

          // Joystick 1: Botões
          for (int i = 0; i < 8; i++) { if (bitRead(payload[13], i) == 1) { joystick1.pressButton(i); } else { joystick1.releaseButton(i); } }
          for (int i = 0; i < 8; i++) { if (bitRead(payload[14], i) == 1) { joystick1.pressButton(i + 8); } else { joystick1.releaseButton(i + 8); } }

          // Joystick 2: Teclas
          for (int i = 0; i < 8; i++) { if (bitRead(payload[15], i) == 1) { joystick2.pressButton(i); } else { joystick2.releaseButton(i); } }
          for (int i = 0; i < 8; i++) { if (bitRead(payload[16], i) == 1) { joystick2.pressButton(i + 8); } else { joystick2.releaseButton(i + 8); } }
          for (int i = 0; i < 8; i++) { if (bitRead(payload[17], i) == 1) { joystick2.pressButton(i + 16); } else { joystick2.releaseButton(i + 16); } }
          
          // Envia o estado de ambos
          joystick1.sendState();
          joystick2.sendState();
          Serial.println("Frame OK e SINCRONIZADO. Joysticks atualizados.");
        }
      }
    }
    // Se o byte lido não era 0xF1, ele é simplesmente ignorado e o loop continua,
    // efetivamente limpando o buffer de dados ruins.
  }
}
