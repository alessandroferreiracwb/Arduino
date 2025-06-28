// ==== CÓDIGO FINAL E COMPLETO DO RECEPTOR (2 JOYSTICKS + EIXOS NOVOS) ====

#include <stdint.h>
#include <Joystick.h>

// --- Definições do Protocolo e do Frame ---
const int FRAME_SIZE = 24; const int DATA_PAYLOAD_LEN = 22; const byte EXPECTED_INIT_FRAME = 0xF1; const byte EXPECTED_END_FRAME = 0xE7;
byte receivedFrame[FRAME_SIZE]; byte payload[DATA_PAYLOAD_LEN]; 

// --- Objeto Joystick 1 (O principal, com eixos e 16 botões) ---
Joystick_ joystick1(
  JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_GAMEPAD,
  16, 0, true, true, true, true, true, true, true, true, true, true, false);

// --- Objeto Joystick 2 (O teclado de membrana, com 24 botões e sem eixos) ---
Joystick_ joystick2(
  JOYSTICK_DEFAULT_REPORT_ID + 1, JOYSTICK_TYPE_GAMEPAD,
  24, 0, false, false, false, false, false, false, false, false, false, false, false);


// --- Função CRC-16 (sem alterações) ---
uint16_t crc16(const byte *data, int len){uint16_t crc=0x0000;while(len--){crc^=*data++;for(uint8_t i=0;i<8;i++){if(crc&0x0001){crc=(crc>>1)^0x8408;}else{crc>>=1;}}}return crc;}

void setup() {
  Serial.begin(9600); Serial1.begin(9600);

  // --- Configuração dos Eixos do Joystick 1 (com faixa simétrica) ---
  joystick1.setXAxisRange(-512, 512); joystick1.setYAxisRange(-512, 512); joystick1.setZAxisRange(-512, 512); joystick1.setRxAxisRange(-512, 512); joystick1.setRyAxisRange(-512, 512); joystick1.setRzAxisRange(-512, 512); joystick1.setRudderRange(-512, 512); joystick1.setThrottleRange(-512, 512); joystick1.setAcceleratorRange(-512, 512); joystick1.setBrakeRange(-512, 512);
  
  // Inicia ambos os joysticks
  joystick1.begin(false);
  joystick2.begin(false);
  
  Serial.println("Receptor Final com 2 Joysticks e Eixos Novos pronto.");
}

void loop() {
  if (Serial1.available() >= FRAME_SIZE) {
    Serial1.readBytes(receivedFrame, FRAME_SIZE);

    // ETAPA 1: Organizar e Validar o Frame (sem alterações)
    for(int i=0; i<DATA_PAYLOAD_LEN; i++) { payload[i] = receivedFrame[i]; }
    uint16_t receivedCrc = (receivedFrame[22] << 8) | receivedFrame[23];
    uint16_t calculatedCrc = crc16(payload, DATA_PAYLOAD_LEN);

    if (payload[0] != EXPECTED_INIT_FRAME || payload[21] != EXPECTED_END_FRAME || calculatedCrc != receivedCrc) {
      Serial.println("ERRO: Frame invalido!");
      return; 
    }

    // --- ETAPA 2: SUCESSO! ATUALIZAR JOYSTICK 1 (EIXOS + 16 BOTÕES) ---
    
    // Decodificar e atualizar os 10 eixos
    int finalAxisValues[10];
    for (int i = 0; i < 10; i++) {
      byte rawAxisByte = payload[i + 3];
      int magnitude = rawAxisByte & 0x3F;
      bool goLeft   = bitRead(rawAxisByte, 6);
      bool goRight  = bitRead(rawAxisByte, 7);
      int mappedMagnitude = map(magnitude, 0, 63, 0, 512);
      int finalValue = 0;
      if (goRight && !goLeft) { finalValue = mappedMagnitude; } 
      else if (goLeft && !goRight) { finalValue = -mappedMagnitude; }
      finalAxisValues[i] = finalValue;
    }
    joystick1.setXAxis(finalAxisValues[0]); joystick1.setYAxis(finalAxisValues[1]); joystick1.setZAxis(finalAxisValues[2]); joystick1.setRxAxis(finalAxisValues[3]); joystick1.setRyAxis(finalAxisValues[4]); joystick1.setRzAxis(finalAxisValues[5]); joystick1.setRudder(finalAxisValues[6]); joystick1.setThrottle(finalAxisValues[7]); joystick1.setAccelerator(finalAxisValues[8]); joystick1.setBrake(finalAxisValues[9]);

    // Atualizar os 16 botões do Joystick 1 (bytes 13 e 14)
    for (int i=0; i<8; i++) { if (bitRead(payload[13], i) == 1) { joystick1.pressButton(i); } else { joystick1.releaseButton(i); } }
    for (int i=0; i<8; i++) { if (bitRead(payload[14], i) == 1) { joystick1.pressButton(i+8); } else { joystick1.releaseButton(i+8); } }

    // --- ETAPA 3: ATUALIZAR JOYSTICK 2 (24 TECLAS DE MEMBRANA) ---
    // Atualizar as 24 teclas (bytes 15, 16 e 17)
    for (int i=0; i<8; i++) { if (bitRead(payload[15], i) == 1) { joystick2.pressButton(i); } else { joystick2.releaseButton(i); } }
    for (int i=0; i<8; i++) { if (bitRead(payload[16], i) == 1) { joystick2.pressButton(i+8); } else { joystick2.releaseButton(i+8); } }
    for (int i=0; i<8; i++) { if (bitRead(payload[17], i) == 1) { joystick2.pressButton(i+16); } else { joystick2.releaseButton(i+16); } }
    
    // --- ETAPA 4: ENVIAR ESTADO DE AMBOS OS JOYSTICKS ---
    joystick1.sendState();
    joystick2.sendState();

    Serial.println("Frame OK. Joysticks 1 e 2 atualizados.");
  }
}
