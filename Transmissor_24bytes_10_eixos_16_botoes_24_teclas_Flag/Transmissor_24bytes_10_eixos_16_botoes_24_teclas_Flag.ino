// ==== CÓDIGO DO TRANSMISSOR (COM NOVO FORMATO DE EIXO) ====

#include <stdint.h>

// --- Definições do Protocolo e do Frame ---
const int FRAME_SIZE = 24; const int DATA_PAYLOAD_LEN = 22; const byte Init_Frame = 0xF1; const byte Cod_end1 = 0x0E; const byte Cod_end2 = 0xD1; const byte End_Frame = 0xE7;

// --- Controle de Dados e Tempo ---
const int DELAY_POR_PASSO = 100; // Delay entre cada passo da simulação

// --- Variáveis Globais ---
byte dataFrame[FRAME_SIZE]; int botaoAtivo = 0; int teclaAtivo = 0;

// --- Função CRC-16 (sem alterações) ---
uint16_t crc16(const byte *data, int len){uint16_t crc=0x0000;while(len--){crc^=*data++;for(uint8_t i=0;i<8;i++){if(crc&0x0001){crc=(crc>>1)^0x8408;}else{crc>>=1;}}}return crc;}

void setup() {
  Serial1.begin(9600);
}

// Função auxiliar para montar e enviar o frame
void enviarFrame(byte valorEixo) {
  // 1. Monta o payload
  dataFrame[0] = Init_Frame;
  dataFrame[1] = Cod_end1;
  dataFrame[2] = Cod_end2;
  // Aplica o mesmo valor de eixo para todos os 10 eixos
  for (int j = 3; j <= 12; j++) { dataFrame[j] = valorEixo; }

  // Lógica dos botões e teclas (sem alterações)
  byte byteBotoes1_8 = 0; byte byteBotoes9_16 = 0;
  if (botaoAtivo < 8) { byteBotoes1_8 = bit(botaoAtivo); } else { byteBotoes9_16 = bit(botaoAtivo - 8); }
  dataFrame[13] = byteBotoes1_8; dataFrame[14] = byteBotoes9_16;
  byte byteTeclas1_8=0; byte byteTeclas9_16=0; byte byteTeclas17_24=0;
  if (teclaAtivo < 8) { byteTeclas1_8 = bit(teclaAtivo); } else if (teclaAtivo < 16) { byteTeclas9_16 = bit(teclaAtivo - 8); } else { byteTeclas17_24 = bit(teclaAtivo - 16); }
  dataFrame[15] = byteTeclas1_8; dataFrame[16] = byteTeclas9_16; dataFrame[17] = byteTeclas17_24;
  for (int j = 18; j <= 20; j++) { dataFrame[j] = 0x00; }
  dataFrame[21] = End_Frame;

  // 2. Calcula e insere o CRC
  uint16_t crcResult = crc16(dataFrame, DATA_PAYLOAD_LEN);
  dataFrame[22] = (crcResult >> 8) & 0xFF; dataFrame[23] = crcResult & 0xFF;

  // 3. Envia e atualiza contadores
  Serial1.write(dataFrame, FRAME_SIZE);
  botaoAtivo = (botaoAtivo + 1) % 16;
  teclaAtivo = (teclaAtivo + 1) % 24;
  delay(DELAY_POR_PASSO);
}


void loop() {
  // --- FASE 1: Mover para a Direita (Bit 7 = 1) ---
  // Varre o valor analógico de 0 a 63
  for (int magnitude = 0; magnitude <= 63; magnitude++) {
    // Liga o bit 7 e combina com o valor da magnitude (bits 0-5)
    byte valorEixo = (1 << 7) | magnitude;
    enviarFrame(valorEixo);
  }

  // --- FASE 2: Mover para a Esquerda (Bit 6 = 1) ---
  // Varre o valor analógico de 0 a 63
  for (int magnitude = 0; magnitude <= 63; magnitude++) {
    // Liga o bit 6 e combina com o valor da magnitude (bits 0-5)
    byte valorEixo = (1 << 6) | magnitude;
    enviarFrame(valorEixo);
  }
}
