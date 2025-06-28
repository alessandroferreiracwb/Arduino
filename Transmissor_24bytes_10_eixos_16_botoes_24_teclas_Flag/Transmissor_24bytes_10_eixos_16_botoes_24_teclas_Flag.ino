// ==== CÓDIGO FINAL DO TRANSMISSOR (FORMATADO E COMENTADO) ====
// Este código é responsável por gerar e enviar um pacote de dados (frame) de 24 bytes
// que simula o estado de dois joysticks para o Arduino Receptor.

// Inclusão de bibliotecas para tipos de dados de tamanho fixo.
#include <stdint.h>

// --- Definições do Protocolo e do Frame ---
// Constantes que definem a estrutura do nosso pacote de dados.
const int FRAME_SIZE = 24;          // Tamanho total do pacote de dados em bytes.
const int DATA_PAYLOAD_LEN = 22;    // Tamanho dos dados usados para calcular o CRC (bytes 0-21).
const byte Init_Frame = 0xF1;       // Byte mágico que marca o início de um frame válido.
const byte Cod_end1   = 0x0E;       // Parte 1 de um código de identificação/endereço.
const byte Cod_end2   = 0xD1;       // Parte 2 de um código de identificação/endereço.
const byte End_Frame  = 0xE7;       // Byte mágico que marca o fim do payload de dados.

// --- Controle de Dados e Tempo ---
const int DELAY_POR_PASSO = 100;    // Delay em milissegundos entre o envio de cada frame, para controlar a velocidade da simulação.

// --- Variáveis Globais ---
byte dataFrame[FRAME_SIZE];         // Array de bytes que armazena o pacote de dados completo a ser enviado.
int botaoAtivo = 0;                 // Contador para simular qual botão do joystick 1 (0-15) está ativo.
int teclaAtivo = 0;                 // Contador para simular qual tecla do joystick 2 (0-23) está ativa.

// --- Função de Cálculo CRC-16 ---
// Calcula um checksum de 16 bits (CRC-16-CCITT/KERMIT) sobre um bloco de dados.
// Isso é usado pelo receptor para verificar se os dados chegaram sem corrupção.
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
  // Inicia a porta serial de hardware (Serial1) nos pinos 0(RX) e 1(TX) para comunicar com o Receptor.
  Serial1.begin(9600);
}

// --- Loop Principal ---
// O loop principal executa uma simulação contínua, movendo os eixos de um lado para o outro.
void loop() {
  // --- FASE 1: Mover para a Direita (Bit 7 = 1) ---
  // Varre a "magnitude" do movimento analógico de 0 a 63.
  for (int magnitude = 0; magnitude <= 63; magnitude++) {
    // Constrói o byte do eixo:
    // (1 << 7) cria um byte com apenas o bit 7 ligado (10000000).
    // O operador '|' (OU bit-a-bit) combina a flag de direção com a magnitude (bits 0-5).
    byte valorEixo = (1 << 7) | magnitude;
    enviarFrame(valorEixo);
  }

  // --- FASE 2: Mover para a Esquerda (Bit 6 = 1) ---
  // Varre a "magnitude" do movimento analógico de 0 a 63.
  for (int magnitude = 0; magnitude <= 63; magnitude++) {
    // Constrói o byte do eixo:
    // (1 << 6) cria um byte com apenas o bit 6 ligado (01000000).
    // O operador '|' combina a flag de direção com a magnitude.
    byte valorEixo = (1 << 6) | magnitude;
    enviarFrame(valorEixo);
  }
}

// --- Função Auxiliar de Envio ---
// Esta função monta e envia um frame completo a cada chamada.
void enviarFrame(byte valorEixo) {

  // ETAPA 1: Montagem do Cabeçalho (Bytes 0, 1, 2)
  dataFrame[0] = Init_Frame;
  dataFrame[1] = Cod_end1;
  dataFrame[2] = Cod_end2;

  // ETAPA 2: Dados dos Eixos (Bytes 3 a 12)
  // Aplica o mesmo valor de eixo codificado para todos os 10 eixos.
  for (int j = 3; j <= 12; j++) {
    dataFrame[j] = valorEixo;
  }

  // ETAPA 3: Dados dos Botões do Joystick 1 (Bytes 13 e 14)
  // Simula o pressionamento de um botão por vez, entre 16 botões.
  byte byteBotoes1_8 = 0;
  byte byteBotoes9_16 = 0;
  if (botaoAtivo < 8) {
    byteBotoes1_8 = bit(botaoAtivo); // Se o botão for 0-7, usa o primeiro byte.
  } else {
    byteBotoes9_16 = bit(botaoAtivo - 8); // Se for 8-15, usa o segundo byte.
  }
  dataFrame[13] = byteBotoes1_8;
  dataFrame[14] = byteBotoes9_16;

  // ETAPA 4: Dados das Teclas do Joystick 2 (Bytes 15, 16, 17)
  // Simula o pressionamento de uma tecla por vez, entre 24 teclas.
  byte byteTeclas1_8 = 0;
  byte byteTeclas9_16 = 0;
  byte byteTeclas17_24 = 0;
  if (teclaAtivo < 8) {
    byteTeclas1_8 = bit(teclaAtivo); // Teclas 0-7 no primeiro byte.
  } else if (teclaAtivo < 16) {
    byteTeclas9_16 = bit(teclaAtivo - 8); // Teclas 8-15 no segundo byte.
  } else {
    byteTeclas17_24 = bit(teclaAtivo - 16); // Teclas 16-23 no terceiro byte.
  }
  dataFrame[15] = byteTeclas1_8;
  dataFrame[16] = byteTeclas9_16;
  dataFrame[17] = byteTeclas17_24;

  // ETAPA 5: Preenchimento e Marcador de Fim (Bytes 18 a 21)
  for (int j = 18; j <= 20; j++) {
    dataFrame[j] = 0x00; // Zera bytes não utilizados para manter o CRC consistente.
  }
  dataFrame[21] = End_Frame;

  // ETAPA 6: Cálculo e Inserção do CRC (Bytes 22 e 23)
  uint16_t crcResult = crc16(dataFrame, DATA_PAYLOAD_LEN);
  dataFrame[22] = (crcResult >> 8) & 0xFF; // Adiciona o byte mais significativo do CRC.
  dataFrame[23] = crcResult & 0xFF;        // Adiciona o byte menos significativo do CRC.

  // ETAPA 7: Envio e Atualização dos Contadores
  Serial1.write(dataFrame, FRAME_SIZE);
  botaoAtivo = (botaoAtivo + 1) % 16;   // Incrementa o contador de botões e volta a 0 após 15.
  teclaAtivo = (teclaAtivo + 1) % 24;   // Incrementa o contador de teclas e volta a 0 após 23.
  delay(DELAY_POR_PASSO);
}
