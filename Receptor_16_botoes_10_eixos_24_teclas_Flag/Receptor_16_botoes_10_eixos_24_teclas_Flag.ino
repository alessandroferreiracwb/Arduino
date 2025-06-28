// ==== CÓDIGO FINAL DO RECEPTOR (2 JOYSTICKS + EIXOS NOVOS) ====
// Este código recebe um pacote de dados de 24 bytes, valida a sua integridade e
// usa os dados para controlar dois joysticks virtuais distintos que se apresentam
// ao computador através de um único cabo USB.

#include <stdint.h>      // Biblioteca para tipos de dados de tamanho fixo (ex: uint16_t).
#include <Joystick.h>    // Biblioteca para emular um ou mais joysticks USB.

// --- Definições do Protocolo e do Frame ---
// Estas constantes devem ser exatamente as mesmas definidas no Transmissor.
const int FRAME_SIZE = 24;
const int DATA_PAYLOAD_LEN = 22;
const byte EXPECTED_INIT_FRAME = 0xF1;
const byte EXPECTED_END_FRAME = 0xE7;

// --- Variáveis Globais ---
byte receivedFrame[FRAME_SIZE];     // Array para armazenar o frame bruto que chega pela serial.
byte payload[DATA_PAYLOAD_LEN];     // Array para armazenar a porção de dados do frame (sem o CRC) para validação.

// --- Criação dos Dispositivos Joystick Virtuais ---
// O segredo para criar dois dispositivos é usar um "Report ID" diferente para cada um.

// --- Objeto Joystick 1 (O principal, com eixos e 16 botões) ---
// Usamos o Report ID padrão (JOYSTICK_DEFAULT_REPORT_ID).
// Configuramos como um GAMEPAD com 16 botões e 10 eixos analógicos.
Joystick_ joystick1(
  JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_GAMEPAD,
  16, 0, true, true, true, true, true, true, true, true, true, true, false);

// --- Objeto Joystick 2 (O teclado de membrana, com 24 botões e sem eixos) ---
// Usamos o Report ID seguinte (JOYSTICK_DEFAULT_REPORT_ID + 1) para o diferenciar do primeiro.
// Configuramos como um GAMEPAD com 24 botões e todos os eixos desabilitados.
Joystick_ joystick2(
  JOYSTICK_DEFAULT_REPORT_ID + 1, JOYSTICK_TYPE_GAMEPAD,
  24, 0, false, false, false, false, false, false, false, false, false, false, false);


// --- Função de Cálculo CRC-16 ---
// É usada para calcular o CRC dos dados recebidos
// e comparar com o CRC que veio no frame, garantindo a integridade dos dados.
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
  Serial.begin(9600);       // Inicia a porta serial USB para mensagens de debug no Monitor Serial.
  Serial1.begin(9600);      // Inicia a porta serial de hardware para receber dados do Transmissor.

  // --- Configuração dos Eixos do Joystick 1 (com faixa simétrica) ---
  // O eixo agora vai de um número negativo até um positivo, com o centro em 0.
  // Isso é essencial para o novo formato de dados dos eixos.
  joystick1.setXAxisRange(-512, 512);
  joystick1.setYAxisRange(-512, 512);
  joystick1.setZAxisRange(-512, 512);
  joystick1.setRxAxisRange(-512, 512);
  joystick1.setRyAxisRange(-512, 512);
  joystick1.setRzAxisRange(-512, 512);
  joystick1.setRudderRange(-512, 512);
  joystick1.setThrottleRange(-512, 512);
  joystick1.setAcceleratorRange(-512, 512);
  joystick1.setBrakeRange(-512, 512);
  
  // Inicia ambos os joysticks para que o computador os reconheça.
  // O parâmetro 'false' impede o envio de um estado inicial vazio.
  joystick1.begin(false);
  joystick2.begin(false);
  
  Serial.println("Receptor Final com 2 Joysticks e Eixos Novos pronto.");
}

// --- Loop Principal ---
void loop() {
  // Verifica se um pacote completo de 24 bytes já chegou na porta serial.
  if (Serial1.available() >= FRAME_SIZE) {
    Serial1.readBytes(receivedFrame, FRAME_SIZE);

    // ETAPA 1: Organizar e Validar o Frame
    // Copia os 22 bytes de dados (sem o CRC) para o array 'payload'.
    for (int i = 0; i < DATA_PAYLOAD_LEN; i++) {
      payload[i] = receivedFrame[i];
    }
    // Reconstrói o CRC de 16 bits que veio nos dois últimos bytes do frame.
    uint16_t receivedCrc = (receivedFrame[22] << 8) | receivedFrame[23];
    // Calcula o nosso próprio CRC sobre o payload recebido.
    uint16_t calculatedCrc = crc16(payload, DATA_PAYLOAD_LEN);

    // Validação tripla: se o início, o fim ou o CRC não baterem, o frame é inválido.
    if (payload[0] != EXPECTED_INIT_FRAME || payload[21] != EXPECTED_END_FRAME || calculatedCrc != receivedCrc) {
      Serial.println("ERRO: Frame invalido!");
      return; // Aborta o processamento deste frame e espera pelo próximo.
    }

    // --- ETAPA 2: SUCESSO! ATUALIZAR JOYSTICK 1 (EIXOS + 16 BOTÕES) ---
    
    // Decodificar e atualizar os 10 eixos
    int finalAxisValues[10];
    for (int i = 0; i < 10; i++) {
      byte rawAxisByte = payload[i + 3]; // Pega o byte do eixo correspondente (3 a 12).
      
      // Extrai a magnitude e as flags de direção usando máscaras e operações de bit.
      int magnitude = rawAxisByte & 0x3F;      // Pega os 6 primeiros bits (0-63).
      bool goLeft   = bitRead(rawAxisByte, 6); // Checa o estado do bit 6.
      bool goRight  = bitRead(rawAxisByte, 7); // Checa o estado do bit 7.
      
      // Converte a magnitude (0-63) para a nossa faixa de eixo (0-512).
      int mappedMagnitude = map(magnitude, 0, 63, 0, 512);
      
      // Determina o valor final do eixo com base nas flags de direção.
      int finalValue = 0; // Padrão é o centro (0).
      if (goRight && !goLeft) {
        finalValue = mappedMagnitude; // Valor positivo.
      } else if (goLeft && !goRight) {
        finalValue = -mappedMagnitude; // Valor negativo.
      }
      finalAxisValues[i] = finalValue;
    }
    // Envia os valores finais para os eixos do joystick 1.
    joystick1.setXAxis(finalAxisValues[0]); joystick1.setYAxis(finalAxisValues[1]); joystick1.setZAxis(finalAxisValues[2]);
    joystick1.setRxAxis(finalAxisValues[3]); joystick1.setRyAxis(finalAxisValues[4]); joystick1.setRzAxis(finalAxisValues[5]);
    joystick1.setRudder(finalAxisValues[6]); joystick1.setThrottle(finalAxisValues[7]); joystick1.setAccelerator(finalAxisValues[8]); 
    joystick1.setBrake(finalAxisValues[9]);

    // Atualizar os 16 botões do Joystick 1 (lendo os bytes 13 e 14).
    for (int i = 0; i < 8; i++) { if (bitRead(payload[13], i) == 1) { joystick1.pressButton(i); } else { joystick1.releaseButton(i); } }
    for (int i = 0; i < 8; i++) { if (bitRead(payload[14], i) == 1) { joystick1.pressButton(i + 8); } else { joystick1.releaseButton(i + 8); } }

    // --- ETAPA 3: ATUALIZAR JOYSTICK 2 (24 TECLAS DE MEMBRANA) ---
    // Atualizar as 24 teclas (lendo os bytes 15, 16 e 17).
    for (int i = 0; i < 8; i++) { if (bitRead(payload[15], i) == 1) { joystick2.pressButton(i); } else { joystick2.releaseButton(i); } }
    for (int i = 0; i < 8; i++) { if (bitRead(payload[16], i) == 1) { joystick2.pressButton(i + 8); } else { joystick2.releaseButton(i + 8); } }
    for (int i = 0; i < 8; i++) { if (bitRead(payload[17], i) == 1) { joystick2.pressButton(i + 16); } else { joystick2.releaseButton(i + 16); } }
    
    // --- ETAPA 4: ENVIAR ESTADO DE AMBOS OS JOYSTICKS ---
    // Envia o estado atualizado de cada joystick para o computador.
    joystick1.sendState();
    joystick2.sendState();

    Serial.println("Frame OK. Joysticks 1 e 2 atualizados.");
  }
}
