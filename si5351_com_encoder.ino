#include <Wire.h>
#include <si5351.h>

// ----------- Configuração de Hardware (Arduino Micro) -----------
// Encoder 0 (Controla CLK0) - Evitando pinos 0, 1, 2 e 3
const int pinCLK0 = 4;
const int pinDT0  = 5;
const int pinSW0  = 6;

// Encoder 1 (Controla CLK1)
const int pinCLK1 = 7;
const int pinDT1  = 8;
const int pinSW1  = 9;

// Encoder 2 (Controla CLK2)
const int pinCLK2 = 10;
const int pinDT2  = 11;
const int pinSW2  = 12;

// ----------- Variáveis de Controle -----------
unsigned long freq[3] = {1000000, 1000000, 1000000}; 
const unsigned long steps[] = {100, 1000, 10000, 100000, 1000000};
int stepIndex[3] = {1, 1, 1}; 
int status_si5351 = 0;

int lastCLK[3];
bool btnState[3] = {false, false, false};
unsigned long btnTime[3] = {0, 0, 0};

Si5351 si5351;

// ----------- Função de Comunicação com o ESP32 -----------
void enviarDadosDisplay() {
  // Envia um pacote formatado: [freq0,step0,freq1,step1,freq2,step2]
  Serial1.print("[");
  for(int i = 0; i < 3; i++) {
    Serial1.print(freq[i]);
    Serial1.print(",");
    Serial1.print(steps[stepIndex[i]]);
    if(i < 2) Serial1.print(",");
  }
  Serial1.println("]"); 
}

void updateSi5351(int channel, unsigned long frequency) {
  si5351.set_freq((uint64_t)frequency * 100ULL, (si5351_clock)channel);
  enviarDadosDisplay(); // Sempre que atualizar a frequência, avisa o display
}

void setup() {
  // Serial USB para Debug no PC
  Serial.begin(115200);
  
  // Serial1 (Pinos 0 e 1) para o ESP32 (CYD)
  Serial1.begin(115200);

  // Inicializa Pinos dos Encoders
  int pins[] = {pinCLK0, pinDT0, pinSW0, pinCLK1, pinDT1, pinSW1, pinCLK2, pinDT2, pinSW2};
  for(int i=0; i<9; i++) pinMode(pins[i], INPUT_PULLUP);

  // Inicializa Si5351 (Pinos 2 e 3 do Micro)
  bool i2c_found = si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  if(!i2c_found) {
    Serial.println("Erro: Si5351 nao encontrado!");
    status_si5351 = 1;
  }

  // Estado inicial
  for (int i = 0; i < 3; i++) {
    lastCLK[i] = digitalRead(pinCLK0 + (i * 3)); 
    updateSi5351(i, freq[i]);
  }

  enviarDadosDisplay(); // Envia estado inicial para o display
}

void loop() {
  handleEncoder(0, pinCLK0, pinDT0, pinSW0);
  handleEncoder(1, pinCLK1, pinDT1, pinSW1);
  handleEncoder(2, pinCLK2, pinDT2, pinSW2);
  
  // Verifica status do Si5351   
  if(!status_si5351) {
    Serial.println("Info: Si5351 encontrado!");
  }else{
    Serial.println("Erro: Si5351 nao encontrado!");
    delay(1000);
  }
}

void handleEncoder(int ch, int clkPin, int dtPin, int swPin) {
  // --- Botão (Troca de Passo) ---
  bool reading = digitalRead(swPin);
  if (reading == LOW && !btnState[ch]) {
    btnState[ch] = true;
    btnTime[ch] = millis();
  } else if (reading == HIGH && btnState[ch]) {
    if (millis() - btnTime[ch] > 50) { 
      stepIndex[ch] = (stepIndex[ch] + 1) % (sizeof(steps) / sizeof(steps[0]));
      enviarDadosDisplay(); // Avisa o display sobre a mudança no Step
    }
    btnState[ch] = false;
  }

  // --- Rotação ---
  int clkCur = digitalRead(clkPin);
  if (clkCur != lastCLK[ch]) {
    if (digitalRead(dtPin) != clkCur) {
      freq[ch] += steps[stepIndex[ch]];
    } else {
      if (freq[ch] >= steps[stepIndex[ch]]) freq[ch] -= steps[stepIndex[ch]];
    }

    if (freq[ch] > 160000000UL) freq[ch] = 160000000UL;

    updateSi5351(ch, freq[ch]); // Já chama o envio de dados
    delay(2); 
  }
  lastCLK[ch] = clkCur;
}
