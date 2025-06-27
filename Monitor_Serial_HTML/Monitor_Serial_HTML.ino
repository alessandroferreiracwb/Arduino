void setup() {
  // Inicia a comunicação com o computador (via USB)
  Serial.begin(9600); 

  // Inicia a comunicação com o outro dispositivo UART (nos pinos 0 e 1)
  // IMPORTANTE: A velocidade (9600) deve ser a mesma do seu outro dispositivo!
  Serial1.begin(9600);

  // Espera a porta serial USB ser aberta pelo computador
  while (!Serial) {
    ; // Aguarda a conexão
  }
  
  Serial.println("Modo Ponte Ativado. Aguardando dados...");
}

void loop() {
  // Verifica se há dados chegando do dispositivo UART (via Serial1)
  if (Serial1.available()) {
    // Lê um byte do dispositivo
    char dataFromDevice = Serial1.read();
    // Envia o mesmo byte para o computador/browser (via Serial)
    Serial.write(dataFromDevice);
  }

  // Verifica se há dados chegando do computador/browser (via Serial)
  if (Serial.available()) {
    // Lê um byte do browser
    char dataFromBrowser = Serial.read();
    // Envia o mesmo byte para o dispositivo UART (via Serial1)
    Serial1.write(dataFromBrowser);
  }
}
