#define RXD2 16
#define TXD2 17  // Obrigatório definir, mesmo que não use TX

void setup() {
  Serial.begin(115200);  // Monitor Serial da IDE
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);  // UART2

  Serial.println("Receptor UART2 pronto. Aguardando dados...");
}

void loop() {
  // Lê e imprime os dados recebidos na UART2
  while (Serial2.available()) {
    char c = Serial2.read();
    Serial.print(c);  // Imprime na IDE
  }
}
