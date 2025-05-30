#define TXD2 17
#define RXD2 16  // Mesmo que não vá usar o RX aqui, precisa definir

void setup() {
  // Inicializa Serial2 no mesmo baud rate usado no receptor
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  // Também inicializa a Serial padrão para debug (opcional)
  Serial.begin(9600);
  Serial.println("Transmissor iniciado. Enviando dados na UART2...");
}

void loop() {
  // Envia uma mensagem a cada 1 segundo
  Serial2.println("AA:BB:CC:DD:FF:24:58:AA:BB:CC:DD:FF:24:58:AA:BB:CC:DD:FF:24:58");
  Serial.println("Mensagem enviada pela UART2.");
  delay(100);
}
