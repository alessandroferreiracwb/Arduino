#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

// Substitua com suas credenciais de Wi-Fi
const char* ssid = "ALESSANDRO";        // Nome da sua rede Wi-Fi
const char* password = "98291490";   // Senha da sua rede Wi-Fi

// Cria um objeto WebSocketsServer na porta 81 (porta padrão para WebSockets)
WebSocketsServer webSocket = WebSocketsServer(81);

// --- Função para calcular CRC16 (CRC-CCITT XModem) ---
unsigned int calculateCRC16(const byte *data, size_t length) {
  unsigned int crc = 0x0000; // Initial value for CRC-CCITT (XModem)
  for (size_t i = 0; i < length; i++) {
    crc ^= (unsigned int)(data[i] << 8);
    for (int j = 0; j < 8; j++) {
      if ((crc & 0x8000) != 0) {
        crc = (crc << 1) ^ 0x1021; // Polynomial for CRC-CCITT
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

// --- Nova Função para imprimir um byte em binário ---
void printByteAsBinary(byte b) {
  for (int i = 7; i >= 0; i--) { // Começa do bit mais significativo (MSB)
    Serial.print((b >> i) & 0x01 ? '1' : '0');
  }
  Serial.print(" "); // Adiciona um espaço para separar os bytes
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      break;
    case WStype_CONNECTED: {
      webSocket.sendTXT(num, "Conectado ao ESP32 via WebSocket!");
    }
      break;
    case WStype_TEXT:
      // --- Início da montagem do Frame UART ---
      byte frame[24];
      memset(frame, 0, sizeof(frame)); // Inicializa o frame com zeros

      // Byte 0: Cabeçalho 1
      frame[0] = 0xF1;
      // Byte 1: Cabeçalho 2
      frame[1] = 0x0E;
      // Byte 2: Cabeçalho 3
      frame[2] = 0xD1;

      // Tenta fazer o parsing do JSON recebido
      {
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, payload);

        if (error) {
          return;
        }

        // --- Mapeamento dos Eixos (Bytes 3 ao 15) ---
        for (int i = 0; i < 8; i++) {
          String eixoName = "eixo" + String(i + 1);
          if (doc["joystickPrincipal"]["eixos"].containsKey(eixoName)) {
            float eixoX = doc["joystickPrincipal"]["eixos"][eixoName]["x"].as<float>();
            frame[3 + i] = (byte)((eixoX + 100.0) / 200.0 * 255.0);
          }
        }

        for (int i = 0; i < 5; i++) {
          String eixoName = "eixo" + String(i + 1);
          if (doc["joystick2"]["eixos"].containsKey(eixoName)) {
            float eixoX = doc["joystick2"]["eixos"][eixoName]["x"].as<float>();
            frame[11 + i] = (byte)((eixoX + 100.0) / 200.0 * 255.0);
          }
        }

        // --- Mapeamento dos Botões (Bytes 16 e 17) ---
        byte jp_buttons_byte1 = 0;
        byte jp_buttons_byte2 = 0;

        if (doc["joystickPrincipal"].containsKey("botoes")) {
          for (int i = 0; i < 12; i++) {
            String btnName = "btn" + String(i + 1);
            if (doc["joystickPrincipal"]["botoes"].containsKey(btnName)) {
              bool is_active = doc["joystickPrincipal"]["botoes"][btnName];
              if (is_active) {
                if (i < 6) {
                  jp_buttons_byte1 |= (1 << i);
                } else {
                  jp_buttons_byte2 |= (1 << (i - 6));
                }
              }
            }
          }
        }
        frame[16] = jp_buttons_byte1;
        frame[17] = jp_buttons_byte2;

        // --- Mapeamento das Teclas (Bytes 18, 19 e 20) ---
        byte j2_keys_byte1 = 0;
        byte j2_keys_byte2 = 0;
        byte j2_keys_byte3 = 0;

        if (doc["joystick2"].containsKey("botoes")) {
          for (int i = 0; i < 24; i++) {
            String btnName = "btn" + String(i + 1);
            if (doc["joystick2"]["botoes"].containsKey(btnName)) {
              bool is_active = doc["joystick2"]["botoes"][btnName];
              if (is_active) {
                if (i < 8) {
                  j2_keys_byte1 |= (1 << i);
                } else if (i < 16) {
                  j2_keys_byte2 |= (1 << (i - 8));
                } else {
                  j2_keys_byte3 |= (1 << (i - 16));
                }
              }
            }
          }
        }
        frame[18] = j2_keys_byte1;
        frame[19] = j2_keys_byte2;
        frame[20] = j2_keys_byte3;

        // Byte 21: Final do cabeçalho
        frame[21] = 0xE7;

        // --- Cálculo e Inserção do CRC16 (Bytes 22 e 23) ---
        unsigned int crc = calculateCRC16(frame, 22);
        frame[22] = (byte)(crc >> 8);
        frame[23] = (byte)(crc & 0xFF);

        // --- Impressão Bruta do Frame UART em BINÁRIO no Monitor Serial ---
        for (int i = 0; i < 24; i++) {
          printByteAsBinary(frame[i]); // Chama a nova função para imprimir em binário
        }
        Serial.println(); // Adiciona uma nova linha após cada frame completo

      } // Fim do bloco de parsing JSON
      break;
    case WStype_BIN:
    case WStype_ERROR:
    case WStype_FRAGMENT_TEXT_START:
    // AQUI: Corrija 'WStype_BIN_START' para 'WStype_FRAGMENT_BIN_START'
    case WStype_FRAGMENT_BIN_START: // Linha corrigida
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:
      break;
  }
}

void setup() {
  // Configura UART0 para 115200 bps.
  Serial.begin(115200);

  // Mensagens iniciais de conexão Wi-Fi
  Serial.print("Conectando-se ao Wi-Fi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("Wi-Fi conectado!");
  Serial.print("Endereço IP do ESP32: ");
  Serial.println(WiFi.localIP());
  Serial.println("Servidor WebSocket iniciado na porta 81");

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

void loop() {
  webSocket.loop();
}
