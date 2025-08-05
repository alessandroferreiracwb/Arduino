#include <SPI.h>
#include <WiFi.h>
#include "driver/twai.h" // Incluindo a biblioteca do driver nativo TWAI

// NOVO: Define o pino do LED e uma variável para o estado
#define LED_PIN GPIO_NUM_2
bool ledState = false; // Estado inicial do LED (desligado)

// função Velocidade da can
void reconfigureCanSpeed(long speed);

// --- Configurações da Rede Wi-Fi ---
const char* ssid = "ESP_CAN"; 
const char* password = "admin123";
const uint16_t port = 8080;

WiFiServer server(port);
WiFiClient client;

// --- Variáveis de estado do CAN Bus ---
bool canDriverIsActive = false;

void setup() {
  Serial.begin(115200);
  SPI.begin();

  // NOVO: Configura o pino do LED como saída
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW); // Garante que o LED inicie desligado

  // 1. Configurar o ESP32 como Ponto de Acesso Wi-Fi
  Serial.print("Criando Ponto de Acesso ");
  Serial.println(ssid);
  WiFi.softAP(ssid, password);
  Serial.print("IP do Servidor: ");
  Serial.println(WiFi.softAPIP());

  server.begin();
  
  // 2. Inicializar o controlador TWAI nativo do ESP32 nos pinos GPIO 5 e GPIO 4
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_5, GPIO_NUM_4, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("TWAI Driver instalado com sucesso.");
    twai_start();
    canDriverIsActive = true;
  } else {
    Serial.println("ERRO: Falha ao instalar o driver TWAI. Funcoes CAN desativadas.");
    canDriverIsActive = false;
  }
}

void loop() {
  // Verificar se há um novo cliente Wi-Fi
  if (!client.connected()) {
    client = server.available();
    if (client) {
      Serial.println("Novo cliente conectado!");
    }
  }

  // --- Lógica de Leitura e Escrita do barramento CAN real ---
  if (client.connected() && canDriverIsActive) {
    // Leitura de dados do barramento CAN real e envio para o cliente Wi-Fi
    twai_message_t rx_message;
    if (twai_receive(&rx_message, pdMS_TO_TICKS(1000)) == ESP_OK) {
        String message = String(rx_message.identifier, HEX) + " ";
        for (uint8_t i = 0; i < rx_message.data_length_code; i++) {
            if (rx_message.data[i] < 16) message += "0";
            message += String(rx_message.data[i], HEX);
        }
        client.println(message);
        Serial.print("Mensagem CAN real enviada para o cliente: ");
        Serial.println(message);
    }

    // Leitura de dados do cliente Wi-Fi e envio para o barramento CAN real
    // Leitura de dados do cliente Wi-Fi e envio para o barramento CAN real
    if (client.available()) {
      String data = client.readStringUntil('\n');
      data.trim();

      Serial.print("Mensagem recebida via WiFi: ");
      Serial.println(data);

      
      
      // NOVO: Verifica se o comando é para o LED
      if (data.startsWith("LED_D2")) {
        if (data.indexOf("ON") != -1) {
          digitalWrite(LED_PIN, HIGH); // Liga o LED
          ledState = true;
          Serial.println("Comando para LED ON recebido. LED ligado.");
        } else if (data.indexOf("OFF") != -1) {
          digitalWrite(LED_PIN, LOW); // Desliga o LED
          ledState = false;
          Serial.println("Comando para LED OFF recebido. LED desligado.");
        }
      } // NOVO: Verifica se o comando é para mudar a velocidade do CAN
        if (data.startsWith("CAN_SPEED")) {
            long newSpeed = data.substring(data.indexOf(' ') + 1).toInt();
            reconfigureCanSpeed(newSpeed);
        }
        // Se não for um comando para o LED ou CAN_SPEED, processa como um frame CAN
        
      // Se não for um comando para o LED, processa como um frame CAN
      else {
        int spaceIndex = data.indexOf(' ');
        if (spaceIndex != -1) {
          String idString = data.substring(0, spaceIndex);
          String frameString = data.substring(spaceIndex + 1);
          
          unsigned long id = strtoul(idString.c_str(), NULL, 16);
          unsigned char dlc = frameString.length() / 2;
          unsigned char buf[8];
          
          for (int i = 0; i < dlc; i++) {
            String byteString = frameString.substring(i * 2, i * 2 + 2);
            buf[i] = strtoul(byteString.c_str(), NULL, 16);
          }
          
          twai_message_t canMsgToSend;
          canMsgToSend.identifier = id;
          canMsgToSend.extd = (idString.length() > 3);
          canMsgToSend.data_length_code = dlc;
          memcpy(canMsgToSend.data, buf, dlc);
          
          if (twai_transmit(&canMsgToSend, pdMS_TO_TICKS(1000)) == ESP_OK) {
            Serial.println("Mensagem enviada para o CAN Bus.");
          } else {
            Serial.println("ERRO: Falha ao enviar a mensagem para o CAN Bus.");
          }
        }
      }
    }
  } else if (client.connected() && !canDriverIsActive) {
      Serial.println("CAN Bus inativo. Nao e possivel ler ou enviar mensagens.");
  }
}

// Implementação da nova função
void reconfigureCanSpeed(long speed) {
    // Para reconfigurar, é necessário desinstalar e reinstalar o driver
    if (canDriverIsActive) {
        twai_stop();
        twai_driver_uninstall();
        canDriverIsActive = false;
        Serial.println("TWAI Driver parado e desinstalado para reconfiguracao.");
    }

    twai_timing_config_t t_config;
    if (speed == 125000) {
        t_config = TWAI_TIMING_CONFIG_125KBITS();
    } else if (speed == 500000) {
        t_config = TWAI_TIMING_CONFIG_500KBITS();
    } else { // Padrão para 250k se nenhum for correspondido
        t_config = TWAI_TIMING_CONFIG_250KBITS();
    }

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_5, GPIO_NUM_4, TWAI_MODE_NORMAL);
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        twai_start();
        canDriverIsActive = true;
        Serial.print("TWAI Driver reconfigurado com sucesso para ");
        Serial.print(speed);
        Serial.println(" bps.");
    } else {
        Serial.println("ERRO: Falha na reconfiguracao do driver TWAI.");
    }
}
// fim
