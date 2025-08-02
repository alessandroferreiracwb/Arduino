/*
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Modificado para incluir um modo de configuração Wi-Fi com seleção de SSID na tela
  e TECLADO ALFANUMÉRICO VIRTUAL para a senha/entrada de texto.
  Funcionalidades do Teclado Aprimoradas:
  - Teclado com alfabeto completo (minúsculas, maiúsculas)
  - Botão SHIFT para alternar entre maiúsculas/minúsculas.
  - Botão MODE para alternar entre modo de LETRAS e modo de NÚMEROS/SÍMBOLOS.
  - Backspace e Conectar (GO!) mantidos.
  A função de calibração do touchscreen foi removida para simplificar o código.
  Agora com SSIDs maiores na tela de seleção de rede, e botão "Config WiFi" mais largo.
  Correção do erro "jump to case label".
  Os botões do teclado numérico foram diminuídos ainda mais.
  O botão MENU na tela principal foi movido para o canto superior direito.
  A cor do texto do IP e SSID na tela inicial foi alterada para TFT_DARKGREEN.
  Corrigida a lógica de rolagem na lista de redes Wi-Fi e seleção de IDs.
  Exibição das setas de rolagem na lista de IDs.
  A altura do retângulo de exibição da senha foi aumentada.
  CORREÇÃO FINAL: Alinhamento do ponto de toque da seta de rolagem para cima.
  Adicionado sensor DHT11 no GPIO 22 para leituras de temperatura e umidade reais.
  AJUSTE: IP e SSID agora aparecem na mesma linha na tela principal.
  CORREÇÃO CRÍTICA: Ajuste na ordem de desenho da drawMainScreen para garantir visibilidade da T/U.
  Este código funciona para o ESP32-2432S028.
  NOVO: Adicionado sincronização de data e hora via NTP.
  CORREÇÃO: Uso de funções nativas para formatar o data, resolvendo o erro 'getFormattedDate()'.
  AJUSTE: Data e Hora agora exibidas no CANTO SUPERIOR ESQUERDO da tela principal.
  NOVO: Tela de configuração MQTT aprimorada com exibição de caminho e chave API,
        e botões "Edit" para abrir o teclado de entrada.
  APERFEIÇOAMENTO: Conexão MQTT não bloqueante e buffer de dados offline.
*/

// Bibliotecas padrão e do framework
#include <SPI.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include <WiFi.h>
#include <EEPROM.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <time.h> // Para gmtime

// Inclusão dos nossos arquivos modularizados
#include "globals.h"        // Contém TODAS as declarações extern e defines
#include "screens/screens.h"
#include "keypad/keypad.h"
#include "wifi_manager/wifi_manager.h"
#include "dht_sensor/dht_sensor.h"
#include "mqtt_config/mqtt_config.h"
#include "mqtt_client/mqtt_client.h"

// Variável para controlar o tempo da última leitura/publicação do sensor
unsigned long lastSensorPublishTime = 0;
const long sensorPublishInterval = 5000; // Intervalo de 5 segundos para leitura/publicação

// Variáveis para gerenciar o timeout da tela de conexão Wi-Fi
unsigned long wifiConnectingStartTime = 0;
const long WIFI_CONNECTION_TIMEOUT_MS = 20000; // 20 segundos para tentar conectar Wi-Fi

// Variável para controlar o tempo da última tentativa de reconexão MQTT
unsigned long lastMqttReconnectAttempt = 0;

// --- Setup: Executado uma única vez ao ligar ou reiniciar ---
void setup() {
  Serial.begin(115200);

  // Inicializa EEPROM
  if (!EEPROM.begin(EEPROM_SIZE_UPDATED)) {
    Serial.println("Falha ao inicializar EEPROM. Verifique o tamanho.");
    while (true);
  }

  // Inicializa o sensor DHT
  dht.begin();
  Serial.println("Sensor DHT inicializado.");

  // Inicializa TFT e Touchscreen
  touchscreenSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
  touchscreen.begin(touchscreenSPI);
  touchscreen.setRotation(1);

  tft.init();
  tft.setRotation(1);

  // Inicializa as definições dos botões do teclado alfanumérico
  initAlphaNumericKeys();

  // Tenta carregar credenciais salvas e configurar MQTT
  loadCredentials(); // Carrega credenciais WiFi
  loadMqttPath();    // Carrega caminho MQTT
  loadApiKey();      // Carrega Chave API
  loadMqttUsername(); // Carrega usuário MQTT
  loadMqttPassword(); // Carrega senha MQTT

  Serial.print("SETUP: Usuario MQTT carregado na RAM (storedMqttUsername): ["); Serial.print(storedMqttUsername); Serial.println("]");
  Serial.print("SETUP: Senha MQTT carregada na RAM (storedMqttPassword): ["); Serial.print(storedMqttPassword); Serial.println("]");


  // Gera um ID de cliente MQTT único (se ainda não tiver um ou for vazio)
  if (strlen(mqttClientId) == 0 || strcmp(mqttClientId, "ESP32Client") == 0) {
    uint64_t chipid = ESP.getEfuseMac();
    sprintf(mqttClientId, "ESP32-%04X%08X", (uint16_t)(chipid >> 32), (uint32_t)chipid);
    Serial.print("Gerado MQTT Client ID: ");
    Serial.println(mqttClientId);
  }
  
  // Configura Wi-Fi
  WiFi.mode(WIFI_STA); // Define o modo Wi-Fi como estação (cliente)

  // Configura NTP (iniciado após conexão Wi-Fi)
  timeClient.begin();
  timeClient.setTimeOffset(utcOffsetInSeconds);

  // Primeira leitura do sensor para popular o display
  readDHTSensor();
  
  // Desenha a tela principal para começar
  drawMainScreen();
}

void loop() {
  unsigned long currentMillis = millis();

  // --- Processos em Segundo Plano (APENAS na Tela Principal) ---
  if (currentState == STATE_MAIN_SCREEN) {
      // --- Manutenção de Conexão Wi-Fi ---
      if (WiFi.status() != WL_CONNECTED) {
        static unsigned long lastWifiAttempt = 0;
        const long wifiRetryInterval = 10000; // Tenta a cada 10 segundos
        
        // Tenta conectar Wi-Fi APENAS se houver SSID salvo
        if (strlen(storedSsid) > 0 || (selectedSsid.length() > 0 && strlen(enteredPassword.c_str()) > 0)) {
          if (currentMillis - lastWifiAttempt > wifiRetryInterval) {
            Serial.println("Wi-Fi desconectado. Tentando reconectar...");
            currentWifiStatus = WIFI_CONNECTING; // Define status para 'Conectando'
            wifiConnectingStartTime = currentMillis; // Inicia o timer da tela de conexão
            connectToWifi(); // Inicia a tentativa de conexão (função agora é TOTALMENTE não bloqueante)
            lastWifiAttempt = currentMillis;
          }
        } else {
          // Serial.println("Nenhum SSID salvo. Configure o Wi-Fi via IHM."); // Mensagem de depuração (comentada)
        }
      } else { // Wi-Fi está conectado
        // Se o status ainda é WIFI_CONNECTING mas Wi-Fi já conectou, atualiza para OK
        if (currentWifiStatus == WIFI_CONNECTING || currentWifiStatus == WIFI_CONNECTION_FAILED) {
            currentWifiStatus = WIFI_CONNECTED_OK;
            Serial.println("Wi-Fi conectado! IP: " + WiFi.localIP().toString());
        }

        // Mantém o cliente MQTT ativo e tenta reconectar MQTT
        mqttClient.loop();
        if (strlen(mqttServerPath) > 0) { // Apenas se um caminho MQTT estiver configurado
            reconnectMqtt();
        }
      }

      // --- Leitura do Sensor DHT e Publicação MQTT (Não Bloqueante) ---
      if (currentMillis - lastSensorPublishTime >= sensorPublishInterval) {
        lastSensorPublishTime = currentMillis; // Atualiza o tempo da última leitura/publicação

        readDHTSensor(); // Lê o sensor (função em dht_sensor.cpp)

        // Se MQTT estiver configurado e conectado, tenta publicar.
        // A função publishSensorData já cuida de buffering se estiver offline.
        if (strlen(mqttServerPath) > 0) {
            publishSensorData(temperature, humidity);
        }
        
        // Sempre redesenha a tela principal após a leitura do sensor
        // para atualizar os valores ou o status 'XX'.
        drawMainScreen();
      }
  }


  // --- Lógica de Toque do Touchscreen (Prioridade Máxima) ---
  if (touchscreen.tirqTouched()) { // Verifica se há uma interrupção de toque
    // A cada toque, obter as coordenadas X, Y e Z
    TS_Point p = touchscreen.getPoint();

    // Mapeia as coordenadas do touchscreen para as coordenadas da tela
    x = map(p.x, touchMinX, touchMaxX, 0, SCREEN_WIDTH);
    y = map(p.y, touchMinY, touchMaxY, 0, SCREEN_HEIGHT);
    z = p.z; // Pressão (Z)

    // Garante que as coordenadas estejam dentro dos limites da tela
    if (x < 0) x = 0;
    if (x >= SCREEN_WIDTH) x = SCREEN_WIDTH - 1;
    if (y < 0) y = 0;
    if (y >= SCREEN_HEIGHT) y = SCREEN_HEIGHT - 1;

    printTouchToSerial(x, y, z); // DEPURACAO DO TOUCH AQUI

    // --- Processar o toque com base no estado atual ---
    if (touchscreen.touched()) { // Processa o clique apenas se o dedo AINDA ESTÁ TOCANDO
      switch (currentState) {
        case STATE_MAIN_SCREEN:
          if (isButtonPressed(menuButton, x, y)) { // Botão MENU reage a qualquer toque dentro de sua área
            Serial.println("Botao MENU pressionado!");
            currentState = STATE_SETUP_MENU;
            drawSetupMenu(); // Desenha a nova tela
          }
          break; // Não usar 'return;' aqui para permitir o while (touchscreen.touched())

        case STATE_SETUP_MENU:
          if (isButtonPressed(wifiConfigButton, x, y)) {
            Serial.println("Botao WiFi pressionado! Iniciando scan...");
            currentState = STATE_WIFI_SCAN;
            performWifiScan();
          } else if (isButtonPressed(mqttButton, x, y)) {
            Serial.println("Botao MQTT pressionado!");
            // Ao entrar na tela de Configuração MQTT, recarrega todos os dados
            loadMqttPath();
            loadApiKey();
            loadMqttUsername();
            loadMqttPassword();
            Serial.print("MQTT CONFIG DISPLAY: Usuario na RAM: ["); Serial.print(storedMqttUsername); Serial.println("]");
            Serial.print("MQTT CONFIG DISPLAY: Senha na RAM: ["); Serial.print(storedMqttPassword); Serial.println("]");

            currentTextInput = String(mqttServerPath); // Carrega o caminho MQTT atual para a string de edição
            currentState = STATE_MQTT_CONFIG_DISPLAY;
            drawMqttConfigDisplayScreen(); // Desenha a nova tela
          } else if (isButtonPressed(clearCredentialsButton, x, y)) {
            Serial.println("Botao Reset pressionado!");
            clearWifiCredentials(); // Esta função reinicia o ESP32.
          } else if (isButtonPressed(backButton, x, y)) {
            Serial.println("Botao Voltar do Menu pressionado!");
            currentState = STATE_MAIN_SCREEN;
            drawMainScreen(); // Desenha a tela principal
          }
          break;

        case STATE_WIFI_SCAN:
          handleWifiScanTouch(x, y);
          break;

        case STATE_WIFI_INPUT:
          handleKeypadTouch(x, y);
          break;

        case STATE_MQTT_CONFIG_DISPLAY:
          if (isButtonPressed(mqttPathEditButton, x, y)) {
            Serial.println("Botao EDIT MQTT Path pressionado!");
            currentFieldBeingEdited = FIELD_MQTT_PATH; // Define o campo a ser editado
            currentTextInput = String(mqttServerPath); // Copia o valor salvo para currentTextInput
            currentState = STATE_TEXT_INPUT; // Vai para o estado genérico de entrada de texto
            drawTextInputScreen("Caminho MQTT", currentTextInput); // Desenha a tela de input
          } else if (isButtonPressed(apiKeyEditButton, x, y)) {
            Serial.println("Botao EDIT API Key pressionado!");
            currentFieldBeingEdited = FIELD_API_KEY; // Define o campo a ser editado
            currentTextInput = String(storedApiKey); // Copia o valor salvo para currentTextInput
            currentState = STATE_TEXT_INPUT; // Vai para o estado genérico de entrada de texto
            drawTextInputScreen("Chave API", currentTextInput); // Desenha a tela de input
          } else if (isButtonPressed(mqttUsernameEditButton, x, y)) { // Botão Usuário MQTT
            Serial.println("Botao EDIT MQTT Username pressionado!");
            currentFieldBeingEdited = FIELD_MQTT_USERNAME; // Define o campo a ser editado
            currentTextInput = String(storedMqttUsername); // Copia o valor salvo para currentTextInput
            Serial.print("EDIT USER: Usuario na RAM (currentTextInput): ["); Serial.print(currentTextInput); Serial.println("]");
            currentState = STATE_TEXT_INPUT;
            drawTextInputScreen("Usuário MQTT", currentTextInput);
          } else if (isButtonPressed(mqttPasswordEditButton, x, y)) { // Botão Senha MQTT
            Serial.println("Botao EDIT MQTT Password pressionado!");
            currentFieldBeingEdited = FIELD_MQTT_PASSWORD; // Define o campo a ser editado
            currentTextInput = String(storedMqttPassword); // Copia o valor salvo para currentTextInput
            Serial.print("EDIT PASS: Senha na RAM (currentTextInput): ["); Serial.print(currentTextInput); Serial.println("]");
            currentState = STATE_TEXT_INPUT;
            drawTextInputScreen("Senha MQTT", currentTextInput);
          } else if (isButtonPressed(testMqttConnectionButton, x, y)) { // NOVO: Botão Testar Conexão MQTT
            Serial.println("Botao TEST MQTT Connection pressionado!");
            currentState = STATE_MQTT_TESTING_CONNECTION; // Muda para o estado de teste
            // A tela de teste será desenhada no loop()
          } else if (isButtonPressed(backButtonMqttConfig, x, y)) { // Botão voltar da tela MQTT Config
            Serial.println("Botao Voltar da Config. MQTT pressionado!");
            currentState = STATE_SETUP_MENU;
            drawSetupMenu(); // Desenha a tela de setup
          }
          break;

        case STATE_TEXT_INPUT:
          handleKeypadTouch(x, y);
          break;
        
        case STATE_WIFI_CONNECTING_SCREEN:
          // NENHUMA LÓGICA DE TOQUE AQUI - esta tela é apenas para feedback visual
          break;

        case STATE_MQTT_TESTING_CONNECTION:
          // NENHUMA LÓGICA DE TOQUE AQUI - esta tela é apenas para feedback visual
          // A transição de estado é gerenciada abaixo no loop()
          break;
      } // Fim do switch (currentState)
    } // Fim do if (touchscreen.touched())
    
    while (touchscreen.touched()) delay(10); // Pequeno delay para anti-bounce do touch
  } // Fim do if (touchscreen.tirqTouched())
  
  // --- Lógica de Monitoramento da Conexão Wi-Fi (FORA DO TOQUE) ---
  if (currentState == STATE_WIFI_CONNECTING_SCREEN) {
      if (WiFi.status() == WL_CONNECTED) {
          currentWifiStatus = WIFI_CONNECTED_OK;
          Serial.println("Wi-Fi conectado com sucesso na tela de status!");
          timeClient.begin();
          timeClient.setTimeOffset(utcOffsetInSeconds);
          if (!timeClient.forceUpdate()) { Serial.println("Falha ao obter a hora do NTP na conexão."); } else { Serial.println("Hora NTP sincronizada."); }
          setupMqttClient();

          drawWifiConnectionStatusScreen("Conectado!", WiFi.SSID().c_str(), TFT_GREEN);
          Serial.println("Exibindo 'Conectado!' por 3 segundos...");
          delay(3000);
          currentState = STATE_MAIN_SCREEN;
          drawMainScreen();
      } else if (currentMillis - wifiConnectingStartTime > WIFI_CONNECTION_TIMEOUT_MS) {
          currentWifiStatus = WIFI_CONNECTION_FAILED;
          Serial.println("Falha na conexao Wi-Fi (timeout).");
          drawWifiConnectionStatusScreen("Falha na Conexao!", selectedSsid.c_str(), TFT_RED);
          Serial.println("Exibindo 'Falha na Conexao!' por 3 segundos...");
          delay(3000);
          currentState = STATE_MAIN_SCREEN; // Volta para a tela principal
          drawMainScreen();
      }
  }

  // --- Lógica de Teste de Conexão MQTT (FORA DO TOQUE) ---
  if (currentState == STATE_MQTT_TESTING_CONNECTION) {
    static unsigned long mqttTestStartTime = 0;
    if (mqttTestStartTime == 0) { // Inicia o timer quando entra no estado
      mqttTestStartTime = currentMillis;
      drawWifiConnectionStatusScreen("Testando MQTT...", mqttServerPath, TFT_YELLOW); // Reusar tela de status
      Serial.println("Iniciando teste de conexao MQTT...");
      // Força uma reconfiguração e reconexão do MQTT
      mqttClientConfigured = false; // Garante que setupMqttClient reconfigure
      setupMqttClient(); // Tenta configurar e conectar
    }

    if (mqttClient.connected()) {
      drawWifiConnectionStatusScreen("MQTT Conectado!", mqttServerPath, TFT_GREEN);
      Serial.println("Teste MQTT: Conectado com sucesso!");
      delay(3000); // Exibe por 3 segundos
      currentState = STATE_MQTT_CONFIG_DISPLAY; // Volta para a tela de configurações MQTT
      drawMqttConfigDisplayScreen();
      mqttTestStartTime = 0; // Reseta o timer
    } else if (currentMillis - mqttTestStartTime > MQTT_CONNECT_TIMEOUT_MS) {
      drawWifiConnectionStatusScreen("MQTT Falhou!", mqttServerPath, TFT_RED);
      Serial.println("Teste MQTT: Falha na conexao (timeout).");
      delay(3000); // Exibe por 3 segundos
      currentState = STATE_MQTT_CONFIG_DISPLAY; // Volta para a tela de configurações MQTT
      drawMqttConfigDisplayScreen();
      mqttTestStartTime = 0; // Reseta o timer
    } else {
      // Se ainda estiver testando, mantém a tela de status do teste
      mqttClient.loop(); // Mantém o loop do cliente MQTT durante o teste
    }
  }

} // Fim do loop()
