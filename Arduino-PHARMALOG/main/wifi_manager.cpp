#include "wifi_manager/wifi_manager.h"
#include "screens/screens.h" // Para drawWifiList(), drawPasswordInputScreen(), drawWifiConnectionStatusScreen()
#include "mqtt_client/mqtt_client.h" // Para setupMqttClient()
#include <string.h> // Para strncpy, strlen

// Carrega as credenciais WiFi salvas na EEPROM
void loadCredentials() {
  EEPROM.readString(EEPROM_SSID_ADDR, storedSsid, MAX_SSID_LEN);
  EEPROM.readString(EEPROM_PASS_ADDR, storedPass, MAX_PASS_LEN);

  Serial.print("SSID carregado: ");
  Serial.println(storedSsid);
  Serial.print("Senha carregada: ");
  Serial.println(storedPass);
}

// Salva as credenciais WiFi na EEPROM
void saveCredentials(const char* ssid, const char* password) {
  Serial.print("Salvando SSID: ");
  Serial.println(ssid);
  Serial.print("Salvando Senha: ");
  Serial.println(password);

  EEPROM.writeString(EEPROM_SSID_ADDR, ssid);
  EEPROM.writeString(EEPROM_PASS_ADDR, password);
  EEPROM.commit();

  Serial.println("Credenciais salvas na EEPROM.");
}

// Realiza a varredura de redes WiFi e exibe na tela
// Esta função bloqueia durante WiFi.scanNetworks()
void performWifiScan() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(FONT_SIZE);
  tft.drawCentreString("Procurando Redes...", SCREEN_WIDTH / 2, 5, FONT_SIZE);

  Serial.println("Iniciando varredura WiFi...");
  numNetworks = WiFi.scanNetworks(); // <--- ESTA FUNÇÃO BLOQUEIA!
  Serial.print("Varredura concluída. ");
  Serial.print(numNetworks);
  Serial.println(" redes encontradas.");

  // Limpa o array ssids antes de preencher
  for (int i = 0; i < MAX_WIFI_NETWORKS_TO_STORE; ++i) {
    ssids[i] = "";
  }

  if (numNetworks == 0) {
    tft.drawCentreString("Nenhuma rede encontrada.", SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2, FONT_SIZE);
    Serial.println("Nenhuma rede encontrada.");
  } else {
    // Armazena no máximo MAX_WIFI_NETWORKS_TO_STORE SSIDs
    for (int i = 0; i < numNetworks && i < MAX_WIFI_NETWORKS_TO_STORE; ++i) {
      ssids[i] = WiFi.SSID(i);
    }
    numNetworks = min(numNetworks, MAX_WIFI_NETWORKS_TO_STORE); // Atualiza numNetworks para o que realmente foi armazenado
    currentScrollOffset = 0; // Reseta o offset de rolagem
    drawWifiList(); // Desenha a lista de redes
  }
  drawButton(backButton); // Desenha o botão de voltar na tela de lista de Wi-Fi
}

// Inicia a tentativa de conexão Wi-Fi (TOTALMENTE NÃO BLOQUEANTE)
// O status será monitorado no loop() principal (main.ino)
void connectToWifi() {
  const char* ssidToConnect;
  const char* passToConnect;

  // Prioriza selectedSsid se disponível, senão storedSsid
  if (selectedSsid.length() > 0) {
    ssidToConnect = selectedSsid.c_str();
    passToConnect = enteredPassword.c_str();
  } else {
    ssidToConnect = storedSsid;
    passToConnect = storedPass;
  }

  Serial.print("Iniciando conexao Wi-Fi para SSID: ");
  Serial.println(ssidToConnect);
  // Não imprima a senha no serial para segurança: Serial.print("Senha: "); Serial.println(passToConnect);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssidToConnect, passToConnect);

  // A função agora é TOTALMENTE não bloqueante.
  // Ela apenas INICIA o processo de conexão.
  // O loop() principal no main.ino é responsável por:
  // - Monitorar o WiFi.status()
  // - Atualizar o currentWifiStatus
  // - Mudar de tela ou exibir mensagens de sucesso/falha
  // Não há 'delay()' nem 'while()' aqui!

  // Salva as credenciais se a tentativa veio da tela de input (após GO!)
  if (selectedSsid.length() > 0) { // Se um SSID foi selecionado na lista (vindo de uma tela de input)
      saveCredentials(selectedSsid.c_str(), enteredPassword.c_str());
      // Copia para as variáveis globais storedSsid e storedPass
      strncpy(storedSsid, selectedSsid.c_str(), MAX_SSID_LEN);
      storedSsid[MAX_SSID_LEN] = '\0';
      strncpy(storedPass, enteredPassword.c_str(), MAX_PASS_LEN);
      storedPass[MAX_PASS_LEN] = '\0';
  }
  
  // Limpa os campos de seleção/entrada após iniciar a conexão
  selectedSsid = "";
  enteredPassword = "";
}

// Trata os toques na tela de varredura WiFi (lista de redes)
void handleWifiScanTouch(int touchX, int touchY) {
  int current_ssid_item_height = tft.fontHeight(FONT_SIZE) + 4;
  int current_max_visible_ssids = (SCREEN_HEIGHT - 25 - backButton.height - 10) / current_ssid_item_height;

  // Verifica se o botão de voltar foi pressionado
  if (isButtonPressed(backButton, touchX, touchY)) {
    Serial.println("Botao Voltar da lista WiFi pressionado!");
    currentState = STATE_SETUP_MENU;
    drawSetupMenu();
    return;
  }

  // Verifica se o botão de rolagem para cima foi pressionado
  if (isButtonPressed(scrollUpButton, touchX, touchY)) {
    Serial.println("Botao Scroll UP pressionado!");
    if (currentScrollOffset > 0) {
      currentScrollOffset--;
      drawWifiList(); // Redesenha a lista
    }
    return;
  }

  // Verifica se o botão de rolagem para baixo foi pressionado
  if (isButtonPressed(scrollDownButton, touchX, touchY)) {
    Serial.println("Botao Scroll DOWN pressionado!");
    if (currentScrollOffset + current_max_visible_ssids < numNetworks) {
      currentScrollOffset++;
      drawWifiList(); // Redesenha a lista
    }
    return;
  }

  // Verifica se um SSID foi selecionado
  for (int i = 0; i < current_max_visible_ssids; ++i) {
    int networkIndex = i + currentScrollOffset;
    if (networkIndex < numNetworks) {
      int ssid_y_start = 28 + i * current_ssid_item_height;
      int ssid_y_end = ssid_y_start + current_ssid_item_height;
      
      // Verifica se o toque está dentro da área do SSID
      if (touchX >= 0 && touchX < (SCREEN_WIDTH - SCROLL_ARROW_WIDTH - 5) &&
          touchY >= ssid_y_start && touchY < ssid_y_end) {
        selectedSsid = ssids[networkIndex];
        Serial.print("SSID selecionado: ");
        Serial.println(selectedSsid);
        enteredPassword = ""; // Limpa qualquer senha anterior
        currentState = STATE_WIFI_INPUT; // Muda para o estado de entrada de senha
        drawPasswordInputScreen(); // Desenha a tela de entrada de senha
        return;
      }
    }
  }
}