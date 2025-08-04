#include "screens/screens.h"
#include <cmath> // Para isnan()

// --- Draw the main screen ---
void drawMainScreen() {
  tft.fillScreen(TFT_BLACK); // Fundo preto para a tela toda

  // --- Área do Topo: Data, Hora, Status Wi-Fi e MQTT ---
  // Variáveis de posicionamento para o topo
  int top_y_line1 = 5;  // Linha para a hora e status Wi-Fi
  int top_y_line2 = 20; // Linha para a data e status MQTT
  int text_start_x = 5;

  if (WiFi.status() == WL_CONNECTED) {
    timeClient.update();
    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE, TFT_BLACK); // Texto branco com fundo preto
    time_t epochTime = timeClient.getEpochTime();
    struct tm *ptm = gmtime (&epochTime);
    char dateBuffer[11];
    sprintf(dateBuffer, "%04d-%02d-%02d", ptm->tm_year + 1900, ptm->tm_mon + 1, ptm->tm_mday);
    String dateString = String(dateBuffer);
    String formattedTime = timeClient.getFormattedTime();

    // Linha 1: Hora e status do Wi-Fi
    tft.setTextDatum(TL_DATUM);
    tft.drawString(formattedTime, text_start_x, top_y_line1, 1);
    String wifiStatusText = "";
    uint16_t wifiStatusColor = TFT_WHITE;
    switch(currentWifiStatus) {
      case WIFI_IDLE: case WIFI_CONNECTION_FAILED: wifiStatusText = "Desconectado"; wifiStatusColor = TFT_RED; break;
      case WIFI_CONNECTING: wifiStatusText = "Conectando..."; wifiStatusColor = TFT_ORANGE; break;
      case WIFI_CONNECTED_OK: wifiStatusText = "Conectado"; wifiStatusColor = TFT_GREEN; break;
    }
    tft.setTextColor(wifiStatusColor, TFT_BLACK);
    tft.setTextDatum(TR_DATUM); // Alinha no canto superior direito
    tft.drawString("WiFi: " + wifiStatusText, SCREEN_WIDTH - 5, top_y_line1, 1);
    tft.setTextDatum(TL_DATUM);

    // Linha 2: Data e status do MQTT
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString(dateString, text_start_x, top_y_line2, 1);
    String mqttStatusText = "";
    uint16_t mqttStatusColor = TFT_WHITE;
    switch (currentMqttStatus) {
      case APP_MQTT_CONNECTED: mqttStatusText += "Conectado"; mqttStatusColor = TFT_GREEN; break;
      case APP_MQTT_DISCONNECTED: mqttStatusText += "Desconectado"; mqttStatusColor = TFT_RED; break;
      case APP_MQTT_CONNECTING: mqttStatusText += "Conectando..."; mqttStatusColor = TFT_ORANGE; break;
    }
    tft.setTextColor(mqttStatusColor, TFT_BLACK);
    tft.setTextDatum(TR_DATUM); // Alinha no canto superior direito
    tft.drawString("MQTT: " + mqttStatusText, SCREEN_WIDTH - 5, top_y_line2, 1);
    tft.setTextDatum(TL_DATUM);

  } else {
    // Se desconectado
    tft.setTextSize(1);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawCentreString("Wi-Fi: Desconectado", SCREEN_WIDTH/2, 10, 1);
  }

  // Título "PHARMALOG"
  tft.setTextSize(2);
  tft.setTextColor(TFT_SKYBLUE, TFT_BLACK);
  tft.drawCentreString("PHARMALOG", SCREEN_WIDTH / 2, SCREEN_HEIGHT/2 - 60, 2);

  // Área para Temperatura e Umidade
  tft.setTextSize(FONT_SIZE_LARGE);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  
  int text_offset_y = (75 - (tft.fontHeight(FONT_SIZE_LARGE) * 2 + 5)) / 2;
  if (text_offset_y < 0) text_offset_y = 0;

  String tempDisplay = isnan(temperature) ? "XX C" : String(temperature, 1) + " C";
  tft.drawCentreString("Temp.: " + tempDisplay, SCREEN_WIDTH / 2, 60 + text_offset_y, 1);

  String humDisplay = isnan(humidity) ? "XX %" : String(humidity, 0) + " %";
  tft.drawCentreString("Umid.: " + humDisplay, SCREEN_WIDTH / 2, 90 + text_offset_y + tft.fontHeight(FONT_SIZE_LARGE) + 5, 1);

  // --- Botão MENU (Canto Inferior Esquerdo) ---
  drawButton(menuButton);
  tft.drawRect(menuButton.x, menuButton.y, menuButton.width, menuButton.height, TFT_WHITE); // Borda branca
  tft.fillRect(menuButton.x, menuButton.y, menuButton.width, menuButton.height, TFT_BLUE); // Fundo azul
  
  // Desenha o ícone de menu (3 barras) no centro do botão
  int icon_x = menuButton.x + menuButton.width / 2;
  int icon_y = menuButton.y + menuButton.height / 2;
  int bar_width = 30;
  int bar_height = 4;
  int bar_spacing = 6;
  tft.fillRect(icon_x - bar_width / 2, icon_y - bar_height * 1.5, bar_width, bar_height, TFT_WHITE);
  tft.fillRect(icon_x - bar_width / 2, icon_y - bar_height / 2, bar_width, bar_height, TFT_WHITE);
  tft.fillRect(icon_x - bar_width / 2, icon_y + bar_height / 2, bar_width, bar_height, TFT_WHITE);

}


// --- Draw the setup menu ---
void drawSetupMenu() {
  tft.fillScreen(TFT_DARKGREY);

  tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
  tft.setTextSize(FONT_SIZE_TITLE);
  tft.drawCentreString("CONFIGURACOES", SCREEN_WIDTH / 2, 20, FONT_SIZE_TITLE);

  int spacing_y = 15;
  int current_y = 20 + tft.fontHeight(FONT_SIZE_TITLE) + spacing_y;

  wifiConfigButton.x = (SCREEN_WIDTH / 2) - (wifiConfigButton.width / 2);
  wifiConfigButton.y = current_y;
  drawButton(wifiConfigButton);
  current_y += wifiConfigButton.height + spacing_y;

  mqttButton.x = (SCREEN_WIDTH / 2) - (mqttButton.width / 2);
  mqttButton.y = current_y;
  drawButton(mqttButton);
  current_y += mqttButton.height + spacing_y;

  clearCredentialsButton.x = (SCREEN_WIDTH / 2) - (clearCredentialsButton.width / 2);
  clearCredentialsButton.y = current_y;
  drawButton(clearCredentialsButton);

  drawButton(backButton);

  // Exibir versão do firmware no canto inferior direito
  tft.setTextSize(1); // Tamanho menor para a versão
  tft.setTextColor(TFT_LIGHTGREY); // Cor discreta para a versão
  tft.setTextDatum(BR_DATUM); // Alinha o texto pelo canto Bottom-Right
  // Posição: 5px da borda direita e 5px da borda inferior
  tft.drawString(FIRMWARE_VERSION, SCREEN_WIDTH - 5, SCREEN_HEIGHT - 5, 1);
  tft.setTextDatum(TL_DATUM); // Volta para alinhamento Top-Left padrão
}

void drawWifiList() {
  tft.fillScreen(TFT_BLUE); // Fundo azul para a tela de lista de Wi-Fi

  int list_text_area_width = SCREEN_WIDTH - 10 - SCROLL_ARROW_WIDTH - 5;
  int ssid_text_start_x = 10; // Posição X para o texto do SSID

  tft.setTextSize(FONT_SIZE); // Tamanho da fonte para os SSIDs
  int current_ssid_item_height = tft.fontHeight(FONT_SIZE) + 4; // Altura de cada item na lista (texto + padding)
  int current_max_visible_ssids = (SCREEN_HEIGHT - 25 - backButton.height - 10) / current_ssid_item_height;

  // Percorre e desenha os SSIDs visíveis
  for (int i = 0; i < current_max_visible_ssids; ++i) {
    int networkIndex = i + currentScrollOffset;
    int item_y_pos = 28 + i * current_ssid_item_height; // Posição Y para o item

    // Define as cores de fundo alternadas (zebra)
    uint16_t bgColor = (i % 2 == 0) ? TFT_DARKGREY : TFT_BLACK; // CORES ALTERNADAS AQUI
    
    tft.fillRect(0, item_y_pos, list_text_area_width, current_ssid_item_height, bgColor); // Desenha o fundo da zebra
    
    if (networkIndex < numNetworks) {
      tft.setTextColor(TFT_WHITE, bgColor); // Texto branco sobre o fundo alternado do item
      tft.setCursor(ssid_text_start_x, item_y_pos + (current_ssid_item_height - tft.fontHeight(FONT_SIZE)) / 2); // Centraliza verticalmente o texto
      
      String ssidToDisplay = ssids[networkIndex];
      
      // Lógica de truncagem do SSID se for muito longo
      if (tft.textWidth(ssidToDisplay, FONT_SIZE) > (list_text_area_width - ssid_text_start_x - tft.textWidth("...", FONT_SIZE))) {
        String originalSsid = ssidToDisplay;
        ssidToDisplay = "";
        for (char c : originalSsid) {
            if (tft.textWidth(ssidToDisplay + c + "...", FONT_SIZE) < (list_text_area_width - ssid_text_start_x)) {
                ssidToDisplay += c;
            } else {
                break;
            }
        }
        if (ssidToDisplay.length() < originalSsid.length()) { // Se houve truncagem
            ssidToDisplay += "...";
        }
      }
      tft.print(ssidToDisplay);
    } else {
      // Limpa linhas vazias, mantendo a cor de fundo alternada (já preenchida acima)
    }
  }

  // Desenha as setas de rolagem sobre o fundo azul
  tft.fillRect(scrollUpButton.x, scrollUpButton.y, scrollUpButton.width, scrollUpButton.height, TFT_BLUE);
  tft.fillRect(scrollDownButton.x, scrollDownButton.y, scrollDownButton.width, scrollDownButton.height, TFT_BLUE);

  if (currentScrollOffset > 0) { // Desenha seta para cima apenas se houver o que rolar para cima
    int arrow_center_y_up = scrollUpButton.y + (SCROLL_ARROW_HEIGHT / 2);
    tft.fillTriangle(scrollUpButton.x + SCROLL_ARROW_WIDTH / 2,
                     arrow_center_y_up - 6,
                     scrollUpButton.x + 5,
                     arrow_center_y_up + 10,
                     scrollUpButton.x + SCROLL_ARROW_WIDTH - 5,
                     arrow_center_y_up + 10,
                     TFT_YELLOW);
  }
  
  if (currentScrollOffset + current_max_visible_ssids < numNetworks) { // Desenha seta para baixo apenas se houver o que rolar para baixo
    int arrow_center_y_down = scrollDownButton.y + (SCROLL_ARROW_HEIGHT / 2);
    tft.fillTriangle(scrollDownButton.x + SCROLL_ARROW_WIDTH / 2,
                     arrow_center_y_down + 10,
                     scrollDownButton.x + 5,
                     arrow_center_y_down - 6,
                     scrollDownButton.x + SCROLL_ARROW_WIDTH - 5,
                     arrow_center_y_down - 6,
                     TFT_YELLOW);
  }

  drawButton(backButton); // Desenha o botão de voltar por último, para ficar por cima
}

// Função para desenhar a tela de exibição das configurações MQTT
void drawMqttConfigDisplayScreen() {
  tft.fillScreen(TFT_DARKGREY);
  tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
  
  // Título "CONFIGURACOES" no canto superior central
  tft.setTextSize(FONT_SIZE_TITLE); // Tamanho da fonte grande
  tft.setTextDatum(MC_DATUM); // Alinha pelo centro (Middle-Center)
  tft.drawString("CONFIGURACOES", SCREEN_WIDTH / 2, 20, FONT_SIZE_TITLE);
  tft.setTextDatum(TL_DATUM); // Volta para alinhamento Top-Left padrão

  // Botão Voltar (<-) no canto inferior esquerdo
  backButtonMqttConfig.x = 10;
  backButtonMqttConfig.y = SCREEN_HEIGHT - 40 - 10; // Posição padrão do botão Voltar (inferior esquerdo)
  drawButton(backButtonMqttConfig);


  // Declaração das variáveis de layout no escopo da função
  int text_start_x = 10;
  // Largura total para campos e botões Edit (SCREEN_WIDTH - 20px de margem)
  int total_available_width = SCREEN_WIDTH - (text_start_x * 2); 
  // Largura das caixas de texto (ajustada para aproximadamente 75% da área disponível)
  int field_display_width = (total_available_width * 3 / 4) - EDIT_BUTTON_WIDTH - 5;
  if (field_display_width < 60) field_display_width = 60; // Garante um tamanho mínimo

  int edit_button_x_offset = text_start_x + field_display_width + 5; // Posição X para os botões "Edit"

  // Posição Y inicial para os campos, subindo 20% da tela (240 * 0.20 = 48)
  int current_y = (SCREEN_HEIGHT * 0.20); // MANTIDO: Começa em Y=48px (20% do topo da tela)


  // --- Campo Caminho Servidor MQTT ---
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE);
  tft.drawString("Servidor:", text_start_x, current_y, 1);
  current_y += tft.fontHeight(1) + 2;

  int text_display_y_mqtt_path = current_y;
  tft.fillRect(text_start_x, text_display_y_mqtt_path, field_display_width, TEXT_INPUT_HEIGHT, TFT_WHITE);
  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  tft.setTextSize(FONT_SIZE_VALUE);
  String mqttPathDisplay = String(mqttServerPath);
  String tempMqttPath = "";
  for (char c : mqttPathDisplay) {
      if (tft.textWidth(tempMqttPath + c + "...", FONT_SIZE_VALUE) < field_display_width - 10) {
          tempMqttPath += c;
      } else {
          break;
      }
  }
  if (tempMqttPath.length() < mqttPathDisplay.length()) { mqttPathDisplay = tempMqttPath + "..."; } else { mqttPathDisplay = tempMqttPath; }
  tft.drawString(mqttPathDisplay, text_start_x + 5, text_display_y_mqtt_path + (TEXT_INPUT_HEIGHT - tft.fontHeight(FONT_SIZE_VALUE)) / 2, FONT_SIZE_VALUE);

  mqttPathEditButton.x = edit_button_x_offset;
  mqttPathEditButton.y = text_display_y_mqtt_path + (TEXT_INPUT_HEIGHT - mqttPathEditButton.height) / 2;
  drawButton(mqttPathEditButton);
  current_y += TEXT_INPUT_HEIGHT + 5;

  // --- Campo Usuário MQTT ---
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE);
  tft.drawString("Usuario:", text_start_x, current_y, 1);
  current_y += tft.fontHeight(1) + 2;

  int text_display_y_mqtt_user = current_y;
  tft.fillRect(text_start_x, text_display_y_mqtt_user, field_display_width, TEXT_INPUT_HEIGHT, TFT_WHITE);
  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  tft.setTextSize(FONT_SIZE_VALUE);
  String mqttUsernameDisplay = String(storedMqttUsername);
  String tempMqttUsername = "";
  for (char c : mqttUsernameDisplay) {
      if (tft.textWidth(tempMqttUsername + c + "...", FONT_SIZE_VALUE) < field_display_width - 10) {
          tempMqttUsername += c;
      } else {
          break;
      }
  }
  if (tempMqttUsername.length() < mqttUsernameDisplay.length()) { mqttUsernameDisplay = tempMqttUsername + "..."; } else { mqttUsernameDisplay = tempMqttUsername; }
  tft.drawString(mqttUsernameDisplay, text_start_x + 5, text_display_y_mqtt_user + (TEXT_INPUT_HEIGHT - tft.fontHeight(FONT_SIZE_VALUE)) / 2, FONT_SIZE_VALUE);

  mqttUsernameEditButton.x = edit_button_x_offset;
  mqttUsernameEditButton.y = text_display_y_mqtt_user + (TEXT_INPUT_HEIGHT - mqttUsernameEditButton.height) / 2;
  drawButton(mqttUsernameEditButton);
  current_y += TEXT_INPUT_HEIGHT + 5;

  // --- Campo Senha MQTT ---
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE);
  tft.drawString("Senha:", text_start_x, current_y, 1);
  current_y += tft.fontHeight(1) + 2;

  int text_display_y_mqtt_pass = current_y;
  tft.fillRect(text_start_x, text_display_y_mqtt_pass, field_display_width, TEXT_INPUT_HEIGHT, TFT_WHITE);
  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  tft.setTextSize(FONT_SIZE_VALUE);
  String mqttPasswordDisplay = String(storedMqttPassword);
  String maskedPasswordDisplay = "";
  for(int i=0; i<mqttPasswordDisplay.length(); ++i) maskedPasswordDisplay += '*';

  String tempMqttPassword = "";
  for (char c : maskedPasswordDisplay) {
      if (tft.textWidth(tempMqttPassword + c + "...", FONT_SIZE_VALUE) < field_display_width - 10) {
          tempMqttPassword += c;
      } else {
          break;
      }
  }
  if (tempMqttPassword.length() < maskedPasswordDisplay.length()) { mqttPasswordDisplay = tempMqttPassword + "..."; } else { mqttPasswordDisplay = tempMqttPassword; }
  tft.drawString(mqttPasswordDisplay, text_start_x + 5, text_display_y_mqtt_pass + (TEXT_INPUT_HEIGHT - tft.fontHeight(FONT_SIZE_VALUE)) / 2, FONT_SIZE_VALUE);

  mqttPasswordEditButton.x = edit_button_x_offset;
  mqttPasswordEditButton.y = text_display_y_mqtt_pass + (TEXT_INPUT_HEIGHT - mqttPasswordEditButton.height) / 2;
  drawButton(mqttPasswordEditButton);
  current_y += TEXT_INPUT_HEIGHT + 5;

  // --- Campo Chave API ---
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE);
  tft.drawString("Chave API:", text_start_x, current_y, 1);
  current_y += tft.fontHeight(1) + 2;

  int text_display_y_api = current_y;
  tft.fillRect(text_start_x, text_display_y_api, field_display_width, TEXT_INPUT_HEIGHT, TFT_WHITE);
  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  tft.setTextSize(FONT_SIZE_VALUE);
  String apiKeyDisplay = String(storedApiKey);
  String tempApiKey = "";
  for (char c : apiKeyDisplay) {
      if (tft.textWidth(tempApiKey + c + "...", FONT_SIZE_VALUE) < field_display_width - 10) {
          tempApiKey += c;
      } else {
          break;
      }
  }
  if (tempApiKey.length() < apiKeyDisplay.length()) { apiKeyDisplay = tempApiKey + "..."; } else { apiKeyDisplay = tempApiKey; }
  tft.drawString(apiKeyDisplay, text_start_x + 5, text_display_y_api + (TEXT_INPUT_HEIGHT - tft.fontHeight(FONT_SIZE_VALUE)) / 2, FONT_SIZE_VALUE);

  apiKeyEditButton.x = edit_button_x_offset;
  apiKeyEditButton.y = text_display_y_api + (TEXT_INPUT_HEIGHT - apiKeyEditButton.height) / 2;
  drawButton(apiKeyEditButton);
  current_y += TEXT_INPUT_HEIGHT + 10;

  // --- Botão Testar Conexão MQTT ---
  testMqttConnectionButton.x = (SCREEN_WIDTH / 2) - (testMqttConnectionButton.width / 2);
  testMqttConnectionButton.y = current_y;
  drawButton(testMqttConnectionButton);
  drawButton(backButtonMqttConfig);
}

// Renomeada e ajustada para ser genérica para entrada de texto (senha, MQTT Path, API Key)
void drawTextInputScreen(const char* title, const String& currentText) {
  tft.fillScreen(TFT_DARKCYAN);
  tft.setTextColor(TFT_WHITE, TFT_DARKCYAN);

  drawButton(backButtonKeypad); // Botão voltar do teclado

  tft.setTextSize(1);
  tft.drawCentreString(title, SCREEN_WIDTH / 2, 5, 1); // Título dinâmico
  tft.drawCentreString("Digite:", SCREEN_WIDTH / 2, 25, 1);

  tft.fillRect(20, 45, SCREEN_WIDTH - 40, TEXT_INPUT_HEIGHT, TFT_WHITE);
  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  tft.setTextSize(FONT_SIZE); // Mantém o tamanho da fonte de entrada do teclado como FONT_SIZE (2)
  tft.drawCentreString(currentText, SCREEN_WIDTH / 2, 45 + (TEXT_INPUT_HEIGHT - tft.fontHeight(FONT_SIZE)) / 2, FONT_SIZE);

  drawAlphaNumericKeypad(); // Chama a função para desenhar o teclado alfanumérico
}

// Função original drawPasswordInputScreen agora redireciona para a genérica
void drawPasswordInputScreen() {
  drawTextInputScreen("Senha WiFi", enteredPassword);
}

// Função para exibir tela de status de conexão Wi-Fi
void drawWifiConnectionStatusScreen(const char* message, const char* ssid, uint16_t color) {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(color);
  tft.setTextSize(FONT_SIZE);
  tft.drawCentreString(message, SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2 - 20, FONT_SIZE);
  if (ssid && strlen(ssid) > 0) {
    tft.drawCentreString(ssid, SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2 + 10, FONT_SIZE);
  }
}
