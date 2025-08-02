#ifndef SCREENS_H
#define SCREENS_H

#include "../globals.h"
#include <WiFi.h>
#include "../keypad/keypad.h" // Para drawAlphaNumericKeypad()

// Protótipos das funções de desenho de tela
void drawMainScreen();
void drawSetupMenu();
void drawWifiList();
void drawPasswordInputScreen();
void drawMqttConfigDisplayScreen(); // Protótipo para a tela de exibição MQTT
// Ajusta o protótipo para ser genérico para qualquer entrada de texto
void drawTextInputScreen(const char* title, const String& currentText);
void drawWifiConnectionStatusScreen(const char* message, const char* ssid, uint16_t color); // <--- NOVO: Tela de status de conexão Wi-Fi

#endif // SCREENS_H