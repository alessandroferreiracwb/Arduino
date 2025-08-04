#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include "../globals.h" // Inclui definições globais
#include <WiFi.h>
#include <EEPROM.h>
#include "../mqtt_client/mqtt_client.h" // <--- ADICIONE ESTA LINHA AQUI

// Protótipos das funções de gerenciamento WiFi
void loadCredentials();
void saveCredentials(const char* ssid, const char* password);
void connectToWifi(); // Agora é completamente não bloqueante
void performWifiScan(); // Faz o scan e redesenha a lista
void handleWifiScanTouch(int touchX, int touchY);

#endif // WIFI_MANAGER_H