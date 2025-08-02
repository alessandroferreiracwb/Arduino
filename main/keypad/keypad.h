#ifndef KEYPAD_H
#define KEYPAD_H

#include "../globals.h" // Inclui definições globais
#include "../screens/screens.h" // Para drawTextInputScreen, drawPasswordInputScreen, drawWifiConnectionStatusScreen
#include "../wifi_manager/wifi_manager.h" // Para connectToWifi, performWifiScan

// Protótipos das funções do teclado
void initAlphaNumericKeys();
void drawAlphaNumericKeypad();
void handleKeypadTouch(int touchX, int touchY);

#endif // KEYPAD_H