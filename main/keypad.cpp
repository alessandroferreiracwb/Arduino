#include "keypad/keypad.h" // Inclui o cabeçalho específico do keypad
#include "globals.h"       // Caminho direto para globals.h na raiz
#include <string.h>        // Para strlen, strcpy

// Inicializa os botões do teclado alfanumérico
void initAlphaNumericKeys() {
  // Ajustando as posições base para as teclas de caracter
  // As definições de alphaNumKeys[i] na verdade não precisam de posições fixas aqui,
  // pois drawAlphaNumericKeypad as calcula. Apenas garantimos o array alocado.
  for (int i = 0; i < MAX_ALPHANUM_KEYS; ++i) {
    alphaNumKeys[i] = {
      .x = 0, .y = 0, // Placeholder, posições calculadas no desenho
      .width = ALPHANUM_KEY_WIDTH,
      .height = ALPHANUM_KEY_HEIGHT,
      .text = "", // Texto será definido dinamicamente
      .color = TFT_LIGHTGREY,
      .textColor = TFT_BLACK,
      .textSize = 1
    };
  }

  // Define as teclas de controle (posições fixas em relação ao teclado)
  // Calculadas para a última linha do teclado
  int control_key_y = ALPHANUM_KEYBOARD_START_Y + 4 * (ALPHANUM_KEY_HEIGHT + ALPHANUM_KEY_SPACING_Y); // 4 linhas acima (números + 3 linhas de alfabeto)

  // Shift
  controlKeys[0] = {
    .x = ALPHANUM_KEYBOARD_START_X,
    .y = control_key_y,
    .width = ALPHANUM_KEY_WIDTH * 2, // Mais largo
    .height = ALPHANUM_KEY_HEIGHT,
    .text = SHIFT_TEXT,
    .color = TFT_DARKGREY,
    .textColor = TFT_WHITE,
    .textSize = 1
  };

  // Mode (123 / ABC)
  controlKeys[1] = {
    .x = ALPHANUM_KEYBOARD_START_X + (ALPHANUM_KEY_WIDTH * 2) + ALPHANUM_KEY_SPACING_X,
    .y = control_key_y,
    .width = ALPHANUM_KEY_WIDTH * 2, // Mais largo
    .height = ALPHANUM_KEY_HEIGHT,
    .text = MODE_TEXT,
    .color = TFT_DARKGREY,
    .textColor = TFT_WHITE,
    .textSize = 1
  };

  // Backspace (DEL)
  controlKeys[2] = {
    .x = ALPHANUM_KEYBOARD_START_X + (ALPHANUM_KEY_WIDTH * 4) + (ALPHANUM_KEY_SPACING_X * 2),
    .y = control_key_y,
    .width = ALPHANUM_KEY_WIDTH * 2, // Mais largo
    .height = ALPHANUM_KEY_HEIGHT,
    .text = BACK_TEXT,
    .color = TFT_RED,
    .textColor = TFT_WHITE,
    .textSize = 1
  };

  // Enter (GO!)
  controlKeys[3] = {
    .x = ALPHANUM_KEYBOARD_START_X + (ALPHANUM_KEY_WIDTH * 6) + (ALPHANUM_KEY_SPACING_X * 3),
    .y = control_key_y,
    .width = SCREEN_WIDTH - (ALPHANUM_KEYBOARD_START_X + (ALPHANUM_KEY_WIDTH * 6) + (ALPHANUM_KEY_SPACING_X * 3)) - ALPHANUM_KEY_SPACING_X, // Preenche o resto da linha
    .height = ALPHANUM_KEY_HEIGHT,
    .text = ENTER_TEXT,
    .color = TFT_GREEN,
    .textColor = TFT_WHITE,
    .textSize = 1
  };
}

// Desenha o teclado alfanumérico
void drawAlphaNumericKeypad() {
  tft.fillRect(0, ALPHANUM_KEYBOARD_START_Y - 5, SCREEN_WIDTH, SCREEN_HEIGHT - (ALPHANUM_KEYBOARD_START_Y - 5), TFT_DARKCYAN);
  tft.setTextColor(TFT_BLACK, TFT_LIGHTGREY);
  tft.setTextSize(1);

  const char** currentKeyset; // Usaremos um conjunto de chaves de cada vez
  int numKeysInSet;
  int currentKeyIndex = 0; // Índice para alphaNumKeys

  int startY = ALPHANUM_KEYBOARD_START_Y;
  int currentY = startY;

  // --- Linha 1: NÚMEROS (0-9) ---
  currentKeyset = NUMBER_KEYS_TOP;
  numKeysInSet = NUM_NUMBER_KEYS_TOP;
  int row_width_num = numKeysInSet * ALPHANUM_KEY_WIDTH + (numKeysInSet - 1) * ALPHANUM_KEY_SPACING_X;
  int row_start_x_num = (SCREEN_WIDTH - row_width_num) / 2;

  int currentX = row_start_x_num;
  for (int i = 0; i < numKeysInSet; ++i) {
    alphaNumKeys[currentKeyIndex].x = currentX;
    alphaNumKeys[currentKeyIndex].y = currentY;
    alphaNumKeys[currentKeyIndex].text = currentKeyset[i];
    drawButton(alphaNumKeys[currentKeyIndex]);
    currentX += ALPHANUM_KEY_WIDTH + ALPHANUM_KEY_SPACING_X;
    currentKeyIndex++;
  }
  currentY += ALPHANUM_KEY_HEIGHT + ALPHANUM_KEY_SPACING_Y; // Próxima linha para alfabeto

  // --- Linhas 2-4: ALFABETO + . e / (e Símbolos no modo secundário) ---
  // Seleciona o conjunto de chaves do alfabeto (minúsculo ou maiúsculo)
  if (currentKeypadMode == MODE_LOWERCASE) {
    currentKeyset = LOWERCASE_KEYS_ALPHA;
    numKeysInSet = NUM_LOWERCASE_KEYS_ALPHA;
  } else if (currentKeypadMode == MODE_UPPERCASE) {
    currentKeyset = UPPERCASE_KEYS_ALPHA;
    numKeysInSet = NUM_UPPERCASE_KEYS_ALPHA;
  } else { // MODE_NUMBERS (símbolos)
    currentKeyset = SYMBOL_KEYS;
    numKeysInSet = NUM_SYMBOL_KEYS;
  }

  // Calcula larguras para centralizar as linhas do alfabeto
  int row1_alpha_width = NUM_ALPHA_ROW1_KEYS * ALPHANUM_KEY_WIDTH + (NUM_ALPHA_ROW1_KEYS - 1) * ALPHANUM_KEY_SPACING_X;
  int row1_alpha_start_x = (SCREEN_WIDTH - row1_alpha_width) / 2;

  int row2_alpha_width = NUM_ALPHA_ROW2_KEYS * ALPHANUM_KEY_WIDTH + (NUM_ALPHA_ROW2_KEYS - 1) * ALPHANUM_KEY_SPACING_X;
  int row2_alpha_start_x = (SCREEN_WIDTH - row2_alpha_width) / 2;

  int row3_alpha_width = NUM_ALPHA_ROW3_KEYS * ALPHANUM_KEY_WIDTH + (NUM_ALPHA_ROW3_KEYS - 1) * ALPHANUM_KEY_SPACING_X;
  int row3_alpha_start_x = (SCREEN_WIDTH - row3_alpha_width) / 2;

  // Desenha as 3 linhas do alfabeto/símbolos
  for (int i = 0; i < numKeysInSet; ++i) {
    if (i < NUM_ALPHA_ROW1_KEYS) { // Primeira linha do alfabeto (QWERTY)
      currentX = row1_alpha_start_x + i * (ALPHANUM_KEY_WIDTH + ALPHANUM_KEY_SPACING_X);
      alphaNumKeys[currentKeyIndex].y = currentY;
    } else if (i < (NUM_ALPHA_ROW1_KEYS + NUM_ALPHA_ROW2_KEYS)) { // Segunda linha (ASDF)
      currentX = row2_alpha_start_x + (i - NUM_ALPHA_ROW1_KEYS) * (ALPHANUM_KEY_WIDTH + ALPHANUM_KEY_SPACING_X);
      alphaNumKeys[currentKeyIndex].y = currentY + (ALPHANUM_KEY_HEIGHT + ALPHANUM_KEY_SPACING_Y);
    } else { // Terceira linha (ZXCVBNM./)
      currentX = row3_alpha_start_x + (i - (NUM_ALPHA_ROW1_KEYS + NUM_ALPHA_ROW2_KEYS)) * (ALPHANUM_KEY_WIDTH + ALPHANUM_KEY_SPACING_X);
      alphaNumKeys[currentKeyIndex].y = currentY + 2 * (ALPHANUM_KEY_HEIGHT + ALPHANUM_KEY_SPACING_Y);
    }
    alphaNumKeys[currentKeyIndex].x = currentX;
    alphaNumKeys[currentKeyIndex].text = currentKeyset[i];
    drawButton(alphaNumKeys[currentKeyIndex]);
    currentKeyIndex++;
  }

  // Desenha os botões de controle (Shift, Mode, Del, Go)
  for (int i = 0; i < NUM_CONTROL_KEYS; ++i) {
    drawButton(controlKeys[i]);
  }
}


// Função genérica para tratar toques no teclado de entrada de texto
void handleKeypadTouch(int touchX, int touchY) {
  // Trata as teclas alfanuméricas e de símbolos
  const char** currentKeyset;
  int numKeyset;

  // Determina o conjunto de chaves a ser usado para detecção
  if (currentKeypadMode == MODE_LOWERCASE) {
    currentKeyset = LOWERCASE_KEYS_ALPHA;
    numKeyset = NUM_LOWERCASE_KEYS_ALPHA;
  } else if (currentKeypadMode == MODE_UPPERCASE) {
    currentKeyset = UPPERCASE_KEYS_ALPHA;
    numKeyset = NUM_UPPERCASE_KEYS_ALPHA;
  } else { // MODE_NUMBERS
    currentKeyset = NUMBER_KEYS_TOP; // Para números
    numKeyset = NUM_NUMBER_KEYS_TOP;
  }
  
  // Verifica as teclas numéricas da primeira linha (sempre visíveis)
  for (int i = 0; i < NUM_NUMBER_KEYS_TOP; ++i) {
    // Encontre o botão correspondente no array alphaNumKeys
    // Ele estaria nos primeiros NUM_NUMBER_KEYS_TOP índices do alphaNumKeys
    if (isButtonPressed(alphaNumKeys[i], touchX, touchY)) {
      String key = NUMBER_KEYS_TOP[i]; // Pega o caractere real
      if (currentState == STATE_WIFI_INPUT) {
        if (enteredPassword.length() < MAX_PASS_LEN) {
          enteredPassword += key;
          drawPasswordInputScreen();
        }
      } else if (currentState == STATE_TEXT_INPUT) {
        int maxLen = (currentFieldBeingEdited == FIELD_MQTT_PATH) ? MAX_MQTT_PATH_LEN : MAX_API_KEY_LEN;
        if (currentFieldBeingEdited == FIELD_MQTT_USERNAME) maxLen = MAX_MQTT_USERNAME_LEN;
        if (currentFieldBeingEdited == FIELD_MQTT_PASSWORD) maxLen = MAX_MQTT_PASSWORD_LEN;

        if (currentTextInput.length() < maxLen) {
            currentTextInput += key;
            drawTextInputScreen(currentFieldBeingEdited == FIELD_MQTT_PATH ? "Caminho MQTT" : 
                               (currentFieldBeingEdited == FIELD_MQTT_USERNAME ? "Usuário MQTT" :
                               (currentFieldBeingEdited == FIELD_MQTT_PASSWORD ? "Senha MQTT" : "Chave API")), currentTextInput);
        }
      }
      return;
    }
  }

  // Verifica as teclas do alfabeto/símbolos
  // Os índices em alphaNumKeys serão deslocados após os números
  int alphaStartIndex = NUM_NUMBER_KEYS_TOP;
  if (currentKeypadMode == MODE_LOWERCASE || currentKeypadMode == MODE_UPPERCASE) {
    currentKeyset = (currentKeypadMode == MODE_LOWERCASE) ? LOWERCASE_KEYS_ALPHA : UPPERCASE_KEYS_ALPHA;
    numKeyset = NUM_LOWERCASE_KEYS_ALPHA; // Ambas têm o mesmo número de teclas de alfabeto + . /
  } else { // MODE_NUMBERS (símbolos)
    currentKeyset = SYMBOL_KEYS;
    numKeyset = NUM_SYMBOL_KEYS;
  }

  for (int i = 0; i < numKeyset; ++i) {
    if (isButtonPressed(alphaNumKeys[alphaStartIndex + i], touchX, touchY)) {
      String key = currentKeyset[i];
      if (currentState == STATE_WIFI_INPUT) {
        if (enteredPassword.length() < MAX_PASS_LEN) {
          enteredPassword += key;
          drawPasswordInputScreen();
        }
      } else if (currentState == STATE_TEXT_INPUT) {
        int maxLen = (currentFieldBeingEdited == FIELD_MQTT_PATH) ? MAX_MQTT_PATH_LEN : MAX_API_KEY_LEN;
        if (currentFieldBeingEdited == FIELD_MQTT_USERNAME) maxLen = MAX_MQTT_USERNAME_LEN;
        if (currentFieldBeingEdited == FIELD_MQTT_PASSWORD) maxLen = MAX_MQTT_PASSWORD_LEN;

        if (currentTextInput.length() < maxLen) {
            currentTextInput += key;
            drawTextInputScreen(currentFieldBeingEdited == FIELD_MQTT_PATH ? "Caminho MQTT" : 
                               (currentFieldBeingEdited == FIELD_MQTT_USERNAME ? "Usuário MQTT" :
                               (currentFieldBeingEdited == FIELD_MQTT_PASSWORD ? "Senha MQTT" : "Chave API")), currentTextInput);
        }
      }
      return;
    }
  }

  // Trata as teclas de controle (SHIFT, MODE, DEL, GO!, Voltar do teclado)
  if (isButtonPressed(controlKeys[0], touchX, touchY)) { // Shift
    Serial.println("Botao SHIFT pressionado!");
    if (currentKeypadMode == MODE_LOWERCASE) currentKeypadMode = MODE_UPPERCASE;
    else currentKeypadMode = MODE_LOWERCASE;
    if (currentState == STATE_WIFI_INPUT) drawPasswordInputScreen();
    else if (currentState == STATE_TEXT_INPUT) drawTextInputScreen(currentFieldBeingEdited == FIELD_MQTT_PATH ? "Caminho MQTT" : 
                               (currentFieldBeingEdited == FIELD_MQTT_USERNAME ? "Usuário MQTT" :
                               (currentFieldBeingEdited == FIELD_MQTT_PASSWORD ? "Senha MQTT" : "Chave API")), currentTextInput);
    return;
  } else if (isButtonPressed(controlKeys[1], touchX, touchY)) { // Mode (123 / ABC / Símbolos)
    Serial.println("Botao MODE pressionado!");
    if (currentKeypadMode == MODE_NUMBERS) {
      currentKeypadMode = MODE_LOWERCASE; // Volta para minúsculas do modo números
      controlKeys[1].text = "123"; // Texto do botão MODE para ir para números
    } else {
      currentKeypadMode = MODE_NUMBERS; // Vai para números/símbolos
      controlKeys[1].text = "ABC"; // Texto do botão MODE para ir para letras
    }
    // Redesenha a tela para atualizar o layout do teclado
    if (currentState == STATE_WIFI_INPUT) drawPasswordInputScreen();
    else if (currentState == STATE_TEXT_INPUT) drawTextInputScreen(currentFieldBeingEdited == FIELD_MQTT_PATH ? "Caminho MQTT" : 
                               (currentFieldBeingEdited == FIELD_MQTT_USERNAME ? "Usuário MQTT" :
                               (currentFieldBeingEdited == FIELD_MQTT_PASSWORD ? "Senha MQTT" : "Chave API")), currentTextInput);
    return;
  } else if (isButtonPressed(controlKeys[2], touchX, touchY)) { // Backspace (DEL)
    if (currentState == STATE_WIFI_INPUT) {
      if (enteredPassword.length() > 0) {
        enteredPassword.remove(enteredPassword.length() - 1);
        drawPasswordInputScreen();
      }
    } else if (currentState == STATE_TEXT_INPUT) {
      if (currentTextInput.length() > 0) {
        currentTextInput.remove(currentTextInput.length() - 1);
        drawTextInputScreen(currentFieldBeingEdited == FIELD_MQTT_PATH ? "Caminho MQTT" : 
                               (currentFieldBeingEdited == FIELD_MQTT_USERNAME ? "Usuário MQTT" :
                               (currentFieldBeingEdited == FIELD_MQTT_PASSWORD ? "Senha MQTT" : "Chave API")), currentTextInput);
      }
    }
    return;
  } else if (isButtonPressed(controlKeys[3], touchX, touchY)) { // Enter (GO!)
    if (currentState == STATE_WIFI_INPUT) {
      currentWifiStatus = WIFI_CONNECTING; // Define status de conexão Wi-Fi
      connectToWifi(); // Inicia a conexão Wi-Fi (não bloqueante)
      currentState = STATE_WIFI_CONNECTING_SCREEN; // Vai para a tela de feedback de conexão
      drawWifiConnectionStatusScreen("Conectando Wi-Fi...", selectedSsid.c_str(), TFT_YELLOW); // Desenha a tela
    } else if (currentState == STATE_TEXT_INPUT) {
      Serial.print("GO! - Salvando campo ");
      Serial.print(currentFieldBeingEdited);
      Serial.print(": ["); Serial.print(currentTextInput); Serial.println("]");

      if (currentFieldBeingEdited == FIELD_MQTT_PATH) {
        saveMqttPath(currentTextInput.c_str());
      } else if (currentFieldBeingEdited == FIELD_API_KEY) {
        saveApiKey(currentTextInput.c_str());
      } else if (currentFieldBeingEdited == FIELD_MQTT_USERNAME) {
        Serial.print("SALVANDO USUARIO: ["); Serial.print(currentTextInput); Serial.println("]"); // Depuração extra
        saveMqttUsername(currentTextInput.c_str());
      } else if (currentFieldBeingEdited == FIELD_MQTT_PASSWORD) {
        Serial.print("SALVANDO SENHA: ["); Serial.print(currentTextInput); Serial.println("]"); // Depuração extra
        saveMqttPassword(currentTextInput.c_str());
      }
      currentTextInput = "";
      currentFieldBeingEdited = FIELD_NONE;
      currentState = STATE_MQTT_CONFIG_DISPLAY; // Retorna para a tela de configurações MQTT
      drawMqttConfigDisplayScreen();
    }
    return;
  } else if (isButtonPressed(backButtonKeypad, touchX, touchY)) { // Botão voltar do teclado
    Serial.println("Botao Voltar do Teclado pressionado!");
    if (currentState == STATE_WIFI_INPUT) {
      enteredPassword = ""; // Limpa a senha se voltar
      currentState = STATE_WIFI_SCAN; // Volta para a lista de redes
      performWifiScan(); // Redesenha a lista de redes
    } else if (currentState == STATE_TEXT_INPUT) {
      currentTextInput = ""; // Limpa a entrada se voltar
      currentFieldBeingEdited = FIELD_NONE; // Reseta o campo em edição
      currentState = STATE_MQTT_CONFIG_DISPLAY; // Volta para a tela de exibição MQTT
      drawMqttConfigDisplayScreen();
    }
    return;
  }
}
