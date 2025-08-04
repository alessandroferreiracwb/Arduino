// globals.cpp (Arquivo na pasta raiz do projeto)
#include "globals.h" // Inclui o cabeçalho para ter as declarações 'extern'
#include <EEPROM.h> // Necessário para as funções EEPROM
#include <ESP.h> // Necessário para ESP.restart()
#include <time.h> // Para gmtime e strftime

// Definições de instâncias globais
TFT_eSPI tft = TFT_eSPI();
SPIClass touchscreenSPI = SPIClass(VSPI);
XPT2046_Touchscreen touchscreen(XPT2046_CS, XPT2046_IRQ);
DHT dht(DHTPIN, DHTTYPE);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

// Variáveis globais
int x, y, z;
char storedSsid[MAX_SSID_LEN + 1] = {0};
char storedPass[MAX_PASS_LEN + 1] = {0};
float temperature = 0.0;
float humidity = 0.0;
unsigned long lastDHTReadTime = 0;
SystemState currentState = STATE_MAIN_SCREEN; // Usa SystemState
KeypadMode currentKeypadMode = MODE_LOWERCASE;
FieldBeingEdited currentFieldBeingEdited = FIELD_NONE; // Inicializa como nenhum campo sendo editado
AppMqttConnectionStatus currentMqttStatus = APP_MQTT_DISCONNECTED; // Estado inicial MQTT
AppWifiConnectionStatus currentWifiStatus = WIFI_IDLE; // Estado inicial Wi-Fi

int numNetworks = 0;
String ssids[MAX_WIFI_NETWORKS_TO_STORE];
String selectedSsid = "";
String enteredPassword = "";
String currentTextInput = ""; // String genérica para o texto de entrada
int currentScrollOffset = 0;

char mqttServerPath[MAX_MQTT_PATH_LEN + 1] = {0};
char storedApiKey[MAX_API_KEY_LEN + 1] = {0}; // Definição da Chave API
char storedMqttUsername[MAX_MQTT_USERNAME_LEN + 1] = {0}; // Variável para Usuário MQTT
char storedMqttPassword[MAX_MQTT_PASSWORD_LEN + 1] = {0}; // Variável para Senha MQTT


// Variáveis para o cliente MQTT
WiFiClient espClient;
PubSubClient mqttClient(espClient);
char mqttClientId[30] = {0}; // Definido como char array e inicializado com 0s
bool mqttClientConfigured = false; // Flag para saber se o cliente MQTT está configurado

// Buffer de dados do sensor
SensorData bufferedReadings[MAX_BUFFERED_READINGS];
int currentBufferIndex = 0; // Índice para o próximo ponto de dados a ser armazenado
bool bufferFull = false; // Flag para indicar se o buffer já preencheu pelo menos uma vez

// Informações da versão do firmware
const char* FIRMWARE_VERSION = "v1.0.0";


// Definição de botões fixos
Button menuButton = {
  .x = 10,  // <--- Canto inferior esquerdo
  .y = SCREEN_HEIGHT - 60, // <--- Canto inferior esquerdo
  .width = 60, // <--- ALTERADO: Mesma largura do botão de voltar
  .height = 40,  // <--- ALTERADO: Mesma altura do botão de voltar
  .text = "", // <--- ALTERADO: Sem texto, como o botão de voltar
  .color = TFT_BLUE,
  .textColor = TFT_WHITE,
  .textSize = FONT_SIZE // Usaremos o drawButton para desenhar o triângulo
};


// **POSICIONAMENTO AJUSTADO PARA A TELA DE CONFIGURAÇÕES**
// Essas definições agora têm 'width' e 'text' finais
// As posições x e y serão definidas dinamicamente em screens.cpp
Button wifiConfigButton = {
  .x = 0, // Placeholder
  .y = 0, // Placeholder
  .width = 100, // Largura ajustada para 'WiFi'
  .height = 40,
  .text = "WiFi", // Texto do botão 'WiFi'
  .color = TFT_ORANGE,
  .textColor = TFT_WHITE,
  .textSize = FONT_SIZE
};

Button mqttButton = {
  .x = 0, // Placeholder
  .y = 0, // Placeholder
  .width = 100, // Largura ajustada para 'MQTT'
  .height = 40,
  .text = "MQTT", // Texto do botão 'MQTT'
  .color = TFT_BLUE,
  .textColor = TFT_WHITE,
  .textSize = FONT_SIZE
};

Button clearCredentialsButton = {
  .x = 0, // Placeholder
  .y = 0, // Placeholder
  .width = 100, // Largura ajustada para 'Reset'
  .height = 40,
  .text = "Reset", // TEXTO ATUALIZADO AQUI PARA "Reset"
  .color = TFT_RED,
  .textColor = TFT_WHITE,
  .textSize = FONT_SIZE
};

// BOTÃO VOLTAR AGORA COM TEXTO DE SETA E POSIÇÃO FIXA INFERIOR ESQUERDA
Button backButton = {
  .x = 10, // Canto esquerdo inferior
  .y = SCREEN_HEIGHT - 40 - 10, // 10px acima da borda inferior
  .width = 60, // Largura ajustada para caber a seta
  .height = 40,
  .text = "<-", // TEXTO ATUALIZADO AQUI PARA SETA
  .color = TFT_RED,
  .textColor = TFT_WHITE,
  .textSize = FONT_SIZE // Usar FONT_SIZE para a seta ser visível
};

Button backButtonKeypad = {
  .x = 10,
  .y = 5,
  .width = 60,
  .height = 30,
  .text = "<-",
  .color = TFT_RED,
  .textColor = TFT_WHITE,
  .textSize = 1
};

Button backButtonMqttConfig = {
  .x = 10,
  .y = SCREEN_HEIGHT - 40 - 10,
  .width = 60,
  .height = 40,
  .text = "<-",
  .color = TFT_RED,
  .textColor = TFT_WHITE,
  .textSize = FONT_SIZE
};


Button scrollUpButton = {
  .x = SCREEN_WIDTH - SCROLL_ARROW_WIDTH - 5,
  .y = 45,
  .width = SCROLL_ARROW_WIDTH,
  .height = SCROLL_ARROW_HEIGHT,
  .text = "",
  .color = TFT_TRANSPARENT,
  .textColor = TFT_TRANSPARENT,
  .textSize = 1
};

Button scrollDownButton = {
  .x = SCREEN_WIDTH - SCROLL_ARROW_WIDTH - 5,
  .y = SCREEN_HEIGHT - backButton.height - 10 - SCROLL_ARROW_HEIGHT - 5,
  .width = SCROLL_ARROW_WIDTH,
  .height = SCROLL_ARROW_HEIGHT,
  .text = "",
  .color = TFT_TRANSPARENT,
  .textColor = TFT_TRANSPARENT,
  .textSize = 1
};

Button saveMqttButton = { // Botão Salvar para MQTT (usado na tela de entrada de texto)
  .x = (SCREEN_WIDTH / 2) + 5, // Posição à direita
  .y = SCREEN_HEIGHT - 40 - 10, // Mesma altura do botão 'Voltar'
  .width = 70, // Largura adequada para "Salvar"
  .height = 40,
  .text = "Salvar",
  .color = TFT_GREEN,
  .textColor = TFT_WHITE,
  .textSize = FONT_SIZE
};

// Botões de edição na tela de configurações MQTT - ALTURA E LARGURA REDUZIDAS
Button mqttPathEditButton = {
  .x = 0, // Placeholder
  .y = 0, // Placeholder
  .width = EDIT_BUTTON_WIDTH,
  .height = EDIT_BUTTON_HEIGHT,
  .text = "Edit",
  .color = TFT_GREEN,
  .textColor = TFT_WHITE,
  .textSize = 1
};

Button apiKeyEditButton = {
  .x = 0, // Placeholder
  .y = 0, // Placeholder
  .width = EDIT_BUTTON_WIDTH,
  .height = EDIT_BUTTON_HEIGHT,
  .text = "Edit",
  .color = TFT_GREEN,
  .textColor = TFT_WHITE,
  .textSize = 1
};

// Botões para editar Usuário e Senha MQTT - ALTURA E LARGURA REDUZIDAS
Button mqttUsernameEditButton = {
  .x = 0, // Placeholder
  .y = 0, // Placeholder
  .width = EDIT_BUTTON_WIDTH,
  .height = EDIT_BUTTON_HEIGHT,
  .text = "Edit",
  .color = TFT_GREEN,
  .textColor = TFT_WHITE,
  .textSize = 1
};

Button mqttPasswordEditButton = {
  .x = 0, // Placeholder
  .y = 0, // Placeholder
  .width = EDIT_BUTTON_WIDTH,
  .height = EDIT_BUTTON_HEIGHT,
  .text = "Edit",
  .color = TFT_GREEN,
  .textColor = TFT_WHITE,
  .textSize = 1
};

// Botão para testar conexão MQTT
Button testMqttConnectionButton = {
  .x = 0, // Placeholder
  .y = 0, // Placeholder
  .width = 120,
  .height = 40,
  .text = "Testar Conexao",
  .color = TFT_CYAN,
  .textColor = TFT_BLACK,
  .textSize = 1
};


// Definições dos botões de controle do teclado
const char* SHIFT_TEXT = "Shift";
const char* MODE_TEXT = "ABC"; // Inicialmente "ABC"
const char* BACK_TEXT = "DEL";
const char* ENTER_TEXT = "GO!";

// Arrays com os caracteres para cada modo do teclado (NOVO LAYOUT)
const char* NUMBER_KEYS_TOP[] = { // Primeira linha: Números
  "1","2","3","4","5","6","7","8","9","0"
};
const int NUM_NUMBER_KEYS_TOP = sizeof(NUMBER_KEYS_TOP) / sizeof(NUMBER_KEYS_TOP[0]);

const char* LOWERCASE_KEYS_ALPHA[] = { // Alfabeto minúsculo + . e /
  "q","w","e","r","t","y","u","i","o","p",
  "a","s","d","f","g","h","j","k","l",
  "z","x","c","v","b","n","m",".","/" // Ponto e barra no alfabeto
};
const int NUM_LOWERCASE_KEYS_ALPHA = sizeof(LOWERCASE_KEYS_ALPHA) / sizeof(LOWERCASE_KEYS_ALPHA[0]);

const char* UPPERCASE_KEYS_ALPHA[] = { // Alfabeto maiúsculo + . e /
  "Q","W","E","R","T","Y","U","I","O","P",
  "A","S","D","F","G","H","J","K","L",
  "Z","X","C","V","B","N","M",".","/" // Ponto e barra no alfabeto
};
const int NUM_UPPERCASE_KEYS_ALPHA = sizeof(UPPERCASE_KEYS_ALPHA) / sizeof(UPPERCASE_KEYS_ALPHA[0]);

const char* SYMBOL_KEYS[] = { // Caracteres especiais (para o modo de números/símbolos)
  "!","@","#","$","%","&","*","(",")","~",
  "-","+","=","_","`",",",";",":","'",
  "\"","\\","|","<",">","?","^"
};
const int NUM_SYMBOL_KEYS = sizeof(SYMBOL_KEYS) / sizeof(SYMBOL_KEYS[0]);


Button controlKeys[NUM_CONTROL_KEYS];
Button alphaNumKeys[MAX_ALPHANUM_KEYS]; // Ajustado o tamanho do array

// --- Funções Auxiliares Comuns (implementações) ---

// Desenha um botão na tela
void drawButton(Button btn) {
  if (btn.color != TFT_TRANSPARENT) {
    tft.fillRect(btn.x, btn.y, btn.width, btn.height, btn.color);
  }
  tft.drawRect(btn.x, btn.y, btn.width, btn.height, TFT_BLACK);

  if (btn.text[0] != '\0') {
    tft.setTextColor(btn.textColor);
    tft.setTextSize(btn.textSize);
    tft.drawCentreString(btn.text, btn.x + btn.width / 2, btn.y + (btn.height - tft.fontHeight(btn.textSize)) / 2, btn.textSize);
  }
}

// Verifica se um toque ocorreu dentro da área de um botão
bool isButtonPressed(Button btn, int touchX, int touchY) {
  return (touchX >= btn.x && touchX <= (btn.x + btn.width) &&
          touchY >= btn.y && touchY <= (btn.y + btn.height));
}

// Imprime as informações do toque no Monitor Serial
void printTouchToSerial(int touchX, int touchY, int touchZ) {
  Serial.print("X = ");
  Serial.print(touchX);
  Serial.print(" | Y = ");
  Serial.print(touchY);
  Serial.print(" | Pressure = ");
  Serial.print(touchZ);
  Serial.println();
}

// Função para apagar as credenciais WiFi salvas da EEPROM
void clearWifiCredentials() {
  Serial.println("Apagando credenciais...");

  memset(storedSsid, 0, sizeof(storedSsid));
  memset(storedPass, 0, sizeof(storedPass));
  memset(mqttServerPath, 0, sizeof(mqttServerPath));
  memset(storedApiKey, 0, sizeof(storedApiKey));
  memset(storedMqttUsername, 0, sizeof(storedMqttUsername)); // Limpa usuário MQTT
  memset(storedMqttPassword, 0, sizeof(storedMqttPassword)); // Limpa senha MQTT

  EEPROM.writeString(EEPROM_SSID_ADDR, "");
  EEPROM.writeString(EEPROM_PASS_ADDR, "");
  EEPROM.writeString(EEPROM_MQTT_ADDR, "");
  EEPROM.writeString(EEPROM_API_KEY_ADDR, "");
  EEPROM.writeString(EEPROM_MQTT_USERNAME_ADDR, ""); // Escreve vazio para usuário MQTT
  EEPROM.writeString(EEPROM_MQTT_PASSWORD_ADDR, ""); // Escreve vazio para senha MQTT
  EEPROM.commit();

  Serial.println("Credenciais WiFi, MQTT Path, API Key, Usuário e Senha MQTT apagadas da EEPROM e da RAM.");

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(FONT_SIZE);
  tft.drawCentreString("Credenciais Apagadas!", SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2 - (tft.fontHeight(FONT_SIZE)/2), FONT_SIZE);

  delay(2000);

  Serial.println("Reiniciando ESP32...");
  ESP.restart();
}

// Função para salvar o caminho do servidor MQTT na EEPROM
void saveMqttPath(const char* path) {
  Serial.print("Salvando caminho MQTT: ");
  Serial.println(path);
  // Garante que o path seja copiado para o buffer char[] e termine em null
  strncpy(mqttServerPath, path, MAX_MQTT_PATH_LEN);
  mqttServerPath[MAX_MQTT_PATH_LEN] = '\0'; // Garante null-termination
  EEPROM.writeString(EEPROM_MQTT_ADDR, mqttServerPath); // Escreve o char[] na EEPROM
  EEPROM.commit();
  Serial.println("Caminho MQTT salvo na EEPROM.");
  mqttClientConfigured = false; // Marcar como não configurado para forçar reconfiguração
}

// Função para carregar o caminho do servidor MQTT da EEPROM
void loadMqttPath() {
  EEPROM.readString(EEPROM_MQTT_ADDR, mqttServerPath, MAX_MQTT_PATH_LEN);
  Serial.print("Caminho MQTT carregado: ");
  Serial.println(mqttServerPath);
}

// Função para salvar a Chave API na EEPROM
void saveApiKey(const char* key) {
  Serial.print("Salvando Chave API: ");
  Serial.println(key);
  // Garante que a key seja copiada para o buffer char[] e termine em null
  strncpy(storedApiKey, key, MAX_API_KEY_LEN);
  storedApiKey[MAX_API_KEY_LEN] = '\0'; // Garante null-termination
  EEPROM.writeString(EEPROM_API_KEY_ADDR, storedApiKey); // Escreve o char[] na EEPROM
  EEPROM.commit();
  Serial.println("Chave API salva na EEPROM.");
  mqttClientConfigured = false; // Marcar como não configurado para forçar reconfiguração
}

// Função para carregar a Chave API da EEPROM
void loadApiKey() {
  EEPROM.readString(EEPROM_API_KEY_ADDR, storedApiKey, MAX_API_KEY_LEN);
  Serial.print("Chave API carregada: ");
  Serial.println(storedApiKey);
}

// NOVO: Função para salvar o usuário MQTT na EEPROM
void saveMqttUsername(const char* username) {
  Serial.print("Salvando Usuário MQTT: ");
  Serial.println(username);
  strncpy(storedMqttUsername, username, MAX_MQTT_USERNAME_LEN);
  storedMqttUsername[MAX_MQTT_USERNAME_LEN] = '\0';
  EEPROM.writeString(EEPROM_MQTT_USERNAME_ADDR, storedMqttUsername);
  EEPROM.commit();
  Serial.println("Usuário MQTT salvo na EEPROM.");
  mqttClientConfigured = false;
}

// NOVO: Função para carregar o usuário MQTT da EEPROM
void loadMqttUsername() {
  EEPROM.readString(EEPROM_MQTT_USERNAME_ADDR, storedMqttUsername, MAX_MQTT_USERNAME_LEN);
  Serial.print("Usuário MQTT carregado: ");
  Serial.println(storedMqttUsername);
}

// NOVO: Função para salvar a senha MQTT na EEPROM
void saveMqttPassword(const char* password) {
  Serial.print("Salvando Senha MQTT: ");
  Serial.println(password);
  strncpy(storedMqttPassword, password, MAX_MQTT_PASSWORD_LEN);
  storedMqttPassword[MAX_MQTT_PASSWORD_LEN] = '\0';
  EEPROM.writeString(EEPROM_MQTT_PASSWORD_ADDR, storedMqttPassword);
  EEPROM.commit();
  Serial.println("Senha MQTT salva na EEPROM.");
  mqttClientConfigured = false;
}

// NOVO: Função para carregar a senha MQTT da EEPROM
void loadMqttPassword() {
  EEPROM.readString(EEPROM_MQTT_PASSWORD_ADDR, storedMqttPassword, MAX_MQTT_PASSWORD_LEN);
  Serial.print("Senha MQTT carregada: ");
  Serial.println(storedMqttPassword);
}


// Função para formatar a data e hora atual
void getFormattedDateTime(char* buffer, size_t bufferSize) {
  time_t epochTime = timeClient.getEpochTime();
  struct tm *ptm = localtime(&epochTime); // Use localtime para o fuso horário configurado
  
  // Verifica se a hora ainda não foi sincronizada (ano 1970).
  // A biblioteca timeClient.getEpochTime() retorna 0 antes da sincronização.
  // tm_year é anos desde 1900, então 1970 -> 70. Se for 0 ou algo bem pequeno, não sincronizado.
  if (ptm == NULL || ptm->tm_year < 100) { // Ano < 2000 (100 = 1900 + 100)
      snprintf(buffer, bufferSize, "YYYY-MM-DD HH:MM:SS"); // Retorna um placeholder
  } else {
      strftime(buffer, bufferSize, "%Y-%m-%d %H:%M:%S", ptm);
  }
}
