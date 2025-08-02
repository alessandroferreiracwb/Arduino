#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h> // Para String, etc.
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include <DHT.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <WiFi.h> // Incluir para WiFi.status(), WiFi.localIP(), etc.
#include <PubSubClient.h> // Incluir para o cliente MQTT
#include <string.h> // Adicionado para strlen, strncpy

// --- DEFINICOES EEPROM ---
#define MAX_SSID_LEN 32
#define MAX_PASS_LEN 64
#define EEPROM_SSID_ADDR 0
#define EEPROM_PASS_ADDR (EEPROM_SSID_ADDR + MAX_SSID_LEN + 1)
#define MAX_MQTT_PATH_LEN 100 // Tamanho máximo para o caminho MQTT (ajuste se necessário)
#define EEPROM_MQTT_ADDR (EEPROM_PASS_ADDR + MAX_PASS_LEN + 1) // Endereço na EEPROM para o caminho MQTT

// NOVO: Definições para a Chave API
#define MAX_API_KEY_LEN 100 // Tamanho máximo para a Chave API (ajuste se necessário)
#define EEPROM_API_KEY_ADDR (EEPROM_MQTT_ADDR + MAX_MQTT_PATH_LEN + 1)

// NOVO: Definições para Usuário e Senha MQTT
#define MAX_MQTT_USERNAME_LEN 50 // Tamanho máximo para o usuário MQTT
#define EEPROM_MQTT_USERNAME_ADDR (EEPROM_API_KEY_ADDR + MAX_API_KEY_LEN + 1)
#define MAX_MQTT_PASSWORD_LEN 50 // Tamanho máximo para a senha MQTT
#define EEPROM_MQTT_PASSWORD_ADDR (EEPROM_MQTT_USERNAME_ADDR + MAX_MQTT_USERNAME_LEN + 1)


// ATUALIZE O TAMANHO TOTAL DA EEPROM
// Adicione o tamanho do usuário e senha MQTT
#define EEPROM_SIZE_UPDATED (EEPROM_MQTT_PASSWORD_ADDR + MAX_MQTT_PASSWORD_LEN + 1)

// --- DEFINICOES DHT ---
#define DHTPIN 22
#define DHTTYPE DHT11
#define DHT_READ_INTERVAL 5000 // 5 segundos

// --- DEFINICOES NTP ---
const long utcOffsetInSeconds = -3 * 3600; // Curitiba (GMT-3)

// --- DEFINICOES DE TELA E TOUCH ---
#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240
#define FONT_SIZE 2
#define FONT_SIZE_LARGE 3
#define FONT_SIZE_TITLE 2
#define FONT_SIZE_VALUE 1 // Tamanho da fonte para valores de campo (MQTT Path, API Key)

#define XPT2046_IRQ 36
#define XPT2046_MOSI 32
#define XPT2046_MISO 39
#define XPT2046_CLK 25
#define XPT2046_CS 33

// Valores de Calibração do Touch
const int touchMinX = 406;
const int touchMaxX = 3543;
const int touchMinY = 600;
const int touchMaxY = 3579;

// --- ESTRUTURAS E ENUMS GLOBAIS ---
enum SystemState {
  STATE_MAIN_SCREEN,
  STATE_SETUP_MENU,
  STATE_WIFI_SCAN,
  STATE_WIFI_INPUT,
  STATE_MQTT_CONFIG_DISPLAY,
  STATE_TEXT_INPUT,
  STATE_WIFI_CONNECTING_SCREEN,
  STATE_MQTT_TESTING_CONNECTION
};

enum KeypadMode {
  MODE_LOWERCASE,
  MODE_UPPERCASE,
  MODE_NUMBERS
};

enum FieldBeingEdited {
  FIELD_NONE,
  FIELD_MQTT_PATH,
  FIELD_API_KEY,
  FIELD_MQTT_USERNAME,
  FIELD_MQTT_PASSWORD
};

enum AppMqttConnectionStatus {
  APP_MQTT_DISCONNECTED,
  APP_MQTT_CONNECTING,
  APP_MQTT_CONNECTED
};

enum AppWifiConnectionStatus {
  WIFI_IDLE,          // Sem tentativa de conexão em andamento
  WIFI_CONNECTING,
  WIFI_CONNECTED_OK,
  WIFI_CONNECTION_FAILED
};

struct Button {
  int x;
  int y;
  int width;
  int height;
  const char* text;
  uint16_t color;
  uint16_t textColor;
  uint8_t textSize;
};

// --- DEFINICOES DE BOTOES E TECLADO ---
#define SCROLL_ARROW_WIDTH 30
#define SCROLL_ARROW_HEIGHT 25
#define ALPHANUM_KEY_WIDTH 25
#define ALPHANUM_KEY_HEIGHT 25
#define ALPHANUM_KEY_SPACING_X 3
#define ALPHANUM_KEY_SPACING_Y 3
#define ALPHANUM_KEYBOARD_START_X 10
#define ALPHANUM_KEYBOARD_START_Y 80
#define NUM_CONTROL_KEYS 4
#define MAX_WIFI_NETWORKS_TO_STORE 10
#define PASSWORD_INPUT_HEIGHT 35
#define TEXT_INPUT_HEIGHT 20 // Altura da caixa de texto

#define EDIT_BUTTON_WIDTH 45 // Largura reduzida para o botão Edit
#define EDIT_BUTTON_HEIGHT TEXT_INPUT_HEIGHT // Altura do botão Edit igual à caixa de texto


// Teclado: Definições de layout e contagem de teclas
#define NUM_NUM_ROW_KEYS 10
#define NUM_ALPHA_ROW1_KEYS 10
#define NUM_ALPHA_ROW2_KEYS 9
#define NUM_ALPHA_ROW3_KEYS 9
#define NUM_SYMBOL_KEYS_ROW1 10
#define NUM_SYMBOL_KEYS_ROW2 9
#define NUM_SYMBOL_KEYS_ROW3 7

#define TOTAL_ALPHANUM_KEYS (NUM_NUM_ROW_KEYS + NUM_ALPHA_ROW1_KEYS + NUM_ALPHA_ROW2_KEYS + NUM_ALPHA_ROW3_KEYS + NUM_SYMBOL_KEYS_ROW1 + NUM_SYMBOL_KEYS_ROW2 + NUM_SYMBOL_KEYS_ROW3)
#define MAX_ALPHANUM_KEYS TOTAL_ALPHANUM_KEYS

// --- DEFINICOES DE DADOS DE SENSOR ---
#define MAX_BUFFERED_READINGS 20
#define MQTT_CONNECT_TIMEOUT_MS 10000


struct SensorData {
    float temperature;
    float humidity;
    char timestamp[20];
};


// --- VARIAVEIS GLOBAIS (Declaradas como extern) ---
extern TFT_eSPI tft;
extern SPIClass touchscreenSPI;
extern XPT2046_Touchscreen touchscreen;
extern DHT dht;
extern WiFiUDP ntpUDP;
extern NTPClient timeClient;

extern int x, y, z;
extern char storedSsid[MAX_SSID_LEN + 1];
extern char storedPass[MAX_PASS_LEN + 1];
extern float temperature;
extern float humidity;
extern unsigned long lastDHTReadTime;
extern SystemState currentState;
extern KeypadMode currentKeypadMode;
extern FieldBeingEdited currentFieldBeingEdited;
extern AppMqttConnectionStatus currentMqttStatus;
extern AppWifiConnectionStatus currentWifiStatus;

extern int numNetworks;
extern String ssids[MAX_WIFI_NETWORKS_TO_STORE];
extern String selectedSsid;
extern int currentScrollOffset;
extern String enteredPassword;
extern String currentTextInput;

extern char mqttServerPath[MAX_MQTT_PATH_LEN + 1];
extern char storedApiKey[MAX_API_KEY_LEN + 1];
extern char storedMqttUsername[MAX_MQTT_USERNAME_LEN + 1];
extern char storedMqttPassword[MAX_MQTT_PASSWORD_LEN + 1];

// Variáveis para o cliente MQTT
extern WiFiClient espClient;
extern PubSubClient mqttClient;
extern char mqttClientId[30];
extern bool mqttClientConfigured;

// Buffer de dados do sensor
extern SensorData bufferedReadings[MAX_BUFFERED_READINGS];
extern int currentBufferIndex;
extern bool bufferFull;

// Informações da versão do firmware
extern const char* FIRMWARE_VERSION;

// --- Botões ---
extern Button menuButton;
extern Button wifiConfigButton;
extern Button backButton;
extern Button backButtonKeypad;
extern Button scrollUpButton;
extern Button scrollDownButton;
extern Button clearCredentialsButton;
extern Button mqttButton;

extern Button mqttPathEditButton;
extern Button apiKeyEditButton;
extern Button mqttUsernameEditButton;
extern Button mqttPasswordEditButton;
extern Button testMqttConnectionButton;

extern Button backButtonMqttConfig;

extern Button controlKeys[NUM_CONTROL_KEYS];
extern Button alphaNumKeys[MAX_ALPHANUM_KEYS];

// Nomes dos botões de controle do teclado
extern const char* SHIFT_TEXT;
extern const char* MODE_TEXT;
extern const char* BACK_TEXT;
extern const char* ENTER_TEXT;

// Arrays com os caracteres para cada modo do teclado (ORDEM ALTERADA)
extern const char* NUMBER_KEYS_TOP[];
extern const int NUM_NUMBER_KEYS_TOP;

extern const char* LOWERCASE_KEYS_ALPHA[];
extern const int NUM_LOWERCASE_KEYS_ALPHA;

extern const char* UPPERCASE_KEYS_ALPHA[];
extern const int NUM_UPPERCASE_KEYS_ALPHA;

extern const char* SYMBOL_KEYS[];
extern const int NUM_SYMBOL_KEYS;

// --- Funções Auxiliares Comuns (protótipos) ---
void drawButton(Button btn);
bool isButtonPressed(Button btn, int touchX, int touchY);
void printTouchToSerial(int touchX, int touchY, int touchZ);
void clearWifiCredentials();
void saveMqttPath(const char* path);
void loadMqttPath();
void saveApiKey(const char* key);
void loadApiKey();
void saveMqttUsername(const char* username);
void loadMqttUsername();
void saveMqttPassword(const char* password);
void loadMqttPassword();
void getFormattedDateTime(char* buffer, size_t bufferSize);

#endif // GLOBALS_H
