#include <SPI.h>
#include <TFT_eSPI.h>
#include <Preferences.h>
#include <XPT2046_Touchscreen.h>

// --- Definições de Pinos e Calibração do seu hardware ---
#define TFT_CS 15
#define TFT_DC 2
#define TFT_RST -1
#define TFT_MOSI 13
#define TFT_SCLK 14
#define TFT_MISO 12

// Pinos SPI do seu touch
#define XPT2046_MOSI 32
#define XPT2046_MISO 39
#define XPT2046_CLK 25
#define XPT2046_CS 33
#define XPT2046_IRQ 36

// Valores de calibração do seu touch
int touchMinX = 451;
int touchMaxX = 3598;
int touchMinY = 600;
int touchMaxY = 3579;
// -----------------------------------------------------------

TFT_eSPI tft = TFT_eSPI();
SPIClass touchscreenSPI = SPIClass(VSPI);
XPT2046_Touchscreen ts(XPT2046_CS, XPT2046_IRQ);

// Variáveis de estado
enum Screen { MAIN_SCREEN, SETUP_SCREEN, SEND_SCREEN };
Screen currentScreen = MAIN_SCREEN;

// Variáveis para as configurações
Preferences preferences;
long canSpeed = 250000;
bool isExtendedID = false;

// Variáveis para a tela de envio
unsigned long sendId = 0x123;
String sendFrame = "00:00:00:00:00:00:00:00";
int sendInterval = 1000;

// Mapeamento da velocidade é mantido para fins de UI
long getCanSpeedCode(long speed) {
  if (speed == 125000) return 125;
  if (speed == 250000) return 250;
  if (speed == 500000) return 500;
  return 250; 
}

// Estrutura para mensagens
struct CanMessage {
  unsigned long id;
  unsigned char data[8];
  unsigned char len;
  bool isExtended;
};
CanMessage canMessages[20];
int messageCount = 0;
int scrollPosition = 0;

// Protótipos das funções
void drawMainScreen();
void drawSetupScreen();
void drawSendScreen();
void drawHeader();
void drawCanMessages();
void handleTouch();
void saveConfig();
void loadConfig();
void processSerialData(String serialData);
void drawButton(int x, int y, int w, int h, const char* label, uint16_t bgColor, uint16_t borderColor);

void setup() {
  Serial.begin(115200);
  SPI.begin();

  tft.init();
  tft.setRotation(1); 
  tft.fillScreen(TFT_BLACK);

  // Inicialização do SPI e do Touch
  touchscreenSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
  ts.begin(touchscreenSPI);
  ts.setRotation(1);

  loadConfig();
  
  // Dados simulados para teste do display
  canMessages[0] = {0x18FEEF00, {0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80}, 8, true};
  canMessages[1] = {0x18FEEF01, {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF}, 6, true};
  canMessages[2] = {0x123, {0x01, 0x02, 0x03, 0x04}, 4, false};
  messageCount = 3;
  
  drawMainScreen();
}

void loop() {
  handleTouch();

  if (currentScreen == MAIN_SCREEN && Serial.available()) {
    String serialData = Serial.readStringUntil('\n');
    processSerialData(serialData);
  }
}

// Processa a string serial e a exibe na tela
void processSerialData(String serialData) {
  serialData.trim();
  int spaceIndex = serialData.indexOf(' ');
  if (spaceIndex == -1) {
    return;
  }
  
  String idString = serialData.substring(0, spaceIndex);
  String dataString = serialData.substring(spaceIndex + 1);

  unsigned long id = strtoul(idString.c_str(), NULL, 16);
  int len = dataString.length() / 2;
  unsigned char buf[8];
  
  if (len > 8) {
    len = 8;
  }
  
  for(int i = 0; i < len; i++) {
    String byteString = dataString.substring(i * 2, i * 2 + 2);
    buf[i] = strtoul(byteString.c_str(), NULL, 16);
  }

  if (messageCount < 20) {
    canMessages[messageCount].id = id;
    canMessages[messageCount].len = len;
    canMessages[messageCount].isExtended = (idString.length() > 3);
    memcpy(canMessages[messageCount].data, buf, len);
    messageCount++;
  } else {
    for (int i = 0; i < 19; i++) {
      canMessages[i] = canMessages[i+1];
    }
    canMessages[19].id = id;
    canMessages[19].len = len;
    canMessages[19].isExtended = (idString.length() > 3);
    memcpy(canMessages[19].data, buf, len);
  }
  
  drawCanMessages();
}

// --- Funções de Desenho das Telas ---
void drawMainScreen() {
    tft.fillScreen(TFT_BLACK);
    drawHeader();
    drawCanMessages();
}

void drawHeader() {
  tft.fillRect(0, 0, tft.width(), tft.height(), TFT_BLACK); 
  
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE);
  tft.setTextDatum(BC_DATUM); 
  tft.drawString("CAN BUS", tft.width() / 2, tft.height() - 5); 
  
  // Botão SEND no canto inferior esquerdo
  drawButton(10, tft.height() - 40, 80, 30, "SEND", TFT_RED, TFT_WHITE);

  // Botão SETUP no canto inferior direito
  drawButton(tft.width() - 90, tft.height() - 40, 80, 30, "SETUP", TFT_BLUE, TFT_WHITE);
}

void drawCanMessages() {
  tft.fillRect(0, 0, tft.width(), tft.height() - 60, TFT_BLACK); 
  tft.setTextDatum(TL_DATUM);
  
  tft.setTextSize(1);
  
  int y = 5; 
  for (int i = 0; i < messageCount; i++) {
    uint16_t color = (i % 2 == 0) ? TFT_LIGHTGREY : TFT_CYAN;
    tft.setTextColor(color);
    
    String idString = String(canMessages[i].id, HEX);
    idString.toUpperCase();

    String frameData = "";
    for (int j = 0; j < canMessages[i].len; j++) {
      if (canMessages[i].data[j] < 16) frameData += "0";
      frameData += String(canMessages[i].data[j], HEX);
      
      if (j < canMessages[i].len - 1) {
        frameData += ":";
      }
    }
    frameData.toUpperCase();
    
    String line = idString + "  " + frameData;
    
    tft.drawString(line, 5, y); 
    y += 10;
  }
}

void drawSetupScreen() {
  tft.fillScreen(TFT_BLACK);

  drawButton(10, tft.height() - 40, 80, 30, "Voltar", TFT_RED, TFT_WHITE);
  drawButton(tft.width() - 90, tft.height() - 40, 80, 30, "Salvar", TFT_GREEN, TFT_WHITE);

  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("Velocidade CAN:", 10, 40);
  
  tft.drawRect(10, 70, 80, 30, (canSpeed == 125000) ? TFT_GREEN : TFT_WHITE);
  tft.drawString("125k", 40, 80); // <-- Ajustado 10px para a esquerda, 5px para cima
  tft.drawRect(100, 70, 80, 30, (canSpeed == 250000) ? TFT_GREEN : TFT_WHITE);
  tft.drawString("250k", 130, 80); // <-- Ajustado 10px para a esquerda, 5px para cima
  tft.drawRect(190, 70, 80, 30, (canSpeed == 500000) ? TFT_GREEN : TFT_WHITE);
  tft.drawString("500k", 220, 80); // <-- Ajustado 10px para a esquerda, 5px para cima

  tft.drawString("Tipo de ID:", 10, 120);
  tft.drawRect(10, 150, 100, 30, (!isExtendedID) ? TFT_GREEN : TFT_WHITE);
  tft.drawString("Std", 50, 160); // <-- Ajustado 10px para a esquerda, 5px para cima
  tft.drawRect(120, 150, 100, 30, (isExtendedID) ? TFT_GREEN : TFT_WHITE);
  tft.drawString("Ext", 160, 160); // <-- Ajustado 10px para a esquerda, 5px para cima
}

void drawSendScreen() {
  tft.fillScreen(TFT_BLACK);

  drawButton(10, tft.height() - 40, 80, 30, "Voltar", TFT_RED, TFT_WHITE);
  drawButton(tft.width() - 90, tft.height() - 40, 80, 30, "Enviar", TFT_GREEN, TFT_WHITE);

  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE);
  tft.setTextDatum(TL_DATUM);
  
  tft.drawString("ID:", 10, 40);
  tft.drawString("Frame:", 10, 80);
  tft.drawString("Intervalo (ms):", 10, 120);
}

// --- Funções de Lógica ---
void handleTouch() {
  if (ts.touched()) {
    TS_Point p = ts.getPoint();
    uint16_t touch_x = map(p.x, touchMinX, touchMaxX, 0, tft.width());
    uint16_t touch_y = map(p.y, touchMinY, touchMaxY, 0, tft.height());

    Serial.print("Touch Mapped: X=");
    Serial.print(touch_x);
    Serial.print(", Y=");
    Serial.println(touch_y);

    if (currentScreen == MAIN_SCREEN) {
      // Botão SEND
      if (touch_x > 10 && touch_x < 90 && touch_y > tft.height() - 40 && touch_y < tft.height() - 10) {
        currentScreen = SEND_SCREEN;
        drawSendScreen();
      }
      // Botão SETUP
      if (touch_x > tft.width() - 90 && touch_x < tft.width() - 10 && touch_y > tft.height() - 40 && touch_y < tft.height() - 10) {
        currentScreen = SETUP_SCREEN;
        drawSetupScreen();
      }
    } else if (currentScreen == SETUP_SCREEN) {
      if (touch_x > 10 && touch_x < 90 && touch_y > tft.height() - 40 && touch_y < tft.height() - 10) {
        currentScreen = MAIN_SCREEN;
        drawMainScreen();
      } else if (touch_x > tft.width() - 90 && touch_x < tft.width() - 10 && touch_y > tft.height() - 40 && touch_y < tft.height() - 10) {
        saveConfig();
        currentScreen = MAIN_SCREEN;
        drawMainScreen();
      } else if (touch_x > 10 && touch_x < 90 && touch_y > 70 && touch_y < 100) {
        canSpeed = 125000;
        drawSetupScreen();
      } else if (touch_x > 100 && touch_x < 180 && touch_y > 70 && touch_y < 100) {
        canSpeed = 250000;
        drawSetupScreen();
      } else if (touch_x > 190 && touch_x < 270 && touch_y > 70 && touch_y < 100) {
        canSpeed = 500000;
        drawSetupScreen();
      } else if (touch_x > 10 && touch_x < 110 && touch_y > 150 && touch_y < 180) {
        isExtendedID = false;
        drawSetupScreen();
      } else if (touch_x > 120 && touch_x < 220 && touch_y > 150 && touch_y < 180) {
        isExtendedID = true;
        drawSetupScreen();
      }
    } else if (currentScreen == SEND_SCREEN) {
      // Botão Voltar
      if (touch_x > 10 && touch_x < 90 && touch_y > tft.height() - 40 && touch_y < tft.height() - 10) {
        currentScreen = MAIN_SCREEN;
        drawMainScreen();
      }
      // Botão Enviar
      if (touch_x > tft.width() - 90 && touch_x < tft.width() - 10 && touch_y > tft.height() - 40 && touch_y < tft.height() - 10) {
        // Implementar a lógica de envio aqui
        Serial.println("Enviar mensagem CAN acionado!");
        currentScreen = MAIN_SCREEN;
        drawMainScreen();
      }
    }
    while (ts.touched()) {
      delay(10);
    }
  }
}

void saveConfig() {
  preferences.begin("can-config", false);
  preferences.putLong("canSpeed", canSpeed);
  preferences.putBool("isExtendedID", isExtendedID);
  preferences.end();
  Serial.println("Configuracoes salvas.");
}

void loadConfig() {
  preferences.begin("can-config", true);
  canSpeed = preferences.getLong("canSpeed", 250000);
  isExtendedID = preferences.getBool("isExtendedID", false);
  preferences.end();
  Serial.println("Configuracoes carregadas.");
}

void drawButton(int x, int y, int w, int h, const char* label, uint16_t bgColor, uint16_t borderColor) {
  tft.fillRect(x, y, w, h, bgColor);
  tft.drawRect(x, y, w, h, borderColor);
  tft.setTextColor(TFT_WHITE);
  tft.setTextDatum(MC_DATUM); 
  tft.drawString(label, x + w / 2, y + h / 2);
}
