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
String sendId = "18FEEF00";
String sendFrame[8] = {"10", "20", "30", "40", "50", "60", "70", "80"};
String sendInterval = "1000";

// Variável para rastrear o campo de entrada ativo
enum ActiveInput { NONE, ID, FRAME_BYTE, TEMPO };
ActiveInput activeInput = NONE;
int activeFrameByte = -1; // 0-7 para o byte do frame

// Estrutura para as teclas do teclado virtual
struct Key {
    int x, y, w, h;
    char label[4];
};

// Layout do teclado hexadecimal redimensionado e movido
Key keys[18] = {
    {10, 100, 30, 20, "7"}, {50, 100, 30, 20, "8"}, {90, 100, 30, 20, "9"}, {130, 100, 30, 20, "A"}, {170, 100, 30, 20, "B"},
    {10, 125, 30, 20, "4"}, {50, 125, 30, 20, "5"}, {90, 125, 30, 20, "6"}, {130, 125, 30, 20, "C"}, {170, 125, 30, 20, "D"},
    {10, 150, 30, 20, "1"}, {50, 150, 30, 20, "2"}, {90, 150, 30, 20, "3"}, {130, 150, 30, 20, "E"}, {170, 150, 30, 20, "F"},
    {10, 175, 30, 20, "0"}, {50, 175, 30, 20, "Bk"}, {90, 175, 30, 20, "En"}
};


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
void sendCanFrameSerial();
void drawButton(int x, int y, int w, int h, const char* label, uint16_t bgColor, uint16_t borderColor, uint8_t textSize = 2);

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
  
  // Dados simulados para preencher a tela principal
  canMessages[0] = {0x18FEEF00, {0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80}, 8, true};
  canMessages[1] = {0x123, {0xAA, 0xBB, 0xCC, 0xDD}, 4, false};
  canMessages[2] = {0x456, {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08}, 8, false};
  canMessages[3] = {0x18FEF000, {0x01}, 1, true};
  canMessages[4] = {0x789, {0x11, 0x22, 0x33, 0x44, 0x55}, 5, false};
  canMessages[5] = {0x1000, {0xEE, 0xFF, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06}, 8, false};
  canMessages[6] = {0x18FEEB00, {0x1A, 0x2B, 0x3C, 0x4D, 0x5E, 0x6F}, 6, true};
  canMessages[7] = {0x5A5A, {0x11, 0x22, 0x33, 0x44}, 4, false};
  canMessages[8] = {0x12345678, {0x00, 0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70}, 8, true};
  canMessages[9] = {0x100, {0x0A, 0x0B, 0x0C, 0x0D}, 4, false};
  canMessages[10] = {0x18FF00, {0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18}, 8, true};
  canMessages[11] = {0x156, {0x01, 0x02, 0x03}, 3, false};
  canMessages[12] = {0x12345, {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF}, 6, false};
  canMessages[13] = {0x18FFAA00, {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88}, 8, true};
  canMessages[14] = {0x7AB, {0x11}, 1, false};
  canMessages[15] = {0x18FFBB00, {0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27}, 8, true};
  messageCount = 16;
  
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
  
  drawButton(10, tft.height() - 40, 80, 30, "SEND", TFT_RED, TFT_WHITE, 2);
  drawButton(tft.width() - 90, tft.height() - 40, 80, 30, "SETUP", TFT_BLUE, TFT_WHITE, 2);
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
    
    // Desenha o ID
    tft.drawString(idString, 5, y); 
    
    // Desenha o frame de dados em uma coordenada X fixa para alinhamento
    tft.drawString(frameData, 90, y); 
    y += 10;
  }
}

void drawSetupScreen() {
  tft.fillScreen(TFT_BLACK);

  drawButton(10, tft.height() - 40, 80, 30, "Voltar", TFT_RED, TFT_WHITE, 2);
  drawButton(tft.width() - 90, tft.height() - 40, 80, 30, "Salvar", TFT_GREEN, TFT_WHITE, 2);

  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("Velocidade CAN:", 10, 40);
  
  tft.drawRect(10, 70, 80, 30, (canSpeed == 125000) ? TFT_GREEN : TFT_WHITE);
  tft.drawString("125k", 40, 80);
  tft.drawRect(100, 70, 80, 30, (canSpeed == 250000) ? TFT_GREEN : TFT_WHITE);
  tft.drawString("250k", 130, 80);
  tft.drawRect(190, 70, 80, 30, (canSpeed == 500000) ? TFT_GREEN : TFT_WHITE);
  tft.drawString("500k", 220, 80);

  tft.drawString("Tipo de ID:", 10, 120);
  tft.drawRect(10, 150, 100, 30, (!isExtendedID) ? TFT_GREEN : TFT_WHITE);
  tft.drawString("Standard", 60, 165);
  tft.drawRect(120, 150, 100, 30, (isExtendedID) ? TFT_GREEN : TFT_WHITE);
  tft.drawString("Extended", 170, 165);
}

void drawKeyboard() {
  for (int i = 0; i < 18; i++) {
    drawButton(keys[i].x, keys[i].y, keys[i].w, keys[i].h, keys[i].label, TFT_DARKGREY, TFT_WHITE, 1);
  }
}

void drawSendScreen() {
  tft.fillScreen(TFT_BLACK);
  
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE);
  tft.setTextDatum(TL_DATUM);
  
  tft.drawString("ID:", 10, 10);
  // Caixa de texto para o ID
  uint16_t idColor = (activeInput == ID) ? TFT_GREEN : TFT_WHITE;
  tft.drawRect(50, 5, 120, 30, idColor);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(sendId, 110, 20);

  tft.setTextDatum(TL_DATUM);
  tft.drawString("Frame:", 10, 50);
  // Caixas de texto para os 8 bytes do frame
  int xPos = 80;
  for (int i = 0; i < 8; i++) {
    uint16_t color = (activeInput == FRAME_BYTE && activeFrameByte == i) ? TFT_GREEN : TFT_WHITE;
    tft.drawRect(xPos, 45, 30, 30, color);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(sendFrame[i], xPos + 15, 60);
    xPos += 35;
  }
  
  // Novo campo de "Tempo"
  tft.setTextSize(2);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("Tempo:", 210, 100);
  uint16_t tempoColor = (activeInput == TEMPO) ? TFT_GREEN : TFT_WHITE;
  tft.drawRect(210, 125, 80, 30, tempoColor);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(sendInterval, 250, 140);
  
  drawButton(10, tft.height() - 40, 80, 30, "Voltar", TFT_RED, TFT_WHITE, 2);
  drawButton(tft.width() - 90, tft.height() - 40, 80, 30, "Enviar", TFT_GREEN, TFT_WHITE, 2);

  drawKeyboard();
}

// --- Funções de Lógica ---
void handleTouch() {
  if (ts.touched()) {
    TS_Point p = ts.getPoint();
    uint16_t touch_x = map(p.x, touchMinX, touchMaxX, 0, tft.width());
    uint16_t touch_y = map(p.y, touchMinY, touchMaxY, 0, tft.height());

    if (currentScreen == MAIN_SCREEN) {
      if (touch_x > 10 && touch_x < 90 && touch_y > tft.height() - 40 && touch_y < tft.height() - 10) {
        currentScreen = SEND_SCREEN;
        activeInput = NONE;
        drawSendScreen();
      }
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
        activeInput = NONE;
        drawMainScreen();
      }
      // Botão Enviar
      if (touch_x > tft.width() - 90 && touch_x < tft.width() - 10 && touch_y > tft.height() - 40 && touch_y < tft.height() - 10) {
        sendCanFrameSerial(); 
        currentScreen = MAIN_SCREEN;
        activeInput = NONE;
        drawMainScreen();
      }
      // Detecção de toque nos campos de entrada
      if (touch_x > 50 && touch_x < 250 && touch_y > 5 && touch_y < 35) {
        activeInput = ID;
        drawSendScreen();
      }
      int xPos = 80;
      for (int i = 0; i < 8; i++) {
        if (touch_x > xPos && touch_x < xPos + 30 && touch_y > 45 && touch_y < 75) {
          activeInput = FRAME_BYTE;
          activeFrameByte = i;
          drawSendScreen();
          break;
        }
        xPos += 35;
      }
      // Detecção de toque no campo TEMPO
      if (touch_x > 210 && touch_x < 290 && touch_y > 125 && touch_y < 155) {
        activeInput = TEMPO;
        drawSendScreen();
      }
      // Detecção de toque no teclado
      for (int i = 0; i < 18; i++) {
        if (touch_x > keys[i].x && touch_x < keys[i].x + keys[i].w &&
            touch_y > keys[i].y && touch_y < keys[i].y + keys[i].h) {
          char keyLabel[4];
          strcpy(keyLabel, keys[i].label);
          if (strcmp(keyLabel, "Bk") == 0) {
            if (activeInput == ID && sendId.length() > 0) sendId.remove(sendId.length() - 1);
            else if (activeInput == FRAME_BYTE && sendFrame[activeFrameByte].length() > 0) sendFrame[activeFrameByte].remove(sendFrame[activeFrameByte].length() - 1);
            else if (activeInput == TEMPO && sendInterval.length() > 0) sendInterval.remove(sendInterval.length() - 1);
          } else if (strcmp(keyLabel, "En") == 0) {
            activeInput = NONE;
          } else {
            if (activeInput == ID) {
              if (sendId.length() < 8) sendId += keyLabel;
            } else if (activeInput == FRAME_BYTE) {
              if (sendFrame[activeFrameByte].length() < 2) sendFrame[activeFrameByte] += keyLabel;
            } else if (activeInput == TEMPO) {
              if (sendInterval.length() < 5) sendInterval += keyLabel;
            }
          }
          drawSendScreen();
          break;
        }
      }
    }
    while (ts.touched()) {
      delay(10);
    }
  }
}

// Nova função para enviar o frame CAN pela serial
void sendCanFrameSerial() {
    // Converte a string de ID para unsigned long
    unsigned long id = strtoul(sendId.c_str(), NULL, 16);
    
    // Converte as strings de frame para bytes
    unsigned char frameBytes[8];
    for (int i = 0; i < 8; i++) {
        frameBytes[i] = strtoul(sendFrame[i].c_str(), NULL, 16);
    }
    
    // Envia o ID (4 bytes) e os dados do frame (8 bytes) pela serial
    Serial.write((uint8_t*)&id, sizeof(id));
    Serial.write(frameBytes, 8);
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

void drawButton(int x, int y, int w, int h, const char* label, uint16_t bgColor, uint16_t borderColor, uint8_t textSize) {
  tft.fillRect(x, y, w, h, bgColor);
  tft.drawRect(x, y, w, h, borderColor);
  tft.setTextColor(TFT_WHITE);
  tft.setTextDatum(MC_DATUM); 
  tft.setTextSize(textSize);
  tft.drawString(label, x + w / 2, y + h / 2);
}
