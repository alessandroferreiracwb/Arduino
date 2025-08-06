#include <SPI.h>
#include <TFT_eSPI.h>
#include <Preferences.h>
#include <XPT2046_Touchscreen.h>
#include <WiFi.h>
#include <SD.h>

// --- Credenciais e Configurações de Conexão Wi-Fi ---
const char* ssid = "ESP_CAN";
const char* password = "admin123";
const char* serverIp = "192.168.4.1";
const uint16_t port = 8080;

WiFiClient client;
bool isConnected = false;
unsigned long lastConnectionAttempt = 0;
const long connectionInterval = 5000;

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

// Pinos do cartão SD, conforme a pinagem na imagem
#define SD_CS 5
#define SD_MOSI 23
#define SD_MISO 19
#define SD_SCLK 18

// Valores de calibração do seu touch
int touchMinX = 451;
int touchMaxX = 3598;
int touchMinY = 600;
int touchMaxY = 3579;
// -----------------------------------------------------------

TFT_eSPI tft = TFT_eSPI();
SPIClass touchscreenSPI = SPIClass(VSPI);
XPT2046_Touchscreen ts(XPT2046_CS, XPT2046_IRQ);
SPIClass sdSPI;

// Variáveis de estado da interface
enum Screen { MAIN_SCREEN, SETUP_SCREEN, SEND_SCREEN, CONNECTING_SCREEN };
Screen currentScreen = CONNECTING_SCREEN;

// Variáveis para as configurações do CAN bus
Preferences preferences;
long canSpeed = 250000;
bool isExtendedID = false;

// Variáveis para a tela de envio
String sendId = "18F00001";
String sendFrame[8] = {"FF", "FF", "FF", "FF", "FF", "FF", "FF", "FF"};
String sendInterval = "100";

// Variável para rastrear o campo de entrada ativo
enum ActiveInput { NONE, ID, FRAME_BYTE, TEMPO };
ActiveInput activeInput = NONE;
int activeFrameByte = -1;

// Estrutura para as teclas do teclado virtual
struct Key {
    int x, y, w, h;
    char label[4];
};

Key keys[18] = {
    {10, 100, 30, 20, "7"}, {50, 100, 30, 20, "8"}, {90, 100, 30, 20, "9"}, {130, 100, 30, 20, "A"}, {170, 100, 30, 20, "B"},
    {10, 125, 30, 20, "4"}, {50, 125, 30, 20, "5"}, {90, 125, 30, 20, "6"}, {130, 125, 30, 20, "C"}, {170, 125, 30, 20, "D"},
    {10, 150, 30, 20, "1"}, {50, 150, 30, 20, "2"}, {90, 150, 30, 20, "3"}, {130, 150, 30, 20, "E"}, {170, 150, 30, 20, "F"},
    {10, 175, 30, 20, "0"}, {50, 175, 30, 20, "Bk"}, {90, 175, 30, 20, "En"}
};

// --- Variáveis de mensagens globais ---
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
void drawConnectingScreen();
void handleTouch();
void saveConfig();
void loadConfig();
void setupWifi();
void checkWifiConnection();
void sendCanFrameWifi();
void processWifiData(String data);
void drawButton(int x, int y, int w, int h, const char* label, uint16_t bgColor, uint16_t borderColor, uint8_t textSize = 2);
int findMessageIndex(unsigned long id);
void drawSingleCanMessage(int index, int yOffset, const unsigned char* oldData = nullptr);
void clearCanMessages();
void sendLedCommand(bool state);
void sendCanSpeedCommand(long speed);
void setupSDCard();

// MODIFICAÇÃO: Protótipo da nova função para salvar no cartão SD
void logToSDCard(String id, String frame);

// --- Função de Configuração Inicial (setup) ---
void setup() {
    Serial.begin(115200);

    // Inicialização dos barramentos SPI para evitar conflitos
    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    
    touchscreenSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
    ts.begin(touchscreenSPI);
    ts.setRotation(1);

    sdSPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);

    loadConfig();
    setupWifi();
    drawConnectingScreen();

    // Chama a função para testar e inicializar o cartão SD
    setupSDCard();
}

// --- Função de Loop Principal (loop) ---
void loop() {
    checkWifiConnection();
    if (isConnected) {
        handleTouch();
        if (currentScreen == MAIN_SCREEN && client.connected() && client.available()) {
            String data = client.readStringUntil('\n');
            processWifiData(data);
        }
    }
}

// --- Funções de Conexão Wi-Fi ---
void setupWifi() {
    Serial.print("Conectando a ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
}

void checkWifiConnection() {
    if (isConnected) {
        if (!client.connected()) {
            Serial.println("Conexao com o servidor perdida.");
            isConnected = false;
            currentScreen = CONNECTING_SCREEN;
            drawConnectingScreen();
        }
        return;
    }

    if (currentScreen != CONNECTING_SCREEN) {
        currentScreen = CONNECTING_SCREEN;
        drawConnectingScreen();
    }

    if (millis() - lastConnectionAttempt < connectionInterval) {
        return;
    }
    lastConnectionAttempt = millis();

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi desconectado. Tentando reconectar...");
        tft.fillScreen(TFT_BLACK);
        tft.setTextSize(2);
        tft.setTextColor(TFT_WHITE);
        tft.setTextDatum(MC_DATUM);
        tft.drawString("TENTANDO CONECTAR...", tft.width() / 2, tft.height() / 2);
        WiFi.begin(ssid, password);
        return;
    }

    if (!client.connected()) {
        Serial.print("WiFi conectado. Tentando conectar ao servidor ");
        if (client.connect(serverIp, port)) {
            Serial.println("Conectado ao servidor!");
            isConnected = true;
            currentScreen = MAIN_SCREEN;
            messageCount = 0;
            drawMainScreen();
        } else {
            Serial.println("Conexão com o servidor falhou.");
        }
    }
}

void drawConnectingScreen() {
    tft.fillScreen(TFT_BLACK);
    tft.setTextSize(2);
    tft.setTextColor(TFT_RED);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("CONEXAO PERDIDA", tft.width() / 2, tft.height() / 2 - 20);
    tft.setTextColor(TFT_WHITE);
    tft.drawString("TENTANDO RECONECTAR...", tft.width() / 2, tft.height() / 2 + 10);
}

// --- Funções de Lógica e Desenho da Interface ---
void processWifiData(String data) {
    data.trim();
    int spaceIndex = data.indexOf(' ');
    if (spaceIndex == -1) {
        return;
    }
    
    String idString = data.substring(0, spaceIndex);
    String dataString = data.substring(spaceIndex + 1);

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
    
    // MODIFICAÇÃO: Chama a nova função para salvar a mensagem no cartão SD
    logToSDCard(idString, dataString);

    int messageIndex = findMessageIndex(id);

    if (messageIndex != -1) {
        bool updated = false;
        unsigned char oldData[8];
        memcpy(oldData, canMessages[messageIndex].data, canMessages[messageIndex].len);

        for(int i = 0; i < len; i++) {
            if (canMessages[messageIndex].data[i] != buf[i]) {
                canMessages[messageIndex].data[i] = buf[i];
                updated = true;
            }
        }
        canMessages[messageIndex].len = len;
        if (updated) {
            drawSingleCanMessage(messageIndex, messageIndex * 10 + 5, oldData);
        }
    } else {
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
}

int findMessageIndex(unsigned long id) {
    for (int i = 0; i < messageCount; i++) {
        if (canMessages[i].id == id) {
            return i;
        }
    }
    return -1;
}

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
    drawButton(10, tft.height() - 40, 80, 30, "SEND", TFT_GREEN, TFT_WHITE, 2);
    drawButton(tft.width() - 200, tft.height() - 40, 80, 30, "LIMPA", TFT_YELLOW, TFT_WHITE, 2);
    drawButton(tft.width() - 90, tft.height() - 40, 80, 30, "SETUP", TFT_BLUE, TFT_WHITE, 2);
}

void drawCanMessages() {
    tft.fillRect(0, 0, tft.width(), tft.height() - 60, TFT_BLACK);
    for (int i = 0; i < messageCount; i++) {
        drawSingleCanMessage(i, i * 10 + 5);
    }
}

void drawSingleCanMessage(int index, int yOffset, const unsigned char* oldData) {
    tft.setTextDatum(TL_DATUM);
    tft.setTextSize(1);
    uint16_t idColor = (index % 2 == 0) ? TFT_LIGHTGREY : TFT_CYAN;
    tft.setTextColor(idColor);

    String idString = String(canMessages[index].id, HEX);
    idString.toUpperCase();

    tft.fillRect(0, yOffset, tft.width(), 10, TFT_BLACK);
    tft.drawString(idString, 5, yOffset);

    int xPos = 90;
    for (int j = 0; j < canMessages[index].len; j++) {
        if (oldData != nullptr && canMessages[index].data[j] != oldData[j]) {
            tft.setTextColor(TFT_RED);
        } else {
            tft.setTextColor(idColor);
        }

        String byteString = "";
        if (canMessages[index].data[j] < 16) byteString += "0";
        byteString += String(canMessages[index].data[j], HEX);
        byteString.toUpperCase();
        tft.drawString(byteString, xPos, yOffset);
        xPos += 15;

        if (j < canMessages[index].len - 1) {
            tft.setTextColor(idColor);
            tft.drawString(":", xPos, yOffset);
            xPos += 5;
        }
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
    tft.drawString("Std", 50, 160);
    tft.drawRect(120, 150, 100, 30, (isExtendedID) ? TFT_GREEN : TFT_WHITE);
    tft.drawString("Ext", 160, 160);
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
    uint16_t idColor = (activeInput == ID) ? TFT_GREEN : TFT_WHITE;
    tft.drawRect(50, 5, 120, 30, idColor);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(sendId, 110, 20);
    tft.setTextDatum(TL_DATUM);
    tft.drawString("Frame:", 10, 50);
    int xPos = 80;
    for (int i = 0; i < 8; i++) {
        uint16_t color = (activeInput == FRAME_BYTE && activeFrameByte == i) ? TFT_GREEN : TFT_WHITE;
        tft.drawRect(xPos, 45, 30, 30, color);
        tft.setTextDatum(MC_DATUM);
        tft.drawString(sendFrame[i], xPos + 15, 60);
        xPos += 35;
    }
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

void handleTouch() {
    if (ts.touched()) {
        TS_Point p = ts.getPoint();
        uint16_t touch_x = map(p.x, touchMinX, touchMaxX, 0, tft.width());
        uint16_t touch_y = map(p.y, touchMinY, touchMaxY, 0, tft.height());

        if (isConnected) {
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
                if (touch_x > tft.width() / 2 - 40 && touch_x < tft.width() / 2 + 40 && touch_y > tft.height() - 40 && touch_y < tft.height() - 10) {
                    clearCanMessages();
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
                    sendCanSpeedCommand(canSpeed);
                    drawSetupScreen();
                } else if (touch_x > 100 && touch_x < 180 && touch_y > 70 && touch_y < 100) {
                    canSpeed = 250000;
                    sendCanSpeedCommand(canSpeed);
                    drawSetupScreen();
                } else if (touch_x > 190 && touch_x < 270 && touch_y > 70 && touch_y < 100) {
                    canSpeed = 500000;
                    sendCanSpeedCommand(canSpeed);
                    drawSetupScreen();
                } else if (touch_x > 10 && touch_x < 110 && touch_y > 150 && touch_y < 180) {
                    isExtendedID = false;
                    drawSetupScreen();
                } else if (touch_x > 120 && touch_x < 220 && touch_y > 150 && touch_y < 180) {
                    isExtendedID = true;
                    drawSetupScreen();
                    sendLedCommand(true);
                }
            } else if (currentScreen == SEND_SCREEN) {
                if (touch_x > 10 && touch_x < 90 && touch_y > tft.height() - 40 && touch_y < tft.height() - 10) {
                    currentScreen = MAIN_SCREEN;
                    activeInput = NONE;
                    drawMainScreen();
                }
                if (touch_x > tft.width() - 90 && touch_x < tft.width() - 10 && touch_y > tft.height() - 40 && touch_y < tft.height() - 10) {
                    sendCanFrameWifi();
                    currentScreen = MAIN_SCREEN;
                    activeInput = NONE;
                    drawMainScreen();
                }
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
                if (touch_x > 210 && touch_x < 290 && touch_y > 125 && touch_y < 155) {
                    activeInput = TEMPO;
                    drawSendScreen();
                }
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
        } else {
            // Ignora toques se não estiver conectado
        }
    }
}

void sendCanFrameWifi() {
    String message = sendId + " ";
    for (int i = 0; i < 8; i++) {
        message += sendFrame[i];
    }
    message += " " + sendInterval;

    if (client.connected()) {
        client.println(message);
        Serial.print("Mensagem enviada via WiFi: ");
        Serial.println(message);
    } else {
        Serial.println("Wi-Fi desconectado, não foi possível enviar a mensagem.");
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

void drawButton(int x, int y, int w, int h, const char* label, uint16_t bgColor, uint16_t borderColor, uint8_t textSize) {
    tft.fillRect(x, y, w, h, bgColor);
    tft.drawRect(x, y, w, h, borderColor);
    tft.setTextColor(TFT_WHITE);
    tft.setTextDatum(MC_DATUM);
    tft.setTextSize(textSize);
    tft.drawString(label, x + w / 2, y + h / 2);
}

void clearCanMessages() {
    messageCount = 0;
    drawMainScreen();
}

void sendLedCommand(bool state) {
    if (client.connected()) {
        String command = "LED_D2 ";
        command += (state ? "ON" : "OFF");
        client.println(command);
        Serial.print("Comando enviado para o LED D2: ");
        Serial.println(command);
    } else {
        Serial.println("Wi-Fi desconectado, não foi possível enviar o comando para o LED.");
    }
}

void sendCanSpeedCommand(long speed) {
    if (client.connected()) {
        String command = "CAN_SPEED " + String(speed);
        client.println(command);
        Serial.print("Comando de velocidade CAN enviado: ");
        Serial.println(command);
    } else {
        Serial.println("Wi-Fi desconectado, não foi possível enviar o comando de velocidade.");
    }
}

// Função para inicializar o cartão SD e listar os arquivos
void setupSDCard() {
    Serial.println("Inicializando o Cartão SD...");
    // Tenta inicializar o cartão SD
    if (!SD.begin(SD_CS, sdSPI)) {
        Serial.println("Falha na inicialização do Cartão SD!");
        Serial.println("Verifique se o cartão está formatado em FAT32 e inserido corretamente.");
        return;
    }
    Serial.println("Cartão SD inicializado com sucesso!");
    
    // Imprime o tipo de cartão
    uint8_t cardType = SD.cardType();
    if (cardType == CARD_MMC) {
        Serial.println("Tipo de Cartão: MMC");
    } else if (cardType == CARD_SD) {
        Serial.println("Tipo de Cartão: SDSC");
    } else if (cardType == CARD_SDHC) {
        Serial.println("Tipo de Cartão: SDHC");
    } else {
        Serial.println("Tipo de Cartão: Desconhecido");
    }
    
    // Lista os arquivos na raiz do cartão para verificação
    File root = SD.open("/");
    if (root) {
        Serial.println("Arquivos na raiz:");
        while(File file = root.openNextFile()){
            if(file){
                Serial.print("  - ");
                Serial.print(file.name());
                if (file.isDirectory()) {
                    Serial.println(" <DIR>");
                } else {
                    Serial.print("\tTamanho: ");
                    Serial.println(file.size());
                }
            }
        }
        root.close();
    }
}

// MODIFICAÇÃO: Nova função para salvar o log no cartão SD
void logToSDCard(String id, String frame) {
    // Tenta abrir o arquivo "log.txt" no modo de anexação (append)
    File logFile = SD.open("/log.txt", FILE_APPEND);
    if (!logFile) {
        Serial.println("Erro ao abrir log.txt");
        return;
    }

    // Monta a linha de log com ID e frame e a escreve no arquivo
    String logEntry = "ID: " + id + " | Frame: " + frame;
    logFile.println(logEntry);
    
    // Fecha o arquivo e imprime uma confirmação no Serial Monitor
    logFile.close();
    Serial.println("Log gravado no cartao SD.");
}
