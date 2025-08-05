#include <SPI.h>            // Biblioteca para comunicação SPI
#include <TFT_eSPI.h>       // Biblioteca do driver da tela TFT
#include <Preferences.h>    // Biblioteca para salvar dados na memória flash
#include <XPT2046_Touchscreen.h> // Biblioteca do driver do touchscreen
#include <WiFi.h>           // Biblioteca para conexão Wi-Fi

// --- Credenciais e Configurações de Conexão Wi-Fi ---
const char* ssid = "ESP_CAN";             // SSID da rede Wi-Fi
const char* password = "admin123";        // Senha da rede Wi-Fi
const char* serverIp = "192.168.4.1";     // Endereço IP do servidor (outro ESP32)
const uint16_t port = 8080;               // Porta de comunicação do servidor

WiFiClient client;                        // Objeto para gerenciar a conexão com o servidor
bool isConnected = false;                 // Flag que indica se o cliente está conectado
unsigned long lastConnectionAttempt = 0;  // Variável para controle do tempo de reconexão
const long connectionInterval = 5000;     // Intervalo de 5 segundos para tentar reconectar

// --- Definições de Pinos e Calibração do Hardware ---
#define TFT_CS 15      // Pino Chip Select (CS) para a tela TFT
#define TFT_DC 2       // Pino Data/Command (DC) para a tela TFT
#define TFT_RST -1     // Pino de Reset (RST) para a tela (-1 = não usado)
#define TFT_MOSI 13    // Pino MOSI da SPI para a tela
#define TFT_SCLK 14    // Pino SCLK da SPI para a tela
#define TFT_MISO 12    // Pino MISO da SPI para a tela

// Pinos SPI do touchscreen
#define XPT2046_MOSI 32
#define XPT2046_MISO 39
#define XPT2046_CLK 25
#define XPT2046_CS 33
#define XPT2046_IRQ 36

// Valores de calibração do touch (específicos para seu hardware)
int touchMinX = 451;
int touchMaxX = 3598;
int touchMinY = 600;
int touchMaxY = 3579;

TFT_eSPI tft = TFT_eSPI();                         // Cria objeto da tela
SPIClass touchscreenSPI = SPIClass(VSPI);          // Cria objeto SPI para o touch
XPT2046_Touchscreen ts(XPT2046_CS, XPT2046_IRQ);   // Cria objeto do touchscreen

// Variáveis de estado da interface
enum Screen { MAIN_SCREEN, SETUP_SCREEN, SEND_SCREEN, CONNECTING_SCREEN };
Screen currentScreen = CONNECTING_SCREEN;

// Variáveis para as configurações do CAN bus
Preferences preferences;          // Objeto para salvar/carregar configurações
long canSpeed = 250000;           // Velocidade padrão do CAN
bool isExtendedID = false;        // Tipo de ID padrão

// Variáveis para a tela de envio
String sendId = "18F00001";              // ID inicial para envio
String sendFrame[8] = {"FF", "FF", "FF", "FF", "FF", "FF", "FF", "FF"}; // Bytes iniciais do frame
String sendInterval = "100";             // Intervalo de envio padrão

// Variáveis para rastrear o campo de entrada ativo
enum ActiveInput { NONE, ID, FRAME_BYTE, TEMPO };
ActiveInput activeInput = NONE;
int activeFrameByte = -1; // Índice do byte do frame sendo editado

// Estrutura para as teclas do teclado virtual
struct Key {
    int x, y, w, h;
    char label[4];
};

// Definição das posições e rótulos das teclas do teclado virtual
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
CanMessage canMessages[20]; // Array para armazenar as últimas 20 mensagens CAN
int messageCount = 0;       // Contador de mensagens na lista
int scrollPosition = 0;     // Posição de rolagem (não usada neste código)

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

// --- Função de Configuração Inicial (setup) ---
// Chamada apenas uma vez no início do programa
void setup() {
    Serial.begin(115200);   // Inicia comunicação serial para debug
    SPI.begin();            // Inicia o bus SPI para a tela

    tft.init();             // Inicializa a tela TFT
    tft.setRotation(1);     // Define a rotação da tela
    tft.fillScreen(TFT_BLACK); // Preenche a tela com preto

    touchscreenSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS); // Inicia o bus SPI para o touch
    ts.begin(touchscreenSPI); // Inicializa o touchscreen
    ts.setRotation(1);        // Define a rotação do touch

    loadConfig();           // Carrega as configurações salvas na memória flash
    setupWifi();            // Inicia a conexão Wi-Fi

    drawConnectingScreen(); // Desenha a tela de conexão inicial
}

// --- Função de Loop Principal (loop) ---
// Executa repetidamente enquanto o ESP32 estiver ligado
void loop() {
    checkWifiConnection();  // Verifica e gerencia a conexão Wi-Fi

    if (isConnected) {
        handleTouch(); // Lida com toques na tela
        
        // Se estiver na tela principal e houver dados na conexão, processa a mensagem
        if (currentScreen == MAIN_SCREEN && client.connected() && client.available()) {
            String data = client.readStringUntil('\n'); // Lê a mensagem recebida
            processWifiData(data); // Processa os dados
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
            // Se a conexão com o servidor for perdida, redefine o estado para 'conectando'
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
        // Tenta reconectar o Wi-Fi se estiver desconectado
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
        // Tenta conectar ao servidor se o Wi-Fi estiver conectado mas o cliente não
        Serial.print("WiFi conectado. Tentando conectar ao servidor ");
        if (client.connect(serverIp, port)) {
            // Conexão bem-sucedida
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

// Processa os dados recebidos via Wi-Fi (mensagens CAN)
void processWifiData(String data) {
    data.trim(); // Remove espaços em branco
    int spaceIndex = data.indexOf(' ');
    if (spaceIndex == -1) {
        return;
    }
    
    // Extrai o ID e os dados do frame da mensagem
    String idString = data.substring(0, spaceIndex);
    String dataString = data.substring(spaceIndex + 1);

    unsigned long id = strtoul(idString.c_str(), NULL, 16); // Converte ID para hexadecimal
    int len = dataString.length() / 2; // Calcula o tamanho do frame
    unsigned char buf[8];
    
    if (len > 8) {
        len = 8;
    }
    
    for(int i = 0; i < len; i++) {
        // Converte cada byte da string para um número hexadecimal
        String byteString = dataString.substring(i * 2, i * 2 + 2);
        buf[i] = strtoul(byteString.c_str(), NULL, 16);
    }

    int messageIndex = findMessageIndex(id);

    if (messageIndex != -1) {
        // Se o ID já existe, atualiza apenas os bytes que mudaram
        bool updated = false;
        unsigned char oldData[8];
        memcpy(oldData, canMessages[messageIndex].data, canMessages[messageIndex].len); // Salva os dados antigos

        for(int i = 0; i < len; i++) {
            if (canMessages[messageIndex].data[i] != buf[i]) {
                canMessages[messageIndex].data[i] = buf[i];
                updated = true;
            }
        }
        canMessages[messageIndex].len = len;
        if (updated) {
            // Se houve atualização, redesenha a linha destacando as mudanças
            drawSingleCanMessage(messageIndex, messageIndex * 10 + 5, oldData);
        }
    } else {
        // Se o ID é novo, adiciona uma nova mensagem à lista
        if (messageCount < 20) {
            canMessages[messageCount].id = id;
            canMessages[messageCount].len = len;
            canMessages[messageCount].isExtended = (idString.length() > 3);
            memcpy(canMessages[messageCount].data, buf, len);
            messageCount++;
        } else {
            // Se a lista está cheia, remove a mensagem mais antiga e adiciona a nova no final
            for (int i = 0; i < 19; i++) {
                canMessages[i] = canMessages[i+1];
            }
            canMessages[19].id = id;
            canMessages[19].len = len;
            canMessages[19].isExtended = (idString.length() > 3);
            memcpy(canMessages[19].data, buf, len);
        }
        drawCanMessages(); // Redesenha a tela inteira para incluir a nova mensagem
    }
}

// Procura por um ID de mensagem na lista e retorna o índice
int findMessageIndex(unsigned long id) {
    for (int i = 0; i < messageCount; i++) {
        if (canMessages[i].id == id) {
            return i;
        }
    }
    return -1;
}

// Desenha a tela principal (cabeçalho e mensagens)
void drawMainScreen() {
    tft.fillScreen(TFT_BLACK);
    drawHeader();
    drawCanMessages();
}

// Desenha o cabeçalho com os botões "SEND", "LIMPA" e "SETUP"
void drawHeader() {
    tft.fillRect(0, 0, tft.width(), tft.height(), TFT_BLACK);
    tft.setTextSize(2);
    tft.setTextColor(TFT_WHITE);
    tft.setTextDatum(BC_DATUM);
    drawButton(10, tft.height() - 40, 80, 30, "SEND", TFT_GREEN, TFT_WHITE, 2);
    drawButton(tft.width() - 200, tft.height() - 40, 80, 30, "LIMPA", TFT_YELLOW, TFT_WHITE, 2);
    drawButton(tft.width() - 90, tft.height() - 40, 80, 30, "SETUP", TFT_BLUE, TFT_WHITE, 2);
}

// Desenha a lista completa de mensagens CAN
void drawCanMessages() {
    tft.fillRect(0, 0, tft.width(), tft.height() - 60, TFT_BLACK);
    for (int i = 0; i < messageCount; i++) {
        drawSingleCanMessage(i, i * 10 + 5);
    }
}

// Desenha uma única linha de mensagem CAN, com destaque para bytes alterados
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
        // Se a mensagem for a mesma e o byte atual mudou, usa a cor vermelha
        if (oldData != nullptr && canMessages[index].data[j] != oldData[j]) {
            tft.setTextColor(TFT_RED);
        } else {
            tft.setTextColor(idColor);
        }

        // Converte e desenha o byte atual
        String byteString = "";
        if (canMessages[index].data[j] < 16) byteString += "0";
        byteString += String(canMessages[index].data[j], HEX);
        byteString.toUpperCase();
        tft.drawString(byteString, xPos, yOffset);
        xPos += 15;

        // Desenha o separador ":"
        if (j < canMessages[index].len - 1) {
            tft.setTextColor(idColor);
            tft.drawString(":", xPos, yOffset);
            xPos += 5;
        }
    }
}

// Desenha a tela de configurações
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

// Desenha o teclado virtual na tela
void drawKeyboard() {
    for (int i = 0; i < 18; i++) {
        drawButton(keys[i].x, keys[i].y, keys[i].w, keys[i].h, keys[i].label, TFT_DARKGREY, TFT_WHITE, 1);
    }
}

// Desenha a tela de envio de frames
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

// Trata os eventos de toque na tela
void handleTouch() {
    if (ts.touched()) {
        TS_Point p = ts.getPoint();
        uint16_t touch_x = map(p.x, touchMinX, touchMaxX, 0, tft.width());
        uint16_t touch_y = map(p.y, touchMinY, touchMaxY, 0, tft.height());

        if (isConnected) {
            if (currentScreen == MAIN_SCREEN) {
                // Lógica de botões da tela principal
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
                // Lógica de botões da tela de configurações
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
                // Lógica de botões e teclado da tela de envio
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

// Envia o frame CAN via Wi-Fi
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

// Salva as configurações de velocidade e ID na memória flash
void saveConfig() {
    preferences.begin("can-config", false); // Inicia a Preferences para escrita
    preferences.putLong("canSpeed", canSpeed);
    preferences.putBool("isExtendedID", isExtendedID);
    preferences.end(); // Fecha a Preferences
    Serial.println("Configuracoes salvas.");
}

// Carrega as configurações de velocidade e ID da memória flash
void loadConfig() {
    preferences.begin("can-config", true); // Inicia a Preferences para leitura
    canSpeed = preferences.getLong("canSpeed", 250000); // Carrega com valor padrão se não existir
    isExtendedID = preferences.getBool("isExtendedID", false);
    preferences.end(); // Fecha a Preferences
    Serial.println("Configuracoes carregadas.");
}

// Função utilitária para desenhar botões
void drawButton(int x, int y, int w, int h, const char* label, uint16_t bgColor, uint16_t borderColor, uint8_t textSize) {
    tft.fillRect(x, y, w, h, bgColor);
    tft.drawRect(x, y, w, h, borderColor);
    tft.setTextColor(TFT_WHITE);
    tft.setTextDatum(MC_DATUM);
    tft.setTextSize(textSize);
    tft.drawString(label, x + w / 2, y + h / 2);
}

// Limpa a lista de mensagens CAN
void clearCanMessages() {
    messageCount = 0;
    drawMainScreen();
}

// Envia um comando para ligar/desligar um LED
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

// Envia um comando para alterar a velocidade do CAN
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

// fim
