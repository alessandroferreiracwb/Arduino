#include <SPI.h>            // Inclui a biblioteca para comunicação SPI (Serial Peripheral Interface)
#include <TFT_eSPI.h>       // Inclui a biblioteca para o driver da tela TFT (Thin-Film Transistor)
#include <Preferences.h>    // Inclui a biblioteca Preferences, usada para salvar dados na memória flash
#include <XPT2046_Touchscreen.h> // Inclui a biblioteca para o driver do touchscreen XPT2046
#include <WiFi.h>           // Inclui a biblioteca para conectar o ESP32 a redes Wi-Fi

// --- Credenciais da Rede Wi-Fi ---
const char* ssid = "ESP_CAN";             // Define o nome da rede Wi-Fi (SSID)
const char* password = "admin123";        // Define a senha da rede Wi-Fi
const char* serverIp = "192.168.4.1";     // Define o endereço IP do servidor (o outro ESP32 com o CAN Bus)
const uint16_t port = 8080;               // Define a porta de comunicação do servidor

WiFiClient client;                        // Cria um objeto cliente para se conectar ao servidor Wi-Fi
bool isConnected = false;                 // Flag para verificar se a conexão com o servidor foi estabelecida
unsigned long lastConnectionAttempt = 0;  // Armazena o último tempo em que uma tentativa de conexão foi feita
const long connectionInterval = 5000;     // Define o intervalo de tempo (em milissegundos) para tentar reconectar (5 segundos)

// --- Definições de Pinos e Calibração do seu hardware ---
#define TFT_CS 15      // Pino Chip Select (CS) para a tela TFT
#define TFT_DC 2       // Pino Data/Command (DC) para a tela TFT
#define TFT_RST -1     // Pino de Reset (RST) para a tela TFT (-1 significa que não está em uso)
#define TFT_MOSI 13    // Pino Master Out Slave In (MOSI) para a comunicação SPI com a tela
#define TFT_SCLK 14    // Pino Serial Clock (SCLK) para a comunicação SPI com a tela
#define TFT_MISO 12    // Pino Master In Slave Out (MISO) para a comunicação SPI com a tela

// Pinos SPI do seu touch
#define XPT2046_MOSI 32 // Pino MOSI para o touchscreen
#define XPT2046_MISO 39 // Pino MISO para o touchscreen
#define XPT2046_CLK 25  // Pino SCLK para o touchscreen
#define XPT2046_CS 33   // Pino CS para o touchscreen
#define XPT2046_IRQ 36  // Pino de Interrupção (IRQ) para o touchscreen

// Valores de calibração do seu touch
int touchMinX = 451;  // Valor mínimo de X lido do touchscreen
int touchMaxX = 3598; // Valor máximo de X lido do touchscreen
int touchMinY = 600;  // Valor mínimo de Y lido do touchscreen
int touchMaxY = 3579; // Valor máximo de Y lido do touchscreen
// -----------------------------------------------------------

TFT_eSPI tft = TFT_eSPI();                         // Cria um objeto da tela TFT
SPIClass touchscreenSPI = SPIClass(VSPI);          // Cria um objeto SPI para o touchscreen, usando o bus VSPI
XPT2046_Touchscreen ts(XPT2046_CS, XPT2046_IRQ);   // Cria um objeto para o touchscreen, passando os pinos CS e IRQ

// Variáveis de estado
enum Screen { MAIN_SCREEN, SETUP_SCREEN, SEND_SCREEN, CONNECTING_SCREEN }; // Define os estados possíveis da interface
Screen currentScreen = CONNECTING_SCREEN;                                 // Define a tela inicial como 'conectando'

// Variáveis para as configurações
Preferences preferences;          // Cria um objeto para armazenar as configurações na memória
long canSpeed = 250000;           // Variável para a velocidade do CAN Bus, com valor padrão
bool isExtendedID = false;        // Variável para o tipo de ID (true para estendido, false para padrão), com valor padrão

// Variáveis para a tela de envio
String sendId = "18F00001";              // Variável para o ID da mensagem CAN a ser enviada
String sendFrame[8] = {"FF", "FF", "FF", "FF", "FF", "FF", "FF", "FF"}; // Array para os 8 bytes do frame CAN
String sendInterval = "100";             // Variável para o intervalo de envio da mensagem (em ms)

// Variável para rastrear o campo de entrada ativo
enum ActiveInput { NONE, ID, FRAME_BYTE, TEMPO }; // Define os tipos de campo que podem ser editados
ActiveInput activeInput = NONE;                   // Define o campo de entrada ativo (nenhum por padrão)
int activeFrameByte = -1;                         // Índice do byte do frame ativo para edição

// Estrutura para as teclas do teclado virtual
struct Key {
    int x, y, w, h;       // Posição (x, y) e dimensões (largura, altura) da tecla
    char label[4];        // Texto da tecla
};

Key keys[18] = {
    {10, 100, 30, 20, "7"}, {50, 100, 30, 20, "8"}, {90, 100, 30, 20, "9"}, {130, 100, 30, 20, "A"}, {170, 100, 30, 20, "B"},
    {10, 125, 30, 20, "4"}, {50, 125, 30, 20, "5"}, {90, 125, 30, 20, "6"}, {130, 125, 30, 20, "C"}, {170, 125, 30, 20, "D"},
    {10, 150, 30, 20, "1"}, {50, 150, 30, 20, "2"}, {90, 150, 30, 20, "3"}, {130, 150, 30, 20, "E"}, {170, 150, 30, 20, "F"},
    {10, 175, 30, 20, "0"}, {50, 175, 30, 20, "Bk"}, {90, 175, 30, 20, "En"}
}; // Array que define a posição e o rótulo de cada tecla do teclado virtual

// --- Variáveis de mensagens globais ---
struct CanMessage {
    unsigned long id;     // ID da mensagem CAN
    unsigned char data[8]; // Array de bytes dos dados do frame
    unsigned char len;    // Tamanho (número de bytes) da mensagem
    bool isExtended;      // Se o ID é estendido (true) ou padrão (false)
}; // Estrutura para armazenar informações de uma mensagem CAN recebida
CanMessage canMessages[20]; // Array para armazenar as últimas 20 mensagens CAN recebidas
int messageCount = 0;       // Contador de mensagens na lista
int scrollPosition = 0;     // Posição de rolagem (não usada neste código, mas pode ser útil para implementações futuras)
// --------------------------------------

// Protótipos das funções
void drawMainScreen();        // Declara a função que desenha a tela principal
void drawSetupScreen();       // Declara a função que desenha a tela de configurações
void drawSendScreen();        // Declara a função que desenha a tela de envio de mensagens
void drawHeader();            // Declara a função que desenha o cabeçalho da tela principal
void drawCanMessages();       // Declara a função que desenha a lista de mensagens CAN
void drawConnectingScreen();  // Declara a função que desenha a tela de conexão
void handleTouch();           // Declara a função que lida com os toques na tela
void saveConfig();            // Declara a função que salva as configurações na memória
void loadConfig();            // Declara a função que carrega as configurações da memória
void setupWifi();             // Declara a função que inicia a conexão Wi-Fi
void checkWifiConnection();   // Declara a função que verifica o estado da conexão Wi-Fi
void sendCanFrameWifi();      // Declara a função que envia um frame CAN via Wi-Fi
void processWifiData(String data); // Declara a função que processa os dados recebidos via Wi-Fi
void drawButton(int x, int y, int w, int h, const char* label, uint16_t bgColor, uint16_t borderColor, uint8_t textSize = 2); // Declara a função para desenhar um botão
int findMessageIndex(unsigned long id); // Declara a função que encontra o índice de uma mensagem pelo seu ID
void drawSingleCanMessage(int index, int yOffset); // Declara a função que desenha uma única mensagem CAN
void clearCanMessages(); // Declara a função para limpar as mensagens CAN
//void sendLedCommand(bool state);

void setup() {
    Serial.begin(115200);   // Inicia a comunicação serial para debug, com taxa de 115200 bps
    SPI.begin();            // Inicia o bus SPI padrão

    tft.init();             // Inicializa a tela TFT
    tft.setRotation(1);     // Define a rotação da tela (1 para paisagem)
    tft.fillScreen(TFT_BLACK); // Preenche a tela com a cor preta

    touchscreenSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS); // Inicia o bus SPI para o touchscreen
    ts.begin(touchscreenSPI); // Inicializa o touchscreen com o bus SPI
    ts.setRotation(1);        // Define a rotação do touchscreen para corresponder à da tela

    loadConfig();           // Chama a função para carregar as configurações salvas
    setupWifi();            // Chama a função para iniciar a conexão Wi-Fi

    drawConnectingScreen(); // Chama a função para desenhar a tela inicial de conexão
}

void loop() {
    checkWifiConnection();  // Chama a função para verificar o estado da conexão Wi-Fi

    if (isConnected) {      // Se estiver conectado...
        handleTouch();      // ...chama a função para lidar com os toques na tela
        
        // Altera a lógica de recepção: só processa dados se a tela principal estiver ativa
        if (currentScreen == MAIN_SCREEN && client.connected() && client.available()) { // Se a tela principal estiver ativa, o cliente estiver conectado e houver dados disponíveis...
            String data = client.readStringUntil('\n'); // ...lê os dados até encontrar uma nova linha
            processWifiData(data); // ...e processa os dados recebidos
        }
    }
}

// --- Funções de Conexão Wi-Fi ---
void setupWifi() {
    Serial.print("Conectando a ");   // Imprime no monitor serial
    Serial.println(ssid);           // ...o nome da rede Wi-Fi
    WiFi.begin(ssid, password);     // Inicia a conexão Wi-Fi com as credenciais
}

void checkWifiConnection() {
    if (isConnected) {              // Se já estiver conectado...
        if (!client.connected()) {  // ...e o cliente perder a conexão...
            Serial.println("Conexao com o servidor perdida."); // ...imprime a mensagem
            isConnected = false;    // ...atualiza o status de conexão
            currentScreen = CONNECTING_SCREEN; // ...muda para a tela de conexão
            drawConnectingScreen(); // ...e redesenha a tela
        }
        return; // Sai da função
    }

    if (currentScreen != CONNECTING_SCREEN) { // Se não estiver na tela de conexão e não estiver conectado...
        currentScreen = CONNECTING_SCREEN;    // ...muda para a tela de conexão
        drawConnectingScreen();               // ...e redesenha a tela
    }

    if (millis() - lastConnectionAttempt < connectionInterval) { // Verifica se já passou o tempo para a próxima tentativa de conexão
        return; // Sai da função
    }
    lastConnectionAttempt = millis(); // Atualiza o tempo da última tentativa

    if (WiFi.status() != WL_CONNECTED) { // Se o Wi-Fi não estiver conectado...
        Serial.println("WiFi desconectado. Tentando reconectar..."); // ...imprime a mensagem
        tft.fillScreen(TFT_BLACK);        // Preenche a tela com preto
        tft.setTextSize(2);               // Define o tamanho do texto
        tft.setTextColor(TFT_WHITE);      // Define a cor do texto
        tft.setTextDatum(MC_DATUM);       // Define a posição do texto (centro)
        tft.drawString("TENTANDO CONECTAR...", tft.width() / 2, tft.height() / 2); // Desenha a mensagem
        WiFi.begin(ssid, password);       // Tenta reconectar o Wi-Fi
        return; // Sai da função
    }

    if (!client.connected()) { // Se o Wi-Fi estiver conectado, mas o cliente não...
        Serial.print("WiFi conectado. Tentando conectar ao servidor "); // ...imprime a mensagem
        if (client.connect(serverIp, port)) { // Tenta conectar ao servidor...
            Serial.println("Conectado ao servidor!"); // ...se for bem-sucedido, imprime a mensagem
            isConnected = true;               // ...atualiza o status de conexão
            currentScreen = MAIN_SCREEN;      // ...muda para a tela principal
            messageCount = 0;                 // ...zera o contador de mensagens
            drawMainScreen();                 // ...e desenha a tela principal
        } else {
            Serial.println("Conexão com o servidor falhou."); // Se a conexão falhar, imprime a mensagem
        }
    }
}

void drawConnectingScreen() {
    tft.fillScreen(TFT_BLACK); // Preenche a tela com preto
    tft.setTextSize(2);        // Define o tamanho do texto
    tft.setTextColor(TFT_RED); // Define a cor do texto para vermelho
    tft.setTextDatum(MC_DATUM); // Centraliza o texto
    tft.drawString("CONEXAO PERDIDA", tft.width() / 2, tft.height() / 2 - 20); // Desenha a mensagem de conexão perdida
    tft.setTextColor(TFT_WHITE); // Define a cor do texto para branco
    tft.drawString("TENTANDO RECONECTAR...", tft.width() / 2, tft.height() / 2 + 10); // Desenha a mensagem de reconexão
}

// --- Funções de Desenho e Lógica (restante do código) ---
void processWifiData(String data) {
    data.trim(); // Remove espaços em branco do início e fim da string
    int spaceIndex = data.indexOf(' '); // Encontra o índice do primeiro espaço em branco
    if (spaceIndex == -1) { // Se não houver espaço, a mensagem é inválida
        return; // Sai da função
    }
    
    String idString = data.substring(0, spaceIndex); // Pega a parte do ID antes do espaço
    String dataString = data.substring(spaceIndex + 1); // Pega a parte dos dados após o espaço

    unsigned long id = strtoul(idString.c_str(), NULL, 16); // Converte a string do ID para um número (base 16)
    int len = dataString.length() / 2; // Calcula o comprimento dos dados (cada byte tem 2 caracteres)
    unsigned char buf[8];              // Buffer para armazenar os bytes dos dados
    
    if (len > 8) { // Se o comprimento for maior que 8...
        len = 8; // ...limita a 8
    }
    
    for(int i = 0; i < len; i++) { // Loop para extrair cada byte
        String byteString = dataString.substring(i * 2, i * 2 + 2); // Pega 2 caracteres (um byte) da string
        buf[i] = strtoul(byteString.c_str(), NULL, 16); // Converte a string do byte para um número
    }

    // Encontra a mensagem pelo ID
    int messageIndex = findMessageIndex(id); // Procura se o ID já existe na lista

    if (messageIndex != -1) { // Se o ID já existir...
        // ID existente, apenas atualiza os bytes que mudaram
        bool updated = false; // Flag para verificar se a mensagem foi atualizada
        for(int i = 0; i < len; i++) { // Percorre os bytes da mensagem
            if (canMessages[messageIndex].data[i] != buf[i]) { // Se o byte for diferente...
                canMessages[messageIndex].data[i] = buf[i]; // ...atualiza o byte
                updated = true; // ...e define a flag como verdadeira
            }
        }
        canMessages[messageIndex].len = len; // Atualiza o comprimento da mensagem
        if (updated) { // Se a mensagem foi atualizada...
            drawSingleCanMessage(messageIndex, messageIndex * 10 + 5); // ...redesenha apenas a linha da mensagem
        }
    } else { // Se o ID for novo...
        // ID novo, adiciona uma nova mensagem à lista
        if (messageCount < 20) { // Se a lista não estiver cheia (menos de 20 mensagens)...
            canMessages[messageCount].id = id; // ...adiciona o novo ID
            canMessages[messageCount].len = len; // ...adiciona o comprimento
            canMessages[messageCount].isExtended = (idString.length() > 3); // ...verifica se o ID é estendido
            memcpy(canMessages[messageCount].data, buf, len); // ...copia os dados para o array
            messageCount++; // ...e incrementa o contador
        } else { // Se a lista estiver cheia...
            for (int i = 0; i < 19; i++) { // ...move todas as mensagens uma posição para cima
                canMessages[i] = canMessages[i+1];
            }
            canMessages[19].id = id; // ...adiciona a nova mensagem na última posição
            canMessages[19].len = len;
            canMessages[19].isExtended = (idString.length() > 3);
            memcpy(canMessages[19].data, buf, len);
        }
        drawCanMessages(); // Redesenha tudo apenas quando uma nova mensagem é adicionada
    }
}

int findMessageIndex(unsigned long id) {
    for (int i = 0; i < messageCount; i++) { // Percorre a lista de mensagens
        if (canMessages[i].id == id) { // Se o ID da mensagem atual for igual ao ID procurado...
            return i; // ...retorna o índice
        }
    }
    return -1; // Se o ID não for encontrado, retorna -1
}

void drawMainScreen() {
    tft.fillScreen(TFT_BLACK); // Preenche a tela com preto
    drawHeader();              // Desenha o cabeçalho
    drawCanMessages();         // Desenha a lista de mensagens CAN
}

void drawHeader() {
    tft.fillRect(0, 0, tft.width(), tft.height(), TFT_BLACK); // Limpa a tela inteira com preto
    tft.setTextSize(2);          // Define o tamanho do texto
    tft.setTextColor(TFT_WHITE); // Define a cor do texto
    tft.setTextDatum(BC_DATUM);  // Alinha o texto na parte inferior-central
    //tft.drawString("CAN BUS", tft.width() / 2, tft.height() - 5); // Desenha a string "CAN BUS" no rodapé
    drawButton(10, tft.height() - 40, 80, 30, "SEND", TFT_GREEN, TFT_WHITE, 2); // Desenha o botão "SEND"
    drawButton(tft.width() - 200, tft.height() - 40, 80, 30, "LIMPA", TFT_YELLOW, TFT_WHITE, 2); // Desenha o botão "LIMPA"
    drawButton(tft.width() - 90, tft.height() - 40, 80, 30, "SETUP", TFT_BLUE, TFT_WHITE, 2); // Desenha o botão "SETUP"
}

void drawCanMessages() {
    tft.fillRect(0, 0, tft.width(), tft.height() - 60, TFT_BLACK); // Limpa a área de mensagens com preto
    for (int i = 0; i < messageCount; i++) { // Percorre a lista de mensagens
        drawSingleCanMessage(i, i * 10 + 5); // Desenha cada mensagem
    }
}

void drawSingleCanMessage(int index, int yOffset) {
    tft.setTextDatum(TL_DATUM); // Alinha o texto no canto superior esquerdo
    tft.setTextSize(1);         // Define o tamanho do texto
    uint16_t color = (index % 2 == 0) ? TFT_LIGHTGREY : TFT_CYAN; // Alterna a cor entre cinza claro e ciano
    tft.setTextColor(color); // Define a cor do texto
    
    String idString = String(canMessages[index].id, HEX); // Converte o ID para uma string hexadecimal
    idString.toUpperCase(); // Converte a string para maiúsculas
    String frameData = "";  // Inicializa uma string para os dados
    for (int j = 0; j < canMessages[index].len; j++) { // Percorre os bytes dos dados
        if (canMessages[index].data[j] < 16) frameData += "0"; // Adiciona um zero à esquerda se o byte for menor que 16
        frameData += String(canMessages[index].data[j], HEX); // Converte o byte para uma string hexadecimal e adiciona
        if (j < canMessages[index].len - 1) { // Se não for o último byte...
            frameData += ":"; // ...adiciona um ":"
        }
    }
    frameData.toUpperCase(); // Converte a string dos dados para maiúsculas
    
    tft.fillRect(0, yOffset, tft.width(), 10, TFT_BLACK); // Limpa a linha antes de desenhar
    tft.drawString(idString, 5, yOffset);                 // Desenha o ID da mensagem
    tft.drawString(frameData, 90, yOffset);               // Desenha os dados do frame
}

void drawSetupScreen() {
    tft.fillScreen(TFT_BLACK); // Preenche a tela com preto
    drawButton(10, tft.height() - 40, 80, 30, "Voltar", TFT_RED, TFT_WHITE, 2);   // Desenha o botão "Voltar"
    drawButton(tft.width() - 90, tft.height() - 40, 80, 30, "Salvar", TFT_GREEN, TFT_WHITE, 2); // Desenha o botão "Salvar"
    tft.setTextSize(2);          // Define o tamanho do texto
    tft.setTextColor(TFT_WHITE); // Define a cor do texto
    tft.setTextDatum(TL_DATUM);  // Alinha o texto no canto superior esquerdo
    tft.drawString("Velocidade CAN:", 10, 40); // Desenha o texto do cabeçalho de velocidade
    tft.drawRect(10, 70, 80, 30, (canSpeed == 125000) ? TFT_GREEN : TFT_WHITE); // Desenha o botão de 125k, com borda verde se estiver selecionado
    tft.drawString("125k", 40, 80); // Desenha o texto "125k"
    tft.drawRect(100, 70, 80, 30, (canSpeed == 250000) ? TFT_GREEN : TFT_WHITE); // Desenha o botão de 250k
    tft.drawString("250k", 130, 80); // Desenha o texto "250k"
    tft.drawRect(190, 70, 80, 30, (canSpeed == 500000) ? TFT_GREEN : TFT_WHITE); // Desenha o botão de 500k
    tft.drawString("500k", 220, 80); // Desenha o texto "500k"
    tft.drawString("Tipo de ID:", 10, 120); // Desenha o texto do cabeçalho do tipo de ID
    tft.drawRect(10, 150, 100, 30, (!isExtendedID) ? TFT_GREEN : TFT_WHITE); // Desenha o botão "Std" (padrão)
    tft.drawString("Std", 50, 160); // Desenha o texto "Std"
    tft.drawRect(120, 150, 100, 30, (isExtendedID) ? TFT_GREEN : TFT_WHITE); // Desenha o botão "Ext" (estendido)
    tft.drawString("Ext", 160, 160); // Desenha o texto "Ext"
}

void drawKeyboard() {
    for (int i = 0; i < 18; i++) { // Percorre o array de teclas
        drawButton(keys[i].x, keys[i].y, keys[i].w, keys[i].h, keys[i].label, TFT_DARKGREY, TFT_WHITE, 1); // Desenha cada tecla
    }
}

void drawSendScreen() {
    tft.fillScreen(TFT_BLACK); // Preenche a tela com preto
    tft.setTextSize(2);          // Define o tamanho do texto
    tft.setTextColor(TFT_WHITE); // Define a cor do texto
    tft.setTextDatum(TL_DATUM);  // Alinha o texto no canto superior esquerdo
    tft.drawString("ID:", 10, 10); // Desenha o texto "ID:"
    uint16_t idColor = (activeInput == ID) ? TFT_GREEN : TFT_WHITE; // Define a cor da borda do ID (verde se estiver ativo)
    tft.drawRect(50, 5, 120, 30, idColor); // Desenha o retângulo do ID
    tft.setTextDatum(MC_DATUM);  // Centraliza o texto
    tft.drawString(sendId, 110, 20); // Desenha o valor do ID
    tft.setTextDatum(TL_DATUM);  // Alinha o texto no canto superior esquerdo
    tft.drawString("Frame:", 10, 50); // Desenha o texto "Frame:"
    int xPos = 80;               // Posição inicial para o primeiro byte do frame
    for (int i = 0; i < 8; i++) { // Loop para desenhar os 8 bytes do frame
        uint16_t color = (activeInput == FRAME_BYTE && activeFrameByte == i) ? TFT_GREEN : TFT_WHITE; // Cor da borda do byte
        tft.drawRect(xPos, 45, 30, 30, color); // Desenha o retângulo do byte
        tft.setTextDatum(MC_DATUM); // Centraliza o texto
        tft.drawString(sendFrame[i], xPos + 15, 60); // Desenha o valor do byte
        xPos += 35; // Incrementa a posição para o próximo byte
    }
    tft.setTextSize(2);          // Define o tamanho do texto
    tft.setTextDatum(TL_DATUM);  // Alinha o texto no canto superior esquerdo
    tft.drawString("Tempo:", 210, 100); // Desenha o texto "Tempo:"
    uint16_t tempoColor = (activeInput == TEMPO) ? TFT_GREEN : TFT_WHITE; // Cor da borda do tempo
    tft.drawRect(210, 125, 80, 30, tempoColor); // Desenha o retângulo do tempo
    tft.setTextDatum(MC_DATUM);  // Centraliza o texto
    tft.drawString(sendInterval, 250, 140); // Desenha o valor do intervalo
    drawButton(10, tft.height() - 40, 80, 30, "Voltar", TFT_RED, TFT_WHITE, 2); // Desenha o botão "Voltar"
    drawButton(tft.width() - 90, tft.height() - 40, 80, 30, "Enviar", TFT_GREEN, TFT_WHITE, 2); // Desenha o botão "Enviar"
    drawKeyboard(); // Desenha o teclado virtual
}

void handleTouch() {
    if (ts.touched()) { // Se a tela for tocada...
        TS_Point p = ts.getPoint(); // ...obtém as coordenadas do toque
        uint16_t touch_x = map(p.x, touchMinX, touchMaxX, 0, tft.width()); // Mapeia o valor de X do touchscreen para a tela
        uint16_t touch_y = map(p.y, touchMinY, touchMaxY, 0, tft.height()); // Mapeia o valor de Y do touchscreen para a tela

        if (isConnected) { // Se estiver conectado...
            if (currentScreen == MAIN_SCREEN) { // Se a tela atual for a principal...
                if (touch_x > 10 && touch_x < 90 && touch_y > tft.height() - 40 && touch_y < tft.height() - 10) { // Se o botão "SEND" for tocado...
                    currentScreen = SEND_SCREEN; // ...muda para a tela de envio
                    activeInput = NONE;          // ...reseta o campo ativo
                    drawSendScreen();            // ...e desenha a tela de envio
                }
                if (touch_x > tft.width() - 90 && touch_x < tft.width() - 10 && touch_y > tft.height() - 40 && touch_y < tft.height() - 10) { // Se o botão "SETUP" for tocado...
                    currentScreen = SETUP_SCREEN; // ...muda para a tela de configurações
                    drawSetupScreen();            // ...e desenha a tela de configurações
                }
                 // Verifica o toque no botão "Limpar"
                if (touch_x > tft.width() / 2 - 40 && touch_x < tft.width() / 2 + 40 && touch_y > tft.height() - 40 && touch_y < tft.height() - 10) {
                    clearCanMessages();
                }
            } else if (currentScreen == SETUP_SCREEN) { // Se a tela atual for a de configurações...
                if (touch_x > 10 && touch_x < 90 && touch_y > tft.height() - 40 && touch_y < tft.height() - 10) { // Se o botão "Voltar" for tocado...
                    currentScreen = MAIN_SCREEN; // ...muda para a tela principal
                    drawMainScreen();            // ...e desenha a tela principal
                } else if (touch_x > tft.width() - 90 && touch_x < tft.width() - 10 && touch_y > tft.height() - 40 && touch_y < tft.height() - 10) { // Se o botão "Salvar" for tocado...
                    saveConfig();                // ...salva as configurações
                    currentScreen = MAIN_SCREEN; // ...muda para a tela principal
                    drawMainScreen();            // ...e desenha a tela principal
                } else if (touch_x > 10 && touch_x < 90 && touch_y > 70 && touch_y < 100) { // Se o botão "125k" for tocado...
                    canSpeed = 125000;           // ...define a velocidade CAN
                    drawSetupScreen();           // ...e redesenha a tela
                } else if (touch_x > 100 && touch_x < 180 && touch_y > 70 && touch_y < 100) { // Se o botão "250k" for tocado...
                    canSpeed = 250000;
                    drawSetupScreen();
                } else if (touch_x > 190 && touch_x < 270 && touch_y > 70 && touch_y < 100) { // Se o botão "500k" for tocado...
                    canSpeed = 500000;
                    drawSetupScreen();
                } else if (touch_x > 10 && touch_x < 110 && touch_y > 150 && touch_y < 180) { // Se o botão "Std" for tocado...
                    isExtendedID = false;        // ...define o tipo de ID como padrão
                    drawSetupScreen();
                } else if (touch_x > 120 && touch_x < 220 && touch_y > 150 && touch_y < 180) { // Se o botão "Ext" for tocado...
                    isExtendedID = true;         // ...define o tipo de ID como estendido
                    drawSetupScreen();
                    //sendLedCommand(true);  // NOVO: Envia comando para ligar o LED (Ext)
                }
            } else if (currentScreen == SEND_SCREEN) { // Se a tela atual for a de envio...
                if (touch_x > 10 && touch_x < 90 && touch_y > tft.height() - 40 && touch_y < tft.height() - 10) { // Se o botão "Voltar" for tocado...
                    currentScreen = MAIN_SCREEN; // ...muda para a tela principal
                    activeInput = NONE;          // ...reseta o campo ativo
                    drawMainScreen();            // ...e desenha a tela principal
                }
                if (touch_x > tft.width() - 90 && touch_x < tft.width() - 10 && touch_y > tft.height() - 40 && touch_y < tft.height() - 10) { // Se o botão "Enviar" for tocado...
                    sendCanFrameWifi();          // ...envia a mensagem CAN
                    currentScreen = MAIN_SCREEN; // ...muda para a tela principal
                    activeInput = NONE;          // ...reseta o campo ativo
                    drawMainScreen();            // ...e desenha a tela principal
                }
                if (touch_x > 50 && touch_x < 250 && touch_y > 5 && touch_y < 35) { // Se o campo do ID for tocado...
                    activeInput = ID;            // ...define o ID como campo ativo
                    drawSendScreen();            // ...e redesenha a tela
                }
                int xPos = 80;
                for (int i = 0; i < 8; i++) { // Loop para verificar qual byte do frame foi tocado
                    if (touch_x > xPos && touch_x < xPos + 30 && touch_y > 45 && touch_y < 75) { // Se o byte for tocado...
                        activeInput = FRAME_BYTE;    // ...define o campo ativo como frame
                        activeFrameByte = i;         // ...e o índice do byte ativo
                        drawSendScreen();            // ...e redesenha a tela
                        break;                       // Sai do loop
                    }
                    xPos += 35;
                }
                if (touch_x > 210 && touch_x < 290 && touch_y > 125 && touch_y < 155) { // Se o campo do tempo for tocado...
                    activeInput = TEMPO;             // ...define o tempo como campo ativo
                    drawSendScreen();                // ...e redesenha a tela
                }
                for (int i = 0; i < 18; i++) { // Loop para verificar qual tecla do teclado virtual foi tocada
                    if (touch_x > keys[i].x && touch_x < keys[i].x + keys[i].w &&
                        touch_y > keys[i].y && touch_y < keys[i].y + keys[i].h) {
                        char keyLabel[4];
                        strcpy(keyLabel, keys[i].label); // Copia o rótulo da tecla
                        if (strcmp(keyLabel, "Bk") == 0) { // Se a tecla for "Bk" (backspace)...
                            if (activeInput == ID && sendId.length() > 0) sendId.remove(sendId.length() - 1); // ...remove o último caractere do ID
                            else if (activeInput == FRAME_BYTE && sendFrame[activeFrameByte].length() > 0) sendFrame[activeFrameByte].remove(sendFrame[activeFrameByte].length() - 1); // ...ou do byte do frame
                            else if (activeInput == TEMPO && sendInterval.length() > 0) sendInterval.remove(sendInterval.length() - 1); // ...ou do tempo
                        } else if (strcmp(keyLabel, "En") == 0) { // Se a tecla for "En" (enter)...
                            activeInput = NONE; // ...desativa o campo de entrada
                        } else { // Se for um caractere normal...
                            if (activeInput == ID) { // Se o campo ativo for o ID...
                                if (sendId.length() < 8) sendId += keyLabel; // ...adiciona o caractere se o comprimento for menor que 8
                            } else if (activeInput == FRAME_BYTE) { // Se o campo ativo for o frame...
                                if (sendFrame[activeFrameByte].length() < 2) sendFrame[activeFrameByte] += keyLabel; // ...adiciona o caractere se o comprimento for menor que 2
                            } else if (activeInput == TEMPO) { // Se o campo ativo for o tempo...
                                if (sendInterval.length() < 5) sendInterval += keyLabel; // ...adiciona o caractere se o comprimento for menor que 5
                            }
                        }
                        drawSendScreen(); // Redesenha a tela após a interação com o teclado
                        break;
                    }
                }
            }
            while (ts.touched()) { // Espera o usuário soltar o toque
                delay(10);
            }
        } else {
            // Ignora toques se não estiver conectado
        }
    }
}

void sendCanFrameWifi() {
    String message = sendId + " "; // Monta a mensagem com o ID
    for (int i = 0; i < 8; i++) { // Adiciona os bytes do frame
        message += sendFrame[i];
    }
    message += " " + sendInterval; // Adiciona o intervalo
    
    if (client.connected()) { // Se o cliente estiver conectado...
        client.println(message); // ...envia a mensagem via Wi-Fi
        Serial.print("Mensagem enviada via WiFi: "); // Imprime a mensagem enviada
        Serial.println(message);
    } else {
        Serial.println("Wi-Fi desconectado, não foi possível enviar a mensagem."); // Caso contrário, imprime um erro
    }
}

void saveConfig() {
    preferences.begin("can-config", false); // Inicia a Preferences para escrita
    preferences.putLong("canSpeed", canSpeed); // Salva a velocidade do CAN
    preferences.putBool("isExtendedID", isExtendedID); // Salva o tipo de ID
    preferences.end(); // Fecha a Preferences
    Serial.println("Configuracoes salvas."); // Imprime a confirmação
}

void loadConfig() {
    preferences.begin("can-config", true); // Inicia a Preferences para leitura (true para somente leitura)
    canSpeed = preferences.getLong("canSpeed", 250000); // Carrega a velocidade, com um valor padrão se não existir
    isExtendedID = preferences.getBool("isExtendedID", false); // Carrega o tipo de ID, com um valor padrão
    preferences.end(); // Fecha a Preferences
    Serial.println("Configuracoes carregadas."); // Imprime a confirmação
}

void drawButton(int x, int y, int w, int h, const char* label, uint16_t bgColor, uint16_t borderColor, uint8_t textSize) {
    tft.fillRect(x, y, w, h, bgColor);   // Preenche o retângulo do botão com a cor de fundo
    tft.drawRect(x, y, w, h, borderColor); // Desenha a borda do botão
    tft.setTextColor(TFT_WHITE);         // Define a cor do texto para branco
    tft.setTextDatum(MC_DATUM);          // Centraliza o texto
    tft.setTextSize(textSize);           // Define o tamanho do texto
    tft.drawString(label, x + w / 2, y + h / 2); // Desenha o rótulo do botão
}

void clearCanMessages() {
    messageCount = 0; // Reseta o contador de mensagens para zero
    drawMainScreen(); // Redesenha a tela principal, que agora não terá mensagens para exibir
}
/*
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
*/
// fim