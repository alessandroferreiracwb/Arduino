/*
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Modificado para incluir um modo de configuração Wi-Fi com seleção de SSID na tela
  e TECLADO ALFANUMÉRICO VIRTUAL para a senha/entrada de texto.
  Funcionalidades do Teclado Aprimoradas:
  - Teclado com alfabeto completo (minúsculas, maiúsculas)
  - Botão SHIFT para alternar entre maiúsculas/minúsculas.
  - Botão MODE para alternar entre modo de LETRAS e modo de NÚMEROS/SÍMBOLOS.
  - Backspace e Conectar (GO!) mantidos.
  A função de calibração do touchscreen foi removida para simplificar o código.
  Agora com SSIDs maiores na tela de seleção de rede, e botão "Config WiFi" mais largo.
  Correção do erro "jump to case label".
  Os botões do teclado numérico foram diminuídos ainda mais.
  O botão MENU na tela principal foi movido para o canto superior direito.
  A cor do texto do IP e SSID na tela inicial foi alterada para TFT_DARKGREEN.
  Corrigida a lógica de rolagem na lista de redes Wi-Fi e seleção de IDs.
  Exibição das setas de rolagem na lista de IDs.
  A altura do retângulo de exibição da senha foi aumentada.
  CORREÇÃO FINAL: Alinhamento do ponto de toque da seta de rolagem para cima.
  Adicionado sensor DHT11 no GPIO 22 para leituras de temperatura e umidade reais.
  AJUSTE: IP e SSID agora aparecem na mesma linha na tela principal.
  CORREÇÃO CRÍTICA: Ajuste na ordem de desenho da drawMainScreen para garantir visibilidade da T/U.
  Este código funciona para o ESP32-2432S028.
  NOVO: Adicionado sincronização de data e hora via NTP.
  CORREÇÃO: Uso de funções nativas para formatar a data, resolvendo o erro 'getFormattedDate()'.
  AJUSTE: Data e Hora agora exibidas no CANTO SUPERIOR ESQUERDO da tela principal.
*/

#include <SPI.h>
#include <TFT_eSPI.h>        // Biblioteca principal para o display
#include <XPT2046_Touchscreen.h> // Biblioteca para o touchscreen (leitura dos toques)
#include <WiFi.h>            // Para funcionalidade Wi-Fi
#include <EEPROM.h>          // Para armazenar SSID e Senha (ou pode usar Preferences.h)

// --- Bibliotecas e Definições do DHT ---
#include <Adafruit_Sensor.h> // Necessário para a biblioteca DHT
#include <DHT.h>             // Biblioteca DHT
#include <DHT_U.h>           // Biblioteca DHT Unified (para DHT de Adafruit)

#define DHTPIN 22            // Pino GPIO onde o DHT11 está conectado
#define DHTTYPE DHT11        // Tipo de sensor DHT (DHT11, DHT22, etc.)

DHT dht(DHTPIN, DHTTYPE); // Inicializa o sensor DHT

// --- NTP Definições ---
#include <NTPClient.h>         // Para sincronizar a hora com um servidor NTP
#include <WiFiUdp.h>           // Necessário para o NTPClient
#include <time.h>              // Para a estrutura tm e funções de tempo (gmtime)

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org"); // Servidor NTP padrão
// Ajuste o offset de tempo em segundos para o seu fuso horário (Curitiba: -3 horas * 3600 segundos/hora)
const long utcOffsetInSeconds = -3 * 3600; // Curitiba (GMT-3)

// --- DEFINICOES EEPROM ---
// Tamanho maximo para SSID e Senha (ajuste conforme necessario)
#define MAX_SSID_LEN 32
#define MAX_PASS_LEN 64
// Endereços de memória EEPROM para armazenar SSID e Senha
#define EEPROM_SSID_ADDR 0
#define EEPROM_PASS_ADDR (EEPROM_SSID_ADDR + MAX_SSID_LEN + 1) // +1 para o terminador null
#define EEPROM_SIZE (EEPROM_PASS_ADDR + MAX_PASS_LEN + 1) // Tamanho total da EEPROM usada

TFT_eSPI tft = TFT_eSPI(); // Objeto para o display TFT

// Pinos do Touchscreen (XPT2046)
#define XPT2046_IRQ 36   // T_IRQ (Interrupção do Touch)
#define XPT2046_MOSI 32  // T_DIN (Data In, SPI MOSI para o Touch)
#define XPT2046_MISO 39  // T_OUT (Data Out, SPI MISO para o Touch)
#define XPT2046_CLK 25   // T_CLK (Clock, SPI SCK para o Touch)
#define XPT2046_CS 33    // T_CS (Chip Select para o Touch)

// Cria uma instância SPI para o touchscreen, usando o VSPI do ESP32
SPIClass touchscreenSPI = SPIClass(VSPI);
// Cria o objeto touchscreen, passando o pino CS e IRQ
XPT2046_Touchscreen touchscreen(XPT2046_CS, XPT2046_IRQ);

// Definições de tamanho da tela e fonte
#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240
#define FONT_SIZE 2 // Tamanho da fonte padrão para textos na tela (pode ser ajustado por função)
#define FONT_SIZE_LARGE 2 // Usaremos 3 para temperatura/umidade

// **Valores de Calibração do Touch:**
// Ajustados para "puxar" a leitura para a esquerda
int touchMinX = 406; // Original: 451. Diminuído em 50.
int touchMaxX = 3543; // Original: 3598. Diminuído em 50.
int touchMinY = 600;
int touchMaxY = 3579;

// Variáveis para armazenar as coordenadas do touch (calibradas) e pressão
int x, y, z;

// Armazenamento temporário para SSID e Senha
char storedSsid[MAX_SSID_LEN + 1] = {0};
char storedPass[MAX_PASS_LEN + 1] = {0};

// --- Variáveis para Temperatura e Umidade ---
float temperature = 0.0;
float humidity = 0.0;
unsigned long lastDHTReadTime = 0; // Para controlar o tempo da última leitura
const long DHT_READ_INTERVAL = 5000; // Intervalo de 5 segundos para leitura do DHT

// --- Estado do Sistema ---
enum SystemState {
  STATE_MAIN_SCREEN,
  STATE_SETUP_MENU,
  STATE_WIFI_SCAN,
  STATE_WIFI_INPUT
};

// --- Modos do Teclado Alfanumérico ---
enum KeypadMode {
  MODE_LOWERCASE, // Letras minúsculas
  MODE_UPPERCASE, // Letras maiúsculas
  MODE_NUMBERS    // Números e símbolos
};

KeypadMode currentKeypadMode = MODE_LOWERCASE; // Inicia com letras minúsculas

// Inicialmente, o estado principal é a tela principal.
// Se o botão MENU for pressionado, irá para STATE_SETUP_MENU.
SystemState currentState = STATE_MAIN_SCREEN;

// --- Estrutura para o Botão ---
struct Button {
  int x;
  int y;
  int width;
  int height;
  const char* text;
  uint16_t color;
  uint16_t textColor;
  uint8_t textSize; // Adiciona tamanho do texto para o botão
};

// Definição dos botões fixos
Button menuButton = {
  .x = SCREEN_WIDTH - 80 - 10, // Canto superior direito (largura 80, 10px de margem)
  .y = 2, // Canto superior direito (10px de margem)
  .width = 80,
  .height = 40,
  .text = "MENU",
  .color = TFT_ORANGE,
  .textColor = TFT_WHITE,
  .textSize = FONT_SIZE // Usa o tamanho de fonte padrão
};

// Definição do botão WiFi (agora como única opção no "menu" de setup)
#define BUTTON_MENU_WIDTH 160 // Largura para o botão "Config WiFi"

Button wifiConfigButton = {
  .x = (SCREEN_WIDTH / 2) - (BUTTON_MENU_WIDTH / 2), // Centraliza o botão
  .y = (SCREEN_HEIGHT / 2) - 30, // Posição vertical no meio
  .width = BUTTON_MENU_WIDTH, // Largura
  .height = 40,
  .text = "Config WiFi",
  .color = TFT_ORANGE,
  .textColor = TFT_WHITE,
  .textSize = FONT_SIZE
};

Button backButton = { // Botão Voltar/Cancelar (Inferior esquerdo, padrão)
  .x = 10,
  .y = SCREEN_HEIGHT - 40 - 10,
  .width = 80,
  .height = 40,
  .text = "Voltar",
  .color = TFT_RED,
  .textColor = TFT_WHITE,
  .textSize = FONT_SIZE
};

// Um novo botão "Voltar" temporário para a tela do teclado, com posição ajustada
Button backButtonKeypad = {
  .x = 10, // Canto superior esquerdo do teclado
  .y = 5, // Posição mais alta para não conflitar com campo de senha
  .width = 60, // Menor que o normal para caber
  .height = 30,
  .text = "<-", // Texto para indicar "voltar"
  .color = TFT_RED,
  .textColor = TFT_WHITE,
  .textSize = 1
};


// --- Botões de Rolagem da Lista de WiFi ---
// Definimos como botões explícitos
#define SCROLL_ARROW_WIDTH 30
#define SCROLL_ARROW_HEIGHT 25

// As posições dessas setas serão usadas para definir as áreas de toque,
// mas o desenho será feito com fillTriangle.
Button scrollUpButton = {
    .x = SCREEN_WIDTH - SCROLL_ARROW_WIDTH - 5, // Canto superior direito da área da lista
    .y = 45, // <-- ALTERADO AQUI: Movido para baixo para alinhar a área de toque com o desenho
    .width = SCROLL_ARROW_WIDTH,
    .height = SCROLL_ARROW_HEIGHT,
    .text = "", // Não haverá texto no botão, será um triângulo
    .color = TFT_TRANSPARENT, // Transparente, apenas para definir a área de toque
    .textColor = TFT_TRANSPARENT,
    .textSize = 1
};

Button scrollDownButton = {
    .x = SCREEN_WIDTH - SCROLL_ARROW_WIDTH - 5, // Canto inferior direito da área da lista
    .y = SCREEN_HEIGHT - backButton.height - 10 - SCROLL_ARROW_HEIGHT - 5, // Acima do botão Voltar, com margem
    .width = SCROLL_ARROW_WIDTH,
    .height = SCROLL_ARROW_HEIGHT,
    .text = "", // Não haverá texto no botão, será um triângulo
    .color = TFT_TRANSPARENT, // Transparente, apenas para definir a área de toque
    .textColor = TFT_TRANSPARENT,
    .textSize = 1
};


// --- Teclado Alfanumérico ---
// Ajuste de tamanho e espaçamento para as novas teclas
#define ALPHANUM_KEY_WIDTH 25 // Largura menor para caber mais teclas
#define ALPHANUM_KEY_HEIGHT 25 // Altura menor
#define ALPHANUM_KEY_SPACING_X 3 // Espaçamento horizontal
#define ALPHANUM_KEY_SPACING_Y 3 // Espaçamento vertical

// Posição de início do teclado alfanumérico (agora global para os controlKeys)
#define ALPHANUM_KEYBOARD_START_X 10 // RESTAURADO ESTE DEFINE GLOBALMENTE
#define ALPHANUM_KEYBOARD_START_Y 80

// Nomes dos botões de controle do teclado
const char* SHIFT_TEXT = "Shift";
const char* MODE_TEXT = "123"; // Inicialmente "123" para ir para números
const char* BACK_TEXT = "DEL";
const char* ENTER_TEXT = "GO!";

// Arrays com os caracteres para cada modo do teclado
// Minúsculas (26 letras)
const char* LOWERCASE_KEYS[] = {
  "q","w","e","r","t","y","u","i","o","p", // 10 teclas
  "a","s","d","f","g","h","j","k","l",    // 9 teclas
  "z","x","c","v","b","n","m"             // 7 teclas
};
const int NUM_LOWERCASE_KEYS = sizeof(LOWERCASE_KEYS) / sizeof(LOWERCASE_KEYS[0]);

// Maiúsculas (26 letras)
const char* UPPERCASE_KEYS[] = {
  "Q","W","E","R","T","Y","U","I","O","P",
  "A","S","D","F","G","H","J","K","L",
  "Z","X","C","V","B","N","M"
};
const int NUM_UPPERCASE_KEYS = sizeof(UPPERCASE_KEYS) / sizeof(UPPERCASE_KEYS[0]);


// Números e Símbolos
const char* NUMBER_KEYS[] = {
  "1","2","3","4","5","6","7","8","9","0", // 10 teclas
  "-","+","*","/","=",".","@",            // 7 teclas
  "_","#","!"                             // 3 teclas
};
const int NUM_NUMBER_KEYS = sizeof(NUMBER_KEYS) / sizeof(NUMBER_KEYS[0]);


// Botões especiais que estarão sempre presentes (SHIFT, MODE, DEL, GO!)
Button controlKeys[4];
// Array para armazenar todas as teclas alfanuméricas (letras/números/símbolos)
// Usaremos um tamanho máximo para alocar, assumindo que NUM_LOWERCASE_KEYS é o maior conjunto de caracteres
Button alphaNumKeys[27]; // 26 letras + 1 espaço (para o modo de letras, para o enter)

// Variáveis para o scanner Wi-Fi
int numNetworks = 0;
String ssids[10]; // Armazena até 10 SSIDs para exibição
int currentScrollOffset = 0; // Para rolar a lista de IDs

// SSID_ITEM_HEIGHT será recalculado em drawWifiList para o novo FONT_SIZE
#define SSID_ITEM_HEIGHT (tft.fontHeight(FONT_SIZE) + 4) // Altura de cada item SSID na lista, com padding

// MAX_VISIBLE_SSIDS será recalculado em drawWifiList para caber mais
#define MAX_VISIBLE_SSIDS ((SCREEN_HEIGHT - 25 - backButton.height - 10) / SSID_ITEM_HEIGHT)


String selectedSsid = "";
String enteredPassword = ""; // String para a senha

// --- Funções Auxiliares ---

// Desenha um botão na tela (agora recebe o tamanho do texto do próprio botão)
void drawButton(Button btn) {
  // Se a cor do botão for transparente, não preenche o retângulo, apenas desenha a borda se necessário
  if (btn.color != TFT_TRANSPARENT) {
    tft.fillRect(btn.x, btn.y, btn.width, btn.height, btn.color);
  }
  tft.drawRect(btn.x, btn.y, btn.width, btn.height, TFT_BLACK); // Borda do botão
  
  if (btn.text[0] != '\0') { // Desenha o texto apenas se não for vazio
    tft.setTextColor(btn.textColor);
    tft.setTextSize(btn.textSize); // Usa o tamanho de texto definido no botão
    tft.drawCentreString(btn.text, btn.x + btn.width / 2, btn.y + (btn.height - tft.fontHeight(btn.textSize)) / 2, btn.textSize);
  }
}

// Verifica se um toque ocorreu dentro da área de um botão
bool isButtonPressed(Button btn, int touchX, int touchY) {
  return (touchX >= btn.x && touchX <= (btn.x + btn.width) &&
          touchY >= btn.y && touchY <= (btn.y + btn.height));
}

// Imprime as informações do toque (X, Y, Pressão) no Monitor Serial
void printTouchToSerial(int touchX, int touchY, int touchZ) {
  Serial.print("X = ");
  Serial.print(touchX);
  Serial.print(" | Y = ");
  Serial.print(touchY);
  Serial.print(" | Pressure = ");
  Serial.print(touchZ);
  Serial.println();
}

// --- Funções de Leitura/Gravação EEPROM ---
void saveCredentials(const char* ssid, const char* pass) {
  EEPROM.writeString(EEPROM_SSID_ADDR, ssid);
  EEPROM.writeString(EEPROM_PASS_ADDR, pass);
  EEPROM.commit();
  Serial.println("Credenciais salvas na EEPROM.");
}

void loadCredentials() {
  EEPROM.readString(EEPROM_SSID_ADDR, storedSsid, MAX_SSID_LEN);
  EEPROM.readString(EEPROM_PASS_ADDR, storedPass, MAX_PASS_LEN);
  Serial.print("Credenciais carregadas: SSID='");
  Serial.print(storedSsid);
  Serial.print("', PASS='");
  Serial.print(storedPass);
  Serial.println("'");
}

void clearCredentials() {
  for (int i = 0; i < EEPROM_SIZE; i++) {
    EEPROM.write(i, 0);
  }
  EEPROM.commit();
  Serial.println("Credenciais limpas da EEPROM.");
}


// --- Desenha a tela principal ---
void drawMainScreen() {
  tft.fillScreen(TFT_WHITE); // Limpa a tela por completo
  
  // Desenha o botão MENU primeiro, já que ele está no canto superior direito.
  drawButton(menuButton);

  // --- Adiciona Data e Hora no CANTO SUPERIOR ESQUERDO ---
  if (WiFi.status() == WL_CONNECTED) {
    timeClient.update(); // Pede para o cliente NTP atualizar a hora
    
    tft.setTextSize(1); // Tamanho menor para a data/hora
    tft.setTextColor(TFT_BLACK); // Cor TFT_BLACK para contraste
    
    // Obtém o tempo em segundos desde o Epoch e converte para struct tm
    time_t epochTime = timeClient.getEpochTime();
    struct tm *ptm = gmtime (&epochTime); // Use gmtime para UTC, ou localtime para hora local se configurado o timezone do ESP32

    // Formata a data manualmente
    char dateBuffer[11]; // YYYY-MM-DD\0
    sprintf(dateBuffer, "%04d-%02d-%02d", ptm->tm_year + 1900, ptm->tm_mon + 1, ptm->tm_mday);
    String dateString = String(dateBuffer);

    String formattedTime = timeClient.getFormattedTime(); // Ex: "HH:MM:SS"
    
    // Posições para Data e Hora (canto superior esquerdo)
    int time_x = 5; // 5px de margem da borda esquerda
    int time_y = 10; // 10px de margem da borda superior (para não ficar colado no topo)
    
    tft.setTextDatum(TL_DATUM); // Alinha o texto pelo Top-Left (canto superior esquerdo)
    tft.drawString(formattedTime, time_x, time_y, 1);
    tft.drawString(dateString, time_x, time_y + tft.fontHeight(1) + 2, 1); // Data abaixo da hora
  }
  // --- Fim da seção de Data e Hora ---

  // Retângulo superior para o título "PARMALOG"
  // Ele vai começar logo abaixo da área superior que agora pode ter a data/hora
  int start_y_top_rect_if_wifi = 10 + (2 * tft.fontHeight(1) + 10); // Altura da hora/data + margem
  int start_y_top_rect_no_wifi = 60; // Posição se não houver Wi-Fi (acima do DHT)

  int start_y_top_rect = (WiFi.status() == WL_CONNECTED) ? start_y_top_rect_if_wifi : start_y_top_rect_no_wifi;
  
  // A altura do retângulo deve se estender até o Y=110 (onde começa a área do DHT)
  int height_top_rect = 110 - start_y_top_rect;

  // Garante que o retângulo não tenha altura negativa "PARMALOG"
  if (height_top_rect > 0) {
    tft.fillRect(0, start_y_top_rect, SCREEN_WIDTH, height_top_rect, TFT_WHITE); // Fundo do título: TFT_WHITE
  }

  // 1. Desenha o texto "PARMALOG" DENTRO do retângulo superior
  tft.setTextSize(2); // Tamanho da fonte para o título
  tft.setTextColor(TFT_BLACK); // Cor do texto para TFT_BLACK
  // Calcula o centro vertical do retângulo para o texto
  int center_y_title_text = start_y_top_rect + (height_top_rect / 2) - (tft.fontHeight(2) / 2);
  tft.drawCentreString("PARMALOG", SCREEN_WIDTH / 2, center_y_title_text, 2); // Centralizado

  // 2. Exibe os valores reais do sensor DHT11
  // Limpa APENAS a área onde os valores de T/H serão exibidos ANTES de desenhá-los.
  // A área começa em Y=110 e agora tem altura suficiente para as duas linhas.
  tft.fillRect(0, 110, SCREEN_WIDTH, 75, TFT_WHITE); // Fundo para TFT_DARKGREY
  
  tft.setTextSize(FONT_SIZE_LARGE); // Usa o NOVO tamanho de fonte (3) para T/H
  tft.setTextColor(TFT_BLACK); // Cor do texto de T/H para TFT_WHITE
  
  // Posições ajustadas para centralizar o texto dentro do retângulo da temperatura/umidade.
  // Calcula o offset para centralizar as duas linhas de texto verticalmente no retângulo.
  int text_offset_y = (75 - (tft.fontHeight(FONT_SIZE_LARGE) * 2 + 5)) / 2;
  if (text_offset_y < 0) text_offset_y = 0; // Garante que não vá para fora

  tft.drawCentreString("Temp.: " + String(temperature, 1) + " C", SCREEN_WIDTH / 2, 110 + text_offset_y, FONT_SIZE_LARGE);  
  tft.drawCentreString("Umid.: " + String(humidity, 0) + " %", SCREEN_WIDTH / 2, 110 + text_offset_y + tft.fontHeight(FONT_SIZE_LARGE) + 5, FONT_SIZE_LARGE);  
  
  // 3. Desenha o status do WiFi
  // Limpa a área onde o IP/SSID serão exibidos para evitar "fantasmas"
  tft.fillRect(0, SCREEN_HEIGHT - 60, SCREEN_WIDTH, 40, TFT_WHITE); // Limpa a área de IP/SSID
  
  String wifiInfo = "";
  if (WiFi.status() == WL_CONNECTED) {
    wifiInfo += "IP: " + WiFi.localIP().toString();
    wifiInfo += " | SSID: " + WiFi.SSID(); // Concatenando IP e SSID
  } else {
    wifiInfo += "WiFi: Desconectado";
  }
  
  // Define a cor e o tamanho do texto do IP e SSID
  tft.setTextColor(TFT_DARKGREEN, TFT_WHITE); // Texto verde escuro sobre fundo branco
  tft.setTextSize(1); // Reduz o tamanho do texto para 1

  // Exibe a string concatenada
  tft.drawCentreString(wifiInfo, SCREEN_WIDTH / 2, SCREEN_HEIGHT - 40, 1); // Uma única linha para IP e SSID

}

// --- Desenha o menu de setup (agora apenas para WiFi) ---
void drawSetupMenu() {
  tft.fillScreen(TFT_DARKGREY);
  tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
  tft.drawCentreString("Configuracoes WiFi:", SCREEN_WIDTH / 2, 50, FONT_SIZE); // Texto mais direto
  drawButton(wifiConfigButton); // Apenas o botão de configuração WiFi
  drawButton(backButton); // Botão voltar
}

// --- Funções para o Wi-Fi na tela ---

void performWifiScan() {
  tft.fillScreen(TFT_BLUE);
  tft.setTextColor(TFT_WHITE, TFT_BLUE);
  tft.setTextSize(2);
  tft.drawCentreString("Buscando Redes WiFi...", SCREEN_WIDTH / 2, 5, 1);
  delay(500); // Pequeno atraso para o texto aparecer

  numNetworks = WiFi.scanNetworks();
  Serial.print("Redes encontradas: ");
  Serial.println(numNetworks);
  
  // Depuração: Imprime os SSIDs encontrados no Monitor Serial
  for (int i = 0; i < numNetworks; ++i) {
    Serial.print("  ["); Serial.print(i); Serial.print("] SSID: ");
    Serial.print(WiFi.SSID(i));
    Serial.print(" | RSSI: ");
    Serial.println(WiFi.RSSI(i));
  }


  tft.fillScreen(TFT_BLUE);
  tft.setTextSize(1); // O cabeçalho "Selecione sua rede:" continua pequeno
  tft.drawCentreString("Selecione sua rede:", SCREEN_WIDTH / 2, 5, 1);

  // Armazena SSIDs (limita a 10 para o exemplo)
  // Certifica-se de armazenar no máximo 10 redes ou o número encontrado, o que for menor.
  for (int i = 0; i < numNetworks && i < 10; ++i) {
    ssids[i] = WiFi.SSID(i);
  }
  
  drawWifiList(); // Chama drawWifiList para preencher a área e desenhar as setas
  drawButton(backButton);
}

void drawWifiList() {
  // A largura da área de texto da lista de SSIDs é agora limitada pela posição das setas de rolagem.
  int list_text_area_width = SCREEN_WIDTH - 10 - SCROLL_ARROW_WIDTH - 5; // 10px margem esquerda, 5px espaçamento

  tft.fillRect(0, 25, list_text_area_width, SCREEN_HEIGHT - 25 - backButton.height - 10, TFT_BLACK); // Limpa até antes da seta
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  
  tft.setTextSize(FONT_SIZE); // Define o tamanho da fonte para os SSIDs para o tamanho 2 (FONT_SIZE)

  int current_ssid_item_height = tft.fontHeight(FONT_SIZE) + 4; // Altura de cada item, com padding
  int current_max_visible_ssids = (SCREEN_HEIGHT - 25 - backButton.height - 10) / current_ssid_item_height;

  for (int i = 0; i < current_max_visible_ssids; ++i) {
    int networkIndex = i + currentScrollOffset;
    if (networkIndex < numNetworks) {
      // Desenha o SSID, truncando se for muito longo para a área disponível
      tft.setCursor(10, 28 + i * current_ssid_item_height);
      
      String ssidToDisplay = ssids[networkIndex];
      // Calcula a largura em pixels que o texto pode ocupar
      if (tft.textWidth(ssidToDisplay, FONT_SIZE) > (list_text_area_width - 10)) {
          // Se for muito longo, trunca e adiciona "..."
          String tempSsid = "";
          for(char c : ssidToDisplay) {
              // Certifique-se de que o "..." caiba. Largura de "..." com FONT_SIZE
              if (tft.textWidth(tempSsid + c, FONT_SIZE) < (list_text_area_width - 10 - tft.textWidth("...", FONT_SIZE))) {
                  tempSsid += c;
              } else {
                  break;
              }
          }
          if (tempSsid.length() < ssidToDisplay.length()) { // Se truncou de fato
              ssidToDisplay = tempSsid + "...";
          }
      }
      tft.print(ssidToDisplay);
    } else {
      // Limpa linhas vazias
      tft.fillRect(0, 28 + i * current_ssid_item_height, list_text_area_width, current_ssid_item_height, TFT_BLACK);
    }
  }

  // **** DESENHA OS TRIÂNGULOS DE ROLAGEM AQUI ****
  // Garante que o fundo das setas seja limpo antes de desenhar o triângulo
  tft.fillRect(scrollUpButton.x, scrollUpButton.y, scrollUpButton.width, scrollUpButton.height, TFT_BLUE);
  tft.fillRect(scrollDownButton.x, scrollDownButton.y, scrollDownButton.width, scrollDownButton.height, TFT_BLUE);

  if (currentScrollOffset > 0) { // Desenha seta para cima apenas se houver o que rolar para cima
      // Coordenadas ajustadas para o triângulo ficar visualmente centralizado na área do botão
      int arrow_center_y_up = scrollUpButton.y + (SCROLL_ARROW_HEIGHT / 2); // Centro Y da área de toque
      // Ajustes para o Top Y e Bottom Y do triângulo dentro da área de toque
      tft.fillTriangle(scrollUpButton.x + SCROLL_ARROW_WIDTH / 2,                       // Top X (centro)
                       arrow_center_y_up - 6,                                           // Top Y (um pouco acima do centro da área de toque)
                       scrollUpButton.x + 5,                                            // Bottom-left X
                       arrow_center_y_up + 10,                                          // Bottom-left Y (um pouco abaixo do centro da área de toque)
                       scrollUpButton.x + SCROLL_ARROW_WIDTH - 5,                       // Bottom-right X
                       arrow_center_y_up + 10,                                          // Bottom-right Y
                       TFT_YELLOW); // Cor da seta para cima
  }
  
  if (currentScrollOffset + current_max_visible_ssids < numNetworks) { // Desenha seta para baixo apenas se houver o que rolar para baixo
      // Coordenadas ajustadas para o triângulo ficar visualmente centralizado na área do botão
      int arrow_center_y_down = scrollDownButton.y + (SCROLL_ARROW_HEIGHT / 2); // Centro Y da área de toque
      // Ajustes para o Top Y e Bottom Y do triângulo dentro da área de toque
      tft.fillTriangle(scrollDownButton.x + SCROLL_ARROW_WIDTH / 2,                       // Bottom X (centro)
                       arrow_center_y_down + 10,                                          // Bottom Y (um pouco abaixo do centro da área de toque)
                       scrollDownButton.x + 5,                                            // Top-left X
                       arrow_center_y_down - 6,                                           // Top-left Y (um pouco acima do centro da área de toque)
                       scrollDownButton.x + SCROLL_ARROW_WIDTH - 5,                       // Top-right X
                       arrow_center_y_down - 6,                                           // Top-right Y
                       TFT_YELLOW); // Cor da seta para baixo
  }
}

// Inicializa as definições dos botões do teclado alfanumérico
void initAlphaNumericKeys() {
  // Define os botões de controle (Shift, Mode, DEL, GO!)
  // SHIFT Button
  controlKeys[0] = {
    .x = ALPHANUM_KEYBOARD_START_X, // Canto esquerdo (usa o define global)
    .y = ALPHANUM_KEYBOARD_START_Y + 4 * (ALPHANUM_KEY_HEIGHT + ALPHANUM_KEY_SPACING_Y), // Na última linha
    .width = 50, // Largura adequada para "Shift"
    .height = ALPHANUM_KEY_HEIGHT,
    .text = SHIFT_TEXT,
    .color = TFT_DARKGREY,
    .textColor = TFT_WHITE,
    .textSize = 1
  };
  // MODE Button
  controlKeys[1] = {
    .x = controlKeys[0].x + controlKeys[0].width + ALPHANUM_KEY_SPACING_X,
    .y = controlKeys[0].y,
    .width = 50, // Largura adequada para "123" / "ABC"
    .height = ALPHANUM_KEY_HEIGHT,
    .text = MODE_TEXT, // Texto inicial "123"
    .color = TFT_DARKGREY,
    .textColor = TFT_WHITE,
    .textSize = 1
  };
  // DEL Button
  controlKeys[2] = {
    .x = SCREEN_WIDTH - ALPHANUM_KEYBOARD_START_X - 50, // Canto direito (usa o define global)
    .y = controlKeys[0].y,
    .width = 50, // Largura adequada para "DEL"
    .height = ALPHANUM_KEY_HEIGHT,
    .text = BACK_TEXT,
    .color = TFT_RED,
    .textColor = TFT_WHITE,
    .textSize = 1
  };
  // GO! Button
  controlKeys[3] = {
    .x = controlKeys[2].x - 60 - ALPHANUM_KEY_SPACING_X, // À esquerda do DEL
    .y = controlKeys[0].y,
    .width = 60, // Largura adequada para "GO!"
    .height = ALPHANUM_KEY_HEIGHT,
    .text = ENTER_TEXT,
    .color = TFT_DARKGREEN,
    .textColor = TFT_WHITE,
    .textSize = 1
  };
}


// Desenha o teclado alfanumérico de acordo com o modo atual
void drawAlphaNumericKeypad() {
  // O retângulo de fundo do teclado deve cobrir toda a área do teclado
  tft.fillRect(0, ALPHANUM_KEYBOARD_START_Y - 5, SCREEN_WIDTH, SCREEN_HEIGHT - ALPHANUM_KEYBOARD_START_Y + 5, TFT_DARKCYAN); // Limpa a área do teclado
  tft.setTextColor(TFT_BLACK, TFT_LIGHTGREY);
  tft.setTextSize(1); // Tamanho da fonte das teclas

  const char** currentKeys;
  int numCurrentKeys;
  int startYOffset = ALPHANUM_KEYBOARD_START_Y; // Mantém a base Y para o teclado

  // Declare currentX e currentY aqui, fora do switch, para que tenham escopo em toda a função
  int currentX = 0;
  int currentY = 0;

  switch (currentKeypadMode) {
    case MODE_LOWERCASE:
    { // Adiciona bloco de escopo para as declarações locais
      currentKeys = LOWERCASE_KEYS;
      numCurrentKeys = NUM_LOWERCASE_KEYS;
      // Calcula a largura da linha "QWERTY" (10 teclas) e centraliza
      int row1_width_lc = 10 * ALPHANUM_KEY_WIDTH + 9 * ALPHANUM_KEY_SPACING_X;
      int row1_start_x_lc = (SCREEN_WIDTH - row1_width_lc) / 2;
      
      // Calcula a largura da linha "ASDF" (9 teclas) e centraliza
      int row2_width_lc = 9 * ALPHANUM_KEY_WIDTH + 8 * ALPHANUM_KEY_SPACING_X;
      int row2_start_x_lc = (SCREEN_WIDTH - row2_width_lc) / 2;
      
      // Calcula a largura da linha "ZXCV" (7 teclas) e centraliza
      int row3_width_lc = 7 * ALPHANUM_KEY_WIDTH + 6 * ALPHANUM_KEY_SPACING_X;
      int row3_start_x_lc = (SCREEN_WIDTH - row3_width_lc) / 2;

      for (int i = 0; i < numCurrentKeys; ++i) {
        if (i < 10) { // Primeira linha (QWERTY)
          currentX = row1_start_x_lc + i * (ALPHANUM_KEY_WIDTH + ALPHANUM_KEY_SPACING_X);
          currentY = startYOffset;
        } else if (i < 19) { // Segunda linha (ASDF)
          currentX = row2_start_x_lc + (i - 10) * (ALPHANUM_KEY_WIDTH + ALPHANUM_KEY_SPACING_X);
          currentY = startYOffset + (ALPHANUM_KEY_HEIGHT + ALPHANUM_KEY_SPACING_Y);
        } else { // Terceira linha (ZXCV)
          currentX = row3_start_x_lc + (i - 19) * (ALPHANUM_KEY_WIDTH + ALPHANUM_KEY_SPACING_X);
          currentY = startYOffset + 2 * (ALPHANUM_KEY_HEIGHT + ALPHANUM_KEY_SPACING_Y);
        }
        // Atribui os membros individualmente
        alphaNumKeys[i].x = currentX;
        alphaNumKeys[i].y = currentY;
        alphaNumKeys[i].width = ALPHANUM_KEY_WIDTH;
        alphaNumKeys[i].height = ALPHANUM_KEY_HEIGHT;
        alphaNumKeys[i].text = currentKeys[i];
        alphaNumKeys[i].color = TFT_LIGHTGREY;
        alphaNumKeys[i].textColor = TFT_BLACK;
        alphaNumKeys[i].textSize = 1;

        drawButton(alphaNumKeys[i]);
      }
      controlKeys[1].text = "123"; // Atualiza o texto do botão MODE
      break;
    } // Fim do bloco de escopo para MODE_LOWERCASE
    case MODE_UPPERCASE:
    { // Adiciona bloco de escopo para as declarações locais
      currentKeys = UPPERCASE_KEYS;
      numCurrentKeys = NUM_UPPERCASE_KEYS;
      // Cálculo de largura e centralização para maiúsculas (mesmo das minúsculas)
      int row1_width_uc = 10 * ALPHANUM_KEY_WIDTH + 9 * ALPHANUM_KEY_SPACING_X;
      int row1_start_x_uc = (SCREEN_WIDTH - row1_width_uc) / 2;
      
      int row2_width_uc = 9 * ALPHANUM_KEY_WIDTH + 8 * ALPHANUM_KEY_SPACING_X;
      int row2_start_x_uc = (SCREEN_WIDTH - row2_width_uc) / 2;
      
      int row3_width_uc = 7 * ALPHANUM_KEY_WIDTH + 6 * ALPHANUM_KEY_SPACING_X;
      int row3_start_x_uc = (SCREEN_WIDTH - row3_width_uc) / 2;

      for (int i = 0; i < numCurrentKeys; ++i) {
        if (i < 10) { // Primeira linha (QWERTY)
          currentX = row1_start_x_uc + i * (ALPHANUM_KEY_WIDTH + ALPHANUM_KEY_SPACING_X);
          currentY = startYOffset;
        } else if (i < 19) { // Segunda linha (ASDF)
          currentX = row2_start_x_uc + (i - 10) * (ALPHANUM_KEY_WIDTH + ALPHANUM_KEY_SPACING_X);
          currentY = startYOffset + (ALPHANUM_KEY_HEIGHT + ALPHANUM_KEY_SPACING_Y);
        } else { // Terceira linha (ZXCV)
          currentX = row3_start_x_uc + (i - 19) * (ALPHANUM_KEY_WIDTH + ALPHANUM_KEY_SPACING_X);
          currentY = startYOffset + 2 * (ALPHANUM_KEY_HEIGHT + ALPHANUM_KEY_SPACING_Y);
        }
        // Atribui os membros individualmente
        alphaNumKeys[i].x = currentX;
        alphaNumKeys[i].y = currentY;
        alphaNumKeys[i].width = ALPHANUM_KEY_WIDTH;
        alphaNumKeys[i].height = ALPHANUM_KEY_HEIGHT;
        alphaNumKeys[i].text = currentKeys[i];
        alphaNumKeys[i].color = TFT_LIGHTGREY;
        alphaNumKeys[i].textColor = TFT_BLACK;
        alphaNumKeys[i].textSize = 1;
        drawButton(alphaNumKeys[i]);
      }
      controlKeys[1].text = "123"; // Atualiza o texto do botão MODE
      break;
    } // Fim do bloco de escopo para MODE_UPPERCASE
    case MODE_NUMBERS:
    { // Adiciona bloco de escopo para as declarações locais
      currentKeys = NUMBER_KEYS;
      numCurrentKeys = NUM_NUMBER_KEYS;
      // Calcula a largura da linha de números (10 teclas) e centraliza
      int num_row1_width = 10 * ALPHANUM_KEY_WIDTH + 9 * ALPHANUM_KEY_SPACING_X;
      int num_row1_start_x = (SCREEN_WIDTH - num_row1_width) / 2;

      // Calcula a largura da segunda linha de símbolos (7 teclas) e centraliza
      int num_row2_width = 7 * ALPHANUM_KEY_WIDTH + 6 * ALPHANUM_KEY_SPACING_X;
      int num_row2_start_x = (SCREEN_WIDTH - num_row2_width) / 2;

      // Calcula a largura da terceira linha de símbolos (3 teclas) e centraliza
      int num_row3_width = 3 * ALPHANUM_KEY_WIDTH + 2 * ALPHANUM_KEY_SPACING_X;
      int num_row3_start_x = (SCREEN_WIDTH - num_row3_width) / 2;
      
      for (int i = 0; i < numCurrentKeys; ++i) {
        if (i < 10) { // Primeira linha (números 0-9)
          currentX = num_row1_start_x + i * (ALPHANUM_KEY_WIDTH + ALPHANUM_KEY_SPACING_X);
          currentY = startYOffset;
        } else if (i < 17) { // Segunda linha (símbolos - a @)
          currentX = num_row2_start_x + (i - 10) * (ALPHANUM_KEY_WIDTH + ALPHANUM_KEY_SPACING_X);
          currentY = startYOffset + (ALPHANUM_KEY_HEIGHT + ALPHANUM_KEY_SPACING_Y);
        } else { // Terceira linha (símbolos _ # !)
          currentX = num_row3_start_x + (i - 17) * (ALPHANUM_KEY_WIDTH + ALPHANUM_KEY_SPACING_X);
          currentY = startYOffset + 2 * (ALPHANUM_KEY_HEIGHT + ALPHANUM_KEY_SPACING_Y);
        }
        // Atribui os membros individualmente
        alphaNumKeys[i].x = currentX;
        alphaNumKeys[i].y = currentY;
        alphaNumKeys[i].width = ALPHANUM_KEY_WIDTH;
        alphaNumKeys[i].height = ALPHANUM_KEY_HEIGHT;
        alphaNumKeys[i].text = currentKeys[i];
        alphaNumKeys[i].color = TFT_LIGHTGREY;
        alphaNumKeys[i].textColor = TFT_BLACK;
        alphaNumKeys[i].textSize = 1;
        drawButton(alphaNumKeys[i]);
      }
      controlKeys[1].text = "ABC"; // Atualiza o texto do botão MODE
      break;
    } // Fim do bloco de escopo para MODE_NUMBERS
  }

  // Desenha os botões de controle
  for (int i = 0; i < 4; ++i) {
    drawButton(controlKeys[i]);
  }
}

// Desenha a tela de entrada de senha com o teclado alfanumérico
void drawPasswordInputScreen() {
  tft.fillScreen(TFT_DARKCYAN);
  tft.setTextColor(TFT_WHITE, TFT_DARKCYAN);

  // Desenha o botão Voltar na tela do teclado
  drawButton(backButtonKeypad); // Usa o novo botão específico para o teclado

  tft.setTextSize(1);
  tft.drawCentreString("SSID: " + selectedSsid, SCREEN_WIDTH / 2, 5, 1);
  tft.drawCentreString("Digite a senha:", SCREEN_WIDTH / 2, 25, 1); // Texto mais genérico

  // CAMPO DE EXIBIÇÃO DA SENHA (altura aumentada)
  #define PASSWORD_INPUT_HEIGHT 35 // Nova altura para o campo de senha
  tft.fillRect(20, 45, SCREEN_WIDTH - 40, PASSWORD_INPUT_HEIGHT, TFT_WHITE);
  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  tft.setTextSize(FONT_SIZE); // Mantém o tamanho da fonte da senha como FONT_SIZE (2)
  tft.drawCentreString(enteredPassword, SCREEN_WIDTH / 2, 45 + (PASSWORD_INPUT_HEIGHT - tft.fontHeight(FONT_SIZE)) / 2, FONT_SIZE);

  drawAlphaNumericKeypad(); // Chama a função para desenhar o teclado alfanumérico
}


void connectToWifi() {
  tft.fillScreen(TFT_BLACK); // Limpa a tela para a mensagem de conexão
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawCentreString("Conectando...", SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2 - 20, FONT_SIZE);
  tft.drawCentreString(selectedSsid.c_str(), SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2, FONT_SIZE);

  Serial.print("Tentando conectar a SSID: ");
  Serial.println(selectedSsid);
  Serial.print("Senha: ");
  Serial.println(enteredPassword);

  WiFi.mode(WIFI_STA); // Modo Estação
  WiFi.begin(selectedSsid.c_str(), enteredPassword.c_str());

  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 15000) { // Tenta por 15 segundos
    delay(500);
    Serial.print(".");
    tft.drawCentreString(".", SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2 + 30, FONT_SIZE); // Feedback visual
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Conectado com sucesso!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    tft.fillScreen(TFT_GREEN); // Mensagem de sucesso
    tft.drawCentreString("Conectado!", SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2 - 20, FONT_SIZE);
    tft.drawCentreString(WiFi.localIP().toString(), SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2, FONT_SIZE);
    saveCredentials(selectedSsid.c_str(), enteredPassword.c_str()); // Salva as credenciais

    // Inicializa o NTP após a conexão bem-sucedida para obter a hora
    timeClient.begin();
    timeClient.setTimeOffset(utcOffsetInSeconds);
    // Tenta uma atualização inicial para garantir que a hora esteja disponível imediatamente
    if (!timeClient.forceUpdate()) {
      Serial.println("Falha ao obter a hora do NTP após conexão.");
    } else {
      Serial.println("Hora NTP sincronizada após conexão.");
    }
    
    delay(3000); // Exibe a mensagem por um tempo
  } else {
    Serial.println("Falha na conexao WiFi.");
    tft.fillScreen(TFT_RED); // Mensagem de falha
    tft.drawCentreString("Falha na Conexao!", SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2, FONT_SIZE);
    delay(3000); // Exibe a mensagem por um tempo
  }

  currentState = STATE_MAIN_SCREEN; // Define o estado para a tela principal
  drawMainScreen(); // Redesenha a tela principal
  delay(100); // Pequeno atraso para garantir que a tela principal seja totalmente redesenhada
}


// --- Setup: Executado uma única vez ao ligar ou reiniciar ---
void setup() {
  Serial.begin(115200);

  // Inicializa EEPROM
  if (!EEPROM.begin(EEPROM_SIZE)) {
    Serial.println("Falha ao inicializar EEPROM. Verifique o tamanho.");
    while (true); // Trava se EEPROM não iniciar
  }

  // Inicializa o sensor DHT
  dht.begin();
  Serial.println("Sensor DHT inicializado.");

  touchscreenSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
  touchscreen.begin(touchscreenSPI);
  touchscreen.setRotation(1);

  tft.init();
  tft.setRotation(1);

  initAlphaNumericKeys(); // Inicializa as definições dos botões do teclado alfanumérico

  loadCredentials(); // Tenta carregar credenciais salvas

  // Se houver credenciais salvas, tenta conectar automaticamente e então inicia o NTP
  if (strlen(storedSsid) > 0) {
    tft.fillScreen(TFT_BLUE); // Fundo azul para a tela de conexão

    // Adiciona o texto "PARMALOG" maior acima da mensagem de conexão
    tft.setTextSize(FONT_SIZE); // Volta para FONT_SIZE padrão (tamanho 2)
    tft.setTextColor(TFT_YELLOW); // Cor TFT_YELLOW
    // Calcula a posição Y para centralizar o texto PARMALOG um pouco acima do centro da tela
    int parmalog_y = SCREEN_HEIGHT / 2 - 40; // 40 pixels acima do centro para deixar espaço
    tft.drawCentreString("PARMALOG", SCREEN_WIDTH / 2, parmalog_y, FONT_SIZE); // Desenha PARMALOG com FONT_SIZE

    // Define a cor e o tamanho do texto para a mensagem de conexão (voltando ao original)
    tft.setTextSize(1); // Garante que o tamanho é FONT_SIZE (tamanho 2)
    tft.setTextColor(TFT_YELLOW); // Cor TFT_RED
    tft.drawCentreString("Tentando conectar ao WiFi salvo...", SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2, FONT_SIZE);
    
    Serial.print("Tentando conectar a: ");
    Serial.println(storedSsid);
    WiFi.mode(WIFI_STA);
    WiFi.begin(storedSsid, storedPass);
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startTime < 10000) {
      delay(500);
      Serial.print(".");
      // O feedback visual de pontos é feito aqui, então ele vai usar a última cor de texto (TFT_RED)
      tft.drawCentreString(".", SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2 + 30, FONT_SIZE);  
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nConectado ao WiFi salvo.");
      // INICIALIZA O NTP AQUI APÓS A CONEXÃO WIFI BEM-SUCEDIDA
      timeClient.begin();
      timeClient.setTimeOffset(utcOffsetInSeconds); // Define o fuso horário

      // Force a primeira atualização da hora para que os valores não sejam zero.
      if (!timeClient.forceUpdate()) {
        Serial.println("Falha ao obter a hora do NTP na inicialização.");
      } else {
        Serial.println("Hora NTP sincronizada.");
      }

    } else {
      Serial.println("\nFalha ao conectar ao WiFi salvo. Redefina as credenciais.");
    }
  }

  // Realiza a primeira leitura do DHT para popular os valores
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();

  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Falha ao ler do sensor DHT!");
    // Você pode mostrar uma mensagem de erro no display se quiser
  } else {
    Serial.print("Umidade: "); Serial.print(humidity); Serial.print(" %\t");
    Serial.print("Temperatura: "); Serial.print(temperature); Serial.println(" *C");
  }

  drawMainScreen(); // Desenha a tela principal com os valores iniciais do DHT
}

// --- Loop: Executado continuamente ---
void loop() {
  // Variáveis declaradas ANTES do switch para evitar o erro "jump to case label"
  // Definindo as variáveis da área de toque da lista de SSIDs e rolagem
  int list_text_area_width = SCREEN_WIDTH - 10 - SCROLL_ARROW_WIDTH - 5; // Largura real disponível para o texto do SSID na lista
  int local_ssid_item_height = tft.fontHeight(FONT_SIZE) + 4; // Altura de cada item da lista
  int local_max_visible_ssids = (SCREEN_HEIGHT - 25 - backButton.height - 10) / local_ssid_item_height;

  // As coordenadas X da área de toque da lista de SSIDs
  int list_touch_area_startX = 0; // Começa na borda esquerda da tela
  int list_touch_area_endX = list_text_area_width; // Termina onde a área da seta começa

  // As coordenadas Y da área de toque da lista de SSIDs
  int list_touch_area_startY_ssid = 28; // Abaixo do cabeçalho da lista de SSIDs
  // A área de clique da lista de SSIDs vai até o final da área visível de SSIDs, antes do botão Voltar
  int list_touch_area_endY_ssid = list_touch_area_startY_ssid + (local_max_visible_ssids * local_ssid_item_height);

  // Lógica de leitura do DHT, apenas se estiver na tela principal
  if (currentState == STATE_MAIN_SCREEN && millis() - lastDHTReadTime >= DHT_READ_INTERVAL) {
    lastDHTReadTime = millis(); // Atualiza o tempo da última leitura

    // Lê a umidade e temperatura
    float newHumidity = dht.readHumidity();
    float newTemperature = dht.readTemperature();

    // Verifica se a leitura foi bem-sucedida
    if (isnan(newHumidity) || isnan(newTemperature)) {
      Serial.println("Falha ao ler do sensor DHT!");
      // Opcional: mostrar erro no display
    } else {
      // Atualiza as variáveis globais
      humidity = newHumidity;
      temperature = newTemperature;

      Serial.print("Umidade: "); Serial.print(humidity); Serial.print(" %\t");
      Serial.print("Temperatura: "); Serial.print(temperature); Serial.println(" *C");

      // Redesenha a tela principal para mostrar os novos valores
      drawMainScreen();
    }
  }


  if (touchscreen.tirqTouched() && touchscreen.touched()) {
    TS_Point p = touchscreen.getPoint();

    x = map(p.x, touchMinX, touchMaxX, 0, SCREEN_WIDTH);
    y = map(p.y, touchMinY, touchMaxY, 0, SCREEN_HEIGHT);
    z = p.z;

    if (x < 0) x = 0; if (x >= SCREEN_WIDTH) x = SCREEN_WIDTH - 1;
    if (y < 0) y = 0; if (y >= SCREEN_HEIGHT) y = SCREEN_HEIGHT - 1;

    // printTouchToSerial(x, y, z); // Descomente para ver todos os toques na serial

    // Lógica de estado da máquina
    switch (currentState) {
      case STATE_MAIN_SCREEN:
        if (isButtonPressed(menuButton, x, y)) {
          Serial.println("Botao MENU pressionado!");
          currentState = STATE_SETUP_MENU;
          drawSetupMenu();
        }
        break;

      case STATE_SETUP_MENU:
        if (isButtonPressed(wifiConfigButton, x, y)) {
          Serial.println("Botao Config WiFi pressionado! Iniciando scan...");
          currentState = STATE_WIFI_SCAN;
          performWifiScan();
        } else if (isButtonPressed(backButton, x, y)) {
          Serial.println("Botao Voltar do Menu pressionado!");
          currentState = STATE_MAIN_SCREEN;
          drawMainScreen();
        }
        break;

      case STATE_WIFI_SCAN:
        // *** Lógica de toque para os botões de rolagem explícitos ***
        if (isButtonPressed(scrollUpButton, x, y)) {
            if (currentScrollOffset > 0) {
              currentScrollOffset--;
              drawWifiList(); // Redesenha a lista
            }
        } else if (isButtonPressed(scrollDownButton, x, y)) {
            if (currentScrollOffset + local_max_visible_ssids < numNetworks) {
              currentScrollOffset++;
              drawWifiList(); // Redesenha a lista
            }
            // Importante: Adicionar um pequeno delay após a rolagem para evitar cliques duplos
            // e permitir que o display se atualize completamente.
            // delay(100); // Já tem um delay no final do loop, mas aqui pode ser útil se for muito rápido
        }
        // *** FIM DA LÓGICA DE TOQUE PARA ROLAGEM ***

        // Lógica de toque para seleção de SSID (APENAS na área da lista de texto)
        if (x >= list_touch_area_startX && x < list_touch_area_endX &&
            y >= list_touch_area_startY_ssid && y < list_touch_area_endY_ssid) {
          
          int touchedItem = (y - list_touch_area_startY_ssid) / local_ssid_item_height;
          int selectedNetworkIndex = currentScrollOffset + touchedItem;

          if (selectedNetworkIndex >= 0 && selectedNetworkIndex < numNetworks) {
            selectedSsid = ssids[selectedNetworkIndex];
            enteredPassword = ""; // Limpa a senha anterior
            Serial.print("SSID selecionado: ");
            Serial.println(selectedSsid);
            currentState = STATE_WIFI_INPUT;
            drawPasswordInputScreen();
          }
        }
        
        // Lógica para o botão Voltar (separada)
        if (isButtonPressed(backButton, x, y)) {
          Serial.println("Botao Voltar do Scan pressionado!");
          currentState = STATE_SETUP_MENU;
          drawSetupMenu();
        }
        break;

      case STATE_WIFI_INPUT:
        // *** Lógica para o botão Voltar na tela do teclado ***
        if (isButtonPressed(backButtonKeypad, x, y)) {
          Serial.println("Botao Voltar do Teclado pressionado!");
          currentState = STATE_WIFI_SCAN; // Volta para a tela de scan de WiFi
          performWifiScan(); // Redesenha a lista de Wi-Fi
        }
        // FIM DA LÓGICA DO BOTÃO VOLTAR

        // Lógica dos botões de controle (SHIFT, MODE, DEL, GO!)
        if (isButtonPressed(controlKeys[0], x, y)) { // SHIFT
          Serial.println("Botao SHIFT pressionado!");
          if (currentKeypadMode == MODE_LOWERCASE) {
            currentKeypadMode = MODE_UPPERCASE;
            controlKeys[0].color = TFT_GREEN; // Mudar cor para indicar CAPS ativo
          } else if (currentKeypadMode == MODE_UPPERCASE) {
            currentKeypadMode = MODE_LOWERCASE;
            controlKeys[0].color = TFT_DARKGREY; // Voltar cor
          }
          drawPasswordInputScreen(); // Redesenha o teclado
        } else if (isButtonPressed(controlKeys[1], x, y)) { // MODE (123 / ABC)
          Serial.println("Botao MODE pressionado!");
          if (currentKeypadMode == MODE_NUMBERS) {
            currentKeypadMode = MODE_LOWERCASE;
            controlKeys[1].text = "123"; // Mudar texto para "123"
            controlKeys[0].color = TFT_DARKGREY; // Reseta cor do SHIFT
          } else {
            currentKeypadMode = MODE_NUMBERS;
            controlKeys[1].text = "ABC"; // Mudar texto para "ABC"
            // Se mudar para números, desativa o CAPS
            controlKeys[0].color = TFT_DARKGREY;  
          }
          drawPasswordInputScreen(); // Redesenha o teclado
        } else if (isButtonPressed(controlKeys[2], x, y)) { // DEL (Backspace)
          if (enteredPassword.length() > 0) {
            enteredPassword.remove(enteredPassword.length() - 1);
            drawPasswordInputScreen();
            Serial.print("Senha: "); Serial.println(enteredPassword);
          }
        } else if (isButtonPressed(controlKeys[3], x, y)) { // GO! (Conectar)
          Serial.println("Botao Conectar (WiFi) pressionado!");
          connectToWifi();
        } else { // Lógica para as teclas alfanuméricas
          // Determina o número de teclas a serem verificadas
          int numKeysToCheck;
          if (currentKeypadMode == MODE_LOWERCASE || currentKeypadMode == MODE_UPPERCASE) {
            numKeysToCheck = NUM_LOWERCASE_KEYS; // Ambas têm o mesmo número de letras
          } else { // MODE_NUMBERS
            numKeysToCheck = NUM_NUMBER_KEYS;
          }

          for (int i = 0; i < numKeysToCheck; ++i) {
            // Verifica se as coordenadas do toque estão dentro dos limites do botão
            if (alphaNumKeys[i].text != nullptr && // Adiciona verificação para evitar acesso a ponteiro nulo (segurança)
                x >= alphaNumKeys[i].x && x <= (alphaNumKeys[i].x + alphaNumKeys[i].width) &&
                y >= alphaNumKeys[i].y && y <= (alphaNumKeys[i].y + alphaNumKeys[i].height)) {
              
              if (enteredPassword.length() < MAX_PASS_LEN) {
                enteredPassword += alphaNumKeys[i].text;
                drawPasswordInputScreen();
                Serial.print("Senha: "); Serial.println(enteredPassword);
              }
              break; // Sai do loop após encontrar a tecla pressionada
            }
          }
        }
        break; // Fim do case STATE_WIFI_INPUT
    }

    while (touchscreen.touched()) delay(10);
    delay(50);
  }
}
