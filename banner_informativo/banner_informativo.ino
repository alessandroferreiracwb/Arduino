#include <WiFi.h>
#include <TFT_eSPI.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <UniversalTelegramBot.h>
#include <WiFiClientSecure.h>
#include "time.h"

// Definições da tela
#define TFT_BLACK 0x0000
#define TFT_WHITE 0xFFFF
#define TFT_GREEN 0x07E0
#define TFT_RED 0xF800
#define TFT_LIGHTBLUE 0x04BF
TFT_eSPI tft = TFT_eSPI();

// Credenciais WiFi
const char* ssid = "ALESSANDRO";
const char* password = "98291490";

// Configuração NTP (Hora)
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", -10800, 60000);

// Configuração do Telegram
#define BOT_TOKEN "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"
#define CHAT_ID "1234567890"
WiFiClientSecure clientTelegram;
UniversalTelegramBot bot(BOT_TOKEN, clientTelegram);
long lastUpdateID = 0;

// Variáveis para armazenar os últimos dados
float lastTemp = 0.0;
int lastHumidity = 0;
float lastWindSpeed = 0.0;
float lastWindDirection = 0.0;
float lastCotacaoDolar = 0.0;
String lastTelegramMessage = "";
String lastInspirationalQuote = "";
String lastQuoteAuthor = "";
bool hasWeatherData = false;
bool hasDolarData = false;
bool hasTelegramData = false;
bool hasInspirationalQuoteData = false;
String lastBibleVerse = "";
String lastBibleReference = "";
bool hasBibleData = false;

// Array com versículos bíblicos em português
const String biblicalVerses[] = {
  "O Senhor e o meu pastor; nada me faltara. - Salmos 23:1",
  "Tudo posso naquele que me fortalece. - Filipenses 4:13",
  "O amor e paciente, o amor e bondoso. Nao inveja, nao se vangloria, nao se orgulha. - 1 Corintios 13:4",
  "Confie no Senhor de todo o seu coracao e nao se apoie em seu proprio entendimento. - Proverbios 3:5",
  "Porque Deus tanto amou o mundo que deu o seu Filho Unigenito, para que todo o que nele crer nao pereca, mas tenha a vida eterna. - Joao 3:16",
  "Nao se deixem vencer pelo mal, mas vencam o mal com o bem. - Romanos 12:21",
  "Alegrem-se sempre no Senhor. Novamente direi: alegrem-se! - Filipenses 4:4"
};
const int numVerses = sizeof(biblicalVerses) / sizeof(biblicalVerses[0]);


// Variáveis para controle de tempo das atualizações
unsigned long lastTimeWeatherUpdated = 0;
const long intervalWeather = 5 * 60 * 1000;

unsigned long lastTimeDolarUpdated = 0;
const long intervalDolar = 30 * 60 * 1000;

unsigned long lastTimeTelegramUpdated = 0;
const long intervalTelegram = 10 * 1000;

unsigned long lastTimeQuoteUpdated = 0;
const long intervalQuote = 15 * 60 * 1000;

unsigned long lastTimeBibleUpdated = 0;
const long intervalBible = 60 * 60 * 1000;

// Variáveis para controle da transição de telas
int currentScreen = 0;
unsigned long lastScreenChange = 0;
long currentScreenDuration = 5000;

// Variáveis para o gerenciamento de Wi-Fi
bool isConnecting = false;
unsigned long lastWiFiReconnectAttempt = 0;
const long reconnectInterval = 10000;

// --- Funções de exibição ---

void exibirStatusWifi(uint16_t bgcolor) {
  tft.fillRect(0, 0, tft.width(), 20, TFT_BLACK);
  
  tft.setTextSize(1);
  tft.setCursor(260, 5);
  
  if (WiFi.status() == WL_CONNECTED) {
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.println("WIFI ON");
  } else {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.println("WIFI OFF");
  }
}

void drawTelaConectando() {
  tft.fillScreen(TFT_WHITE);
  tft.setTextColor(TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(10, 80);
  tft.println("Conectando ao Wi-Fi...");
}

// Funcao para remover acentos e substituir 'ç'
String removerAcentos(String texto) {
  texto.replace("á", "a");
  texto.replace("à", "a");
  texto.replace("ã", "a");
  texto.replace("â", "a");
  texto.replace("é", "e");
  texto.replace("ê", "e");
  texto.replace("í", "i");
  texto.replace("ó", "o");
  texto.replace("õ", "o");
  texto.replace("ô", "o");
  texto.replace("ú", "u");
  texto.replace("ü", "u");
  texto.replace("ç", "c");
  return texto;
}

// Funcao para codificar a URL
String urlEncodeString(String str) {
  String encodedString = "";
  char c;
  char code0;
  char code1;
  char code2;
  for (int i = 0; i < str.length(); i++) {
    c = str.charAt(i);
    if (c == ' ') {
      encodedString += '+';
    } else if (isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') {
      encodedString += c;
    } else {
      code1 = (c & 0xf) + '0';
      if ((c & 0xf) > 9) {
        code1 = (c & 0xf) - 10 + 'A';
      }
      c = (c >> 4) & 0xf;
      code0 = c + '0';
      if (c > 9) {
        code0 = c - 10 + 'A';
      }
      encodedString += '%';
      encodedString += code0;
      encodedString += code1;
    }
  }
  return encodedString;
}

// --- Funções de atualização em segundo plano ---

void atualizarTempo() {
  if (WiFi.status() != WL_CONNECTED) return;

  Serial.println("Iniciando a requisicao da API de tempo...");
  HTTPClient http;
  String url = "https://api.open-meteo.com/v1/forecast?latitude=-25.4284&longitude=-49.2733&current_weather=true&hourly=temperature_2m,relative_humidity_2m&forecast_days=1";
  http.begin(url);
  int httpCode = http.GET();
  
  if (httpCode == HTTP_CODE_OK) {
    String payload = http.getString();
    DynamicJsonDocument doc(2048);
    deserializeJson(doc, payload);

    lastTemp = doc["current_weather"]["temperature"];
    lastHumidity = doc["hourly"]["relative_humidity_2m"][0];
    lastWindSpeed = doc["current_weather"]["wind_speed"];
    lastWindDirection = doc["current_weather"]["wind_direction"];
    hasWeatherData = true;
    Serial.println("Dados de tempo recebidos com sucesso.");
  } else {
    Serial.printf("Erro na consulta de tempo. Código HTTP: %d\n", httpCode);
  }
  http.end();
  lastTimeWeatherUpdated = millis();
}

void atualizarDolar() {
  if (WiFi.status() != WL_CONNECTED) return;

  Serial.println("Iniciando a requisicao da API do dolar...");
  HTTPClient http;
  String url = "https://economia.awesomeapi.com.br/last/USD-BRL";
  http.begin(url);
  int httpCode = http.GET();
  
  if (httpCode == HTTP_CODE_OK) {
    String payload = http.getString();
    DynamicJsonDocument doc(1024);
    deserializeJson(doc, payload);

    String cotacaoStr = doc["USDBRL"]["bid"];
    lastCotacaoDolar = cotacaoStr.toFloat();
    hasDolarData = true;
    Serial.println("Dados do dolar recebidos com sucesso.");
  } else {
    Serial.printf("Erro na consulta do dólar. Código HTTP: %d\n", httpCode);
  }
  http.end();
  lastTimeDolarUpdated = millis();
}

void atualizarCitacaoInspiracional() {
  if (WiFi.status() != WL_CONNECTED) return;
  
  Serial.println("Iniciando a requisicao da API de citacao inspiracional...");
  
  HTTPClient https;
  https.begin(clientTelegram, "https://zenquotes.io/api/random");
  int httpCode = https.GET();

  if (httpCode == HTTP_CODE_OK) {
    String payload = https.getString();
    DynamicJsonDocument doc(2048);
    deserializeJson(doc, payload);

    if (doc[0].containsKey("q")) {
      String englishQuote = doc[0]["q"].as<String>();
      lastQuoteAuthor = doc[0]["a"].as<String>();

      HTTPClient translateHttp;
      String encodedQuote = urlEncodeString(englishQuote);
      String translateURL = "https://api.mymemory.translated.net/get?q=" + encodedQuote + "&langpair=en|pt";
      translateHttp.begin(translateURL);
      int translateHttpCode = translateHttp.GET();

      if (translateHttpCode == HTTP_CODE_OK) {
        String translatePayload = translateHttp.getString();
        DynamicJsonDocument translateDoc(2048);
        deserializeJson(translateDoc, translatePayload);
        lastInspirationalQuote = translateDoc["responseData"]["translatedText"].as<String>();
        lastInspirationalQuote = removerAcentos(lastInspirationalQuote);
        hasInspirationalQuoteData = true;
        Serial.println("Citacao inspiracional traduzida com sucesso.");
      } else {
        Serial.printf("Erro na traducao da citacao. Codigo HTTP: %d\n", translateHttpCode);
      }
      translateHttp.end();

    } else {
      Serial.println("Erro: A resposta da API nao contem a citacao.");
    }
  } else {
    Serial.printf("Erro na consulta de citacao. Codigo HTTP: %d\n", httpCode);
  }
  https.end();
  lastTimeQuoteUpdated = millis();
}

void atualizarBiblia() {
  Serial.println("Atualizando versiculo da Biblia (do array local)...");

  randomSeed(millis());
  int randomIndex = random(0, numVerses);
  String fullVerse = biblicalVerses[randomIndex];
  
  int dashIndex = fullVerse.lastIndexOf('-');
  lastBibleVerse = fullVerse.substring(0, dashIndex - 1);
  lastBibleReference = fullVerse.substring(dashIndex + 1);

  hasBibleData = true;
  Serial.println("Versiculo da Biblia selecionado com sucesso.");
}


void handleNewMessages(int numNewMessages) {
  Serial.printf("Processando %d novas mensagens do Telegram...\n", numNewMessages);
  for (int i=0; i<numNewMessages; i++) {
    String chatID = String(bot.messages[i].chat_id);
    Serial.printf("ID do chat da mensagem recebida: %s\n", chatID.c_str());
    if (chatID == CHAT_ID) {
      String text = bot.messages[i].text;
      lastTelegramMessage = text;
      hasTelegramData = true;
      lastUpdateID = bot.messages[i].update_id;
      Serial.printf("Nova mensagem recebida: %s\n", lastTelegramMessage.c_str());
    }
  }
}

void atualizarTelegram() {
  if (WiFi.status() != WL_CONNECTED) return;
  
  Serial.println("Verificando novas mensagens no Telegram...");
  int numNewMessages = bot.getUpdates(lastUpdateID + 1);
  if (numNewMessages > 0) {
    handleNewMessages(numNewMessages);
  }
  lastTimeTelegramUpdated = millis();
}

// --- Funções para desenhar as telas ---

void drawTelaHora() {
  tft.fillScreen(TFT_LIGHTBLUE);
  exibirStatusWifi(TFT_LIGHTBLUE);
  timeClient.update();
  
  // Aumento do tamanho do texto e alinhado a esquerda
  String timeString = timeClient.getFormattedTime();
  String timeOnly = timeString.substring(0, 5);
  
  tft.setTextSize(7);
  tft.setTextColor(TFT_BLACK);
  tft.setCursor(20, 80);
  tft.println(timeOnly);

  tft.setTextSize(2);
  tft.setCursor(20, 150);
  
  time_t epochTime = timeClient.getEpochTime();
  struct tm *timeinfo = localtime(&epochTime);
  
  const char *diasDaSemana[] = {"Dom", "Seg", "Ter", "Qua", "Qui", "Sex", "Sab"};
  
  tft.printf("%s, %02d/%02d/%04d\n", diasDaSemana[timeinfo->tm_wday], timeinfo->tm_mday, timeinfo->tm_mon + 1, timeinfo->tm_year + 1900);
}

void drawTelaTempo() {
  tft.fillScreen(TFT_LIGHTBLUE);
  exibirStatusWifi(TFT_LIGHTBLUE);
  
  tft.setTextSize(2);
  tft.setTextColor(TFT_BLACK, TFT_LIGHTBLUE);
  tft.setCursor(10, 30);
  tft.println("Curitiba - Tempo Agora");
  
  if (hasWeatherData) {
    tft.setTextSize(3);
    tft.setCursor(10, 80);
    tft.printf("Temp: %.1f C", lastTemp);
    
    tft.setCursor(10, 130);
    tft.printf("Umid: %d %%", lastHumidity);
  } else {
    tft.setTextSize(2);
    tft.setTextColor(TFT_RED, TFT_LIGHTBLUE);
    tft.setCursor(10, 80);
    tft.println("Erro ao carregar");
    tft.setCursor(10, 100);
    tft.println("dados. Verifique");
    tft.setCursor(10, 120);
    tft.println("conexao Wi-Fi.");
  }
}

void drawTelaCitacaoInspiracional() {
  tft.fillScreen(TFT_LIGHTBLUE);
  exibirStatusWifi(TFT_LIGHTBLUE);
  
  tft.setTextSize(2);
  tft.setTextColor(TFT_BLACK, TFT_LIGHTBLUE);
  
  if (hasInspirationalQuoteData) {
    tft.setCursor(10, 30);
    int cursorY = 30;
    int maxChars = tft.width() / 12 - 2;
    for (int i = 0; i < lastInspirationalQuote.length(); i += maxChars) {
      String sub = lastInspirationalQuote.substring(i, i + maxChars);
      tft.setCursor(10, cursorY);
      tft.println(sub);
      cursorY += 20;
    }
    tft.setCursor(10, cursorY + 20);
    tft.printf("- %s", lastQuoteAuthor.c_str());
  } else {
    tft.setCursor(10, 80);
    tft.println("Carregando citacao...");
  }
}

void drawTelaDolar() {
  tft.fillScreen(TFT_LIGHTBLUE);
  exibirStatusWifi(TFT_LIGHTBLUE);

  tft.setTextSize(3);
  tft.setTextColor(TFT_BLACK);
  tft.setCursor(10, 80);
  tft.println("Dolar (USD):");
  
  if (hasDolarData) {
    tft.setCursor(10, 130);
    tft.printf("R$ %.2f", lastCotacaoDolar);
  } else {
    tft.setTextSize(2);
    tft.setTextColor(TFT_RED);
    tft.setCursor(10, 130);
    tft.println("Erro ao carregar");
    tft.setCursor(10, 150);
    tft.println("dados. Verifique");
    tft.setCursor(10, 170);
    tft.println("conexao Wi-Fi.");
  }
}

void drawTelaTelegram() {
  tft.fillScreen(TFT_LIGHTBLUE);
  exibirStatusWifi(TFT_LIGHTBLUE);

  tft.setTextSize(2);
  tft.setTextColor(TFT_BLACK, TFT_LIGHTBLUE);
  tft.setCursor(10, 30);
  tft.println("Mensagem Telegram:");
  
  if (hasTelegramData) {
    tft.setCursor(10, 80);
    int cursorY = 80;
    int maxChars = tft.width() / 12 - 2;
    for (int i = 0; i < lastTelegramMessage.length(); i += maxChars) {
      String sub = lastTelegramMessage.substring(i, i + maxChars);
      tft.setCursor(10, cursorY);
      tft.println(sub);
      cursorY += 20;
    }
  } else {
    tft.setCursor(10, 80);
    tft.println("Aguardando mensagem...");
  }
}

void drawTelaBiblia() {
  tft.fillScreen(TFT_LIGHTBLUE);
  exibirStatusWifi(TFT_LIGHTBLUE);

  tft.setTextSize(2);
  tft.setTextColor(TFT_BLACK, TFT_LIGHTBLUE);

  if (hasBibleData) {
    tft.setCursor(10, 30);
    tft.println(lastBibleReference);
    
    tft.setCursor(10, 60);
    int cursorY = 60;
    int maxChars = tft.width() / 12 - 2;
    for (int i = 0; i < lastBibleVerse.length(); i += maxChars) {
      String sub = lastBibleVerse.substring(i, i + maxChars);
      tft.setCursor(10, cursorY);
      tft.println(sub);
      cursorY += 20;
    }
  } else {
    tft.setCursor(10, 80);
    tft.println("Erro ao carregar");
    tft.setCursor(10, 100);
    tft.println("o versiculo.");
  }
}


// --- Funções de configuração e loop ---

void setup() {
  Serial.begin(115200);
  Serial.println("Iniciando...");
  
  clientTelegram.setInsecure();

  tft.init();
  tft.setRotation(1);

  drawTelaConectando();
  
  Serial.print("Iniciando conexao Wi-Fi...");
  WiFi.begin(ssid, password);
  isConnecting = true;
  randomSeed(millis());
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    if (isConnecting) {
      Serial.println("\nConectado ao Wi-Fi com sucesso!");
      Serial.printf("Endereco IP: %s\n", WiFi.localIP().toString().c_str());
      timeClient.begin();
      atualizarTempo();
      atualizarDolar();
      atualizarCitacaoInspiracional();
      atualizarTelegram();
      atualizarBiblia();
      lastScreenChange = millis();
      isConnecting = false;
    }
  } else {
    if (!isConnecting) {
      isConnecting = true;
      lastWiFiReconnectAttempt = millis();
      drawTelaConectando();
    }
    if (millis() - lastWiFiReconnectAttempt > reconnectInterval) {
      Serial.println("Wi-Fi desconectado! Tentando reconectar...");
      WiFi.reconnect();
      lastWiFiReconnectAttempt = millis();
    }
    return;
  }

  if (millis() - lastTimeWeatherUpdated >= intervalWeather) {
    atualizarTempo();
  }

  if (millis() - lastTimeDolarUpdated >= intervalDolar) {
    atualizarDolar();
  }
  
  if (millis() - lastTimeTelegramUpdated >= intervalTelegram) {
    atualizarTelegram();
  }

  if (millis() - lastTimeQuoteUpdated >= intervalQuote) {
    atualizarCitacaoInspiracional();
  }
  
  if (millis() - lastTimeBibleUpdated >= intervalBible) {
    atualizarBiblia();
  }
  
  if (millis() - lastScreenChange >= currentScreenDuration) {
    bool nextScreenFound = false;
    int screensToSkip = 0;
    do {
      currentScreen++;
      if (currentScreen > 5) {
        currentScreen = 0;
      }
      
      switch (currentScreen) {
        case 0:
        case 1:
          nextScreenFound = true;
          break;
        case 2:
          if (hasInspirationalQuoteData) {
            nextScreenFound = true;
            atualizarCitacaoInspiracional();
          } else {
            Serial.println("Pulando tela de Citacao Inspiracional por falta de dados.");
            screensToSkip++;
          }
          break;
        case 3:
          if (hasDolarData) {
            nextScreenFound = true;
          } else {
            Serial.println("Pulando tela de Dolar por falta de dados.");
            screensToSkip++;
          }
          break;
        case 4:
          if (hasBibleData) {
            nextScreenFound = true;
            atualizarBiblia();
          } else {
            Serial.println("Pulando tela da Biblia por falta de dados.");
            screensToSkip++;
          }
          break;
        case 5:
          if (hasTelegramData) {
            nextScreenFound = true;
          } else {
            Serial.println("Pulando tela de Telegram por falta de dados.");
            screensToSkip++;
          }
          break;
      }
      if (screensToSkip >= 4) {
        nextScreenFound = true;
      }
    } while (!nextScreenFound);

    switch (currentScreen) {
      case 0:
        currentScreenDuration = 5000;
        drawTelaHora();
        break;
      case 1:
        currentScreenDuration = 5000;
        drawTelaTempo();
        break;
      case 2:
        if (hasInspirationalQuoteData) {
            int wordCount = 0;
            if (lastInspirationalQuote.length() > 0) {
              bool inWord = false;
              for(int i = 0; i < lastInspirationalQuote.length(); i++) {
                  if (isspace(lastInspirationalQuote.charAt(i))) {
                      inWord = false;
                  } else if (!inWord) {
                      inWord = true;
                      wordCount++;
                  }
              }
            }
            
            long calculatedDuration = 5000 + (wordCount / 3) * 1000; // Tempo de leitura aumentado
            if (calculatedDuration > 20000) calculatedDuration = 20000;
            currentScreenDuration = calculatedDuration;
        } else {
            currentScreenDuration = 3000;
        }
        drawTelaCitacaoInspiracional();
        break;
      case 3:
        if (hasDolarData) {
          currentScreenDuration = 5000;
        } else {
          currentScreenDuration = 3000;
        }
        drawTelaDolar();
        break;
      case 4:
        if (hasBibleData) {
          int wordCount = 0;
            if (lastBibleVerse.length() > 0) {
              bool inWord = false;
              for(int i = 0; i < lastBibleVerse.length(); i++) {
                  if (isspace(lastBibleVerse.charAt(i))) {
                      inWord = false;
                  } else if (!inWord) {
                      inWord = true;
                      wordCount++;
                  }
              }
            }
            long calculatedDuration = 5000 + (wordCount / 3) * 1000; // Tempo de leitura aumentado
            if (calculatedDuration > 20000) calculatedDuration = 20000;
            currentScreenDuration = calculatedDuration;
        } else {
            currentScreenDuration = 3000;
        }
        drawTelaBiblia();
        break;
      case 5:
        if (hasTelegramData) {
          currentScreenDuration = 5000;
        } else {
          currentScreenDuration = 3000;
        }
        drawTelaTelegram();
        break;
    }
    lastScreenChange = millis();
  }
}
