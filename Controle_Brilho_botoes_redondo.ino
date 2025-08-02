#include <SPI.h>              // Inclui a biblioteca para comunicação SPI
#include <TFT_eSPI.h>         // Inclui a biblioteca para controlar o display TFT
#include <XPT2046_Touchscreen.h> // Inclui a biblioteca para o controlador de touchscreen XPT2046

// --- Definições de Pinos e Calibração ---
// Pinos do seu display ESP32-2432S028, conforme o seu User_Setup.h
#define TFT_CS 15             // Pino Chip Select (CS) do TFT
#define TFT_DC 2              // Pino Data/Command (DC) do TFT
#define TFT_RST -1            // Pino de Reset (RST), use -1 se estiver fixado no VCC
#define TFT_BL 21             // Pino do Backlight (luz de fundo), será controlado por PWM

// Pinos do Touchscreen, conforme o seu código funcional
#define XPT2046_IRQ 36        // Pino de Interrupção do Touch, pode ser usado para eficiência
#define XPT2046_MOSI 32       // Pino MOSI do Touch, usado para enviar dados para o chip
#define XPT2046_MISO 39       // Pino MISO do Touch, usado para receber dados do chip
#define XPT2046_CLK 25        // Pino de Clock do Touch
#define XPT2046_CS 33         // Pino Chip Select (CS) do Touch

// Valores de calibração do Touch: estes são os valores brutos mínimo e máximo
// que o touch retorna. Eles são usados para mapear o toque para as coordenadas da tela.
int touchMinX = 451;
int touchMaxX = 3598;
int touchMinY = 600;
int touchMaxY = 3579;
// ----------------------------------------

// Cria os objetos das bibliotecas
TFT_eSPI tft = TFT_eSPI();                         // Objeto para o display TFT
SPIClass touchscreenSPI = SPIClass(VSPI);          // Objeto SPI para o touchscreen
XPT2046_Touchscreen ts(XPT2046_CS, XPT2046_IRQ); // Objeto para o controlador de toque

// Variáveis para o controle de brilho via PWM do ESP32 (LEDC)
#define LEDC_CHANNEL 0           // Canal LEDC (0 a 15) que será usado
#define LEDC_RESOLUTION 8        // Resolução do PWM (8 bits = 0 a 255)
#define LEDC_FREQUENCY 5000      // Frequência do PWM (5 kHz)

// --- Estrutura para os Botões Redondos ---
struct Button {
  int x;          // Posição central X do botão
  int y;          // Posição central Y do botão
  int radius;     // Raio do círculo
  const char* text;   // Texto dentro do botão
  uint16_t color;     // Cor de fundo do botão
  uint16_t textColor; // Cor do texto do botão
  int brightnessValue; // Valor de brilho (PWM) associado a este botão
};

// Definição dos 5 botões de brilho redondos
Button brightnessButtons[] = {
  // Posição (X, Y) do centro, Raio, Texto, Cor do botão, Cor do texto, Valor de brilho
  {50, 105, 25, "10", TFT_RED, TFT_WHITE, 25},
  {105, 105, 25, "25", TFT_ORANGE, TFT_WHITE, 64},
  {160, 105, 25, "50", TFT_YELLOW, TFT_BLACK, 128},
  {215, 105, 25, "75", TFT_GREEN, TFT_WHITE, 191},
  {270, 105, 25, "100", TFT_CYAN, TFT_BLACK, 255} 
};

// *** Função para desenhar um botão redondo ***
void drawButton(Button btn) {
  tft.fillCircle(btn.x, btn.y, btn.radius, btn.color); // Desenha o círculo preenchido
  tft.drawCircle(btn.x, btn.y, btn.radius, TFT_BLACK); // Desenha a borda do círculo
  tft.setTextColor(btn.textColor);                     // Define a cor do texto
  tft.setTextSize(2);                                  // Define o tamanho do texto
  
  // Desenha o texto centralizado, movendo-o 15 pixels para cima
  tft.drawCentreString(btn.text, btn.x, btn.y - 15, 2);
}

// *** Função para verificar o toque em um botão redondo ***
bool isTouchInCircle(Button btn, int touchX, int touchY) {
  // Usa a fórmula da distância (distância^2 < raio^2 para evitar cálculo de raiz quadrada)
  int dx = touchX - btn.x;
  int dy = touchY - btn.y;
  return (dx * dx + dy * dy) < (btn.radius * btn.radius);
}

void setup() {
  Serial.begin(115200); // Inicia a comunicação serial para depuração

  tft.init();         // Inicializa o display TFT
  tft.setRotation(1); // Define a rotação da tela (1 = paisagem)
  tft.fillScreen(TFT_BLACK); // Limpa a tela com a cor preta
  tft.setTextColor(TFT_WHITE, TFT_BLACK); // Define a cor padrão do texto

  // Inicializa o Touchscreen
  touchscreenSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
  ts.begin(touchscreenSPI);
  ts.setRotation(1); // Importante: a rotação do touch deve ser a mesma do display

  // --- Configuração do PWM para o Backlight ---
  ledcSetup(LEDC_CHANNEL, LEDC_FREQUENCY, LEDC_RESOLUTION); // Configura o canal LEDC
  ledcAttachPin(TFT_BL, LEDC_CHANNEL);                   // Associa o canal ao pino do backlight

  ledcWrite(LEDC_CHANNEL, 128); // Define o brilho inicial em 50% (valor 128)
  
  // Loop para desenhar todos os botões na tela
  for (int i = 0; i < 5; i++) {
    drawButton(brightnessButtons[i]);
  }
  
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  
  // Desenha o título centralizado na parte superior da tela
  tft.drawCentreString("Controle de Brilho", tft.width() / 2, 20, 2);
}

void loop() {
  // Verifica se o touch foi pressionado
  if (ts.touched()) {
    TS_Point p = ts.getPoint(); // Obtém as coordenadas brutas do toque

    // Mapeia as coordenadas brutas para as dimensões da tela (calibração)
    uint16_t touch_x = map(p.x, touchMinX, touchMaxX, 0, tft.width());
    uint16_t touch_y = map(p.y, touchMinY, touchMaxY, 0, tft.height());

    // Loop para verificar qual botão foi tocado
    for (int i = 0; i < 5; i++) {
      // Usa a função personalizada para verificar se o toque está dentro do círculo
      if (isTouchInCircle(brightnessButtons[i], touch_x, touch_y)) {
        
        Serial.print("Botao '");
        Serial.print(brightnessButtons[i].text);
        Serial.println("' pressionado.");
        
        // Ativa o brilho correspondente
        ledcWrite(LEDC_CHANNEL, brightnessButtons[i].brightnessValue);
      }
    }
    
    // Aguarda o dedo ser levantado para evitar múltiplos toques
    while (ts.touched()) {
      delay(10);
    }
  }
}
