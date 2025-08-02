#include <SPI.h>              
#include <TFT_eSPI.h>         
#include <XPT2046_Touchscreen.h> 
#include <math.h> 

// --- Definição de cores personalizadas ---
#define TFT_GRAY 0x8410 

// --- Definições de Pinos e Calibração ---
#define XPT2046_IRQ 36
#define XPT2046_MOSI 32
#define XPT2046_MISO 39
#define XPT2046_CLK 25
#define XPT2046_CS 33

// Valores de calibração do Touch gerados pelo seu display
int touchMinX = 348;
int touchMaxX = 3613;
int touchMinY = 469;
int touchMaxY = 3612;

// --- Definição das cores para o botão ---
#define COR_EXTERNA TFT_GRAY
#define COR_INTERNA_LIGADO TFT_GREEN
#define COR_INTERNA_DESLIGADO TFT_RED
#define COR_INTERNA_PRESSIONADO TFT_ORANGE // Adicionamos a cor laranja

// --- Declarações Globais de Objetos e Variáveis ---
TFT_eSPI tft = TFT_eSPI();                         
SPIClass touchscreenSPI = SPIClass(VSPI);          
XPT2046_Touchscreen ts(XPT2046_CS, XPT2046_IRQ); 

bool buttonState = false; 
bool buttonPressed = false;
bool drawingPressedState = false; // Flag para rastrear se o estado pressionado já foi desenhado

#define BUTTON_RADIUS 50
int button_x_center;
int button_y_center;

// --- Funções de desenho e lógica ---

// Desenha o botão redondo na tela
void drawButton() {
  tft.fillScreen(TFT_WHITE); // Fundo branco

  button_x_center = tft.width() / 2;
  button_y_center = tft.height() / 2;

  tft.fillCircle(button_x_center, button_y_center, 30, COR_EXTERNA);

  if (buttonState) {
    tft.fillCircle(button_x_center, button_y_center, 22, COR_INTERNA_LIGADO);
  } else {
    tft.fillCircle(button_x_center, button_y_center, 22, COR_INTERNA_DESLIGADO);
  }
}

// --- SETUP ---
void setup() {
  Serial.begin(115200);

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_WHITE);

  touchscreenSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
  ts.begin(touchscreenSPI);
  ts.setRotation(1);

  drawButton();
}

// --- LOOP ---
void loop() {
  uint16_t touch_x, touch_y;

  if (ts.touched()) {
    TS_Point p = ts.getPoint();
    
    touch_x = map(p.y, touchMaxY, touchMinY, 0, tft.width());
    touch_y = map(p.x, touchMaxX, touchMinX, 0, tft.height());

    if (pow(touch_x - button_x_center, 2) + pow(touch_y - button_y_center, 2) <= pow(30, 2)) {
      if (!buttonPressed) { // Desenha o estado pressionado apenas na primeira vez
        tft.fillCircle(button_x_center, button_y_center, 22, COR_INTERNA_PRESSIONADO);
        buttonPressed = true;
      }
    }
  } 
  else {
    if (buttonPressed) {
      buttonState = !buttonState;
      Serial.print("Botao foi ");
      Serial.println(buttonState ? "LIGADO" : "DESLIGADO");
      drawButton(); // Redesenha com a cor final
      buttonPressed = false;
    }
  }
  delay(10);
}
