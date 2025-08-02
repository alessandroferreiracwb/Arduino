#include <SPI.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>

// --- Definições de Pinos e Calibração ---
#define TFT_CS 15
#define TFT_DC 2
#define TFT_RST -1
#define TFT_BL 21 

#define XPT2046_IRQ 36
#define XPT2046_MOSI 32
#define XPT2046_MISO 39
#define XPT2046_CLK 25
#define XPT2046_CS 33

int touchMinX = 451;
int touchMaxX = 3598;
int touchMinY = 600;
int touchMaxY = 3579;
// ----------------------------------------

TFT_eSPI tft = TFT_eSPI();
SPIClass touchscreenSPI = SPIClass(VSPI);
XPT2046_Touchscreen ts(XPT2046_CS, XPT2046_IRQ);

#define LEDC_CHANNEL 0
#define LEDC_RESOLUTION 8
#define LEDC_FREQUENCY 5000

struct Button {
  int x;
  int y;
  int width;
  int height;
  const char* text;
  uint16_t color;
  uint16_t textColor;
  int brightnessValue;
};

Button brightnessButtons[] = {
  {10, 80, 50, 50, "10", TFT_RED, TFT_WHITE, 20},
  {75, 80, 50, 50, "25", TFT_ORANGE, TFT_WHITE, 64},
  {140, 80, 50, 50, "50", TFT_YELLOW, TFT_BLACK, 128},
  {205, 80, 50, 50, "75", TFT_GREEN, TFT_WHITE, 191},
  {270, 80, 50, 50, "100", TFT_CYAN, TFT_BLACK, 255}
};

// *** A função drawButton foi modificada para centralizar o texto ***
void drawButton(Button btn) {
  tft.fillRect(btn.x, btn.y, btn.width, btn.height, btn.color);
  tft.drawRect(btn.x, btn.y, btn.width, btn.height, TFT_BLACK); // Borda
  tft.setTextColor(btn.textColor);
  tft.setTextSize(2);
  
  // Calcula as coordenadas centrais do botão
  int centerX = btn.x + btn.width / 2;
  int centerY = btn.y + btn.height / 5;
  
  // Desenha o texto centralizado
  tft.drawCentreString(btn.text, centerX, centerY, 2);
}

void setup() {
  Serial.begin(115200);

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  touchscreenSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
  ts.begin(touchscreenSPI);
  ts.setRotation(1);

  ledcSetup(LEDC_CHANNEL, LEDC_FREQUENCY, LEDC_RESOLUTION);
  ledcAttachPin(TFT_BL, LEDC_CHANNEL);

  ledcWrite(LEDC_CHANNEL, 25);
  
  for (int i = 0; i < 5; i++) {
    drawButton(brightnessButtons[i]);
  }
  
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  
  tft.drawCentreString("Controle de Brilho", tft.width() / 2, 20, 2);
}

void loop() {
  if (ts.touched()) {
    TS_Point p = ts.getPoint();
    uint16_t touch_x = map(p.x, touchMinX, touchMaxX, 0, tft.width());
    uint16_t touch_y = map(p.y, touchMinY, touchMaxY, 0, tft.height());

    for (int i = 0; i < 5; i++) {
      if (touch_x >= brightnessButtons[i].x && touch_x <= (brightnessButtons[i].x + brightnessButtons[i].width) &&
          touch_y >= brightnessButtons[i].y && touch_y <= (brightnessButtons[i].y + brightnessButtons[i].height)) {
        
        Serial.print("Botao '");
        Serial.print(brightnessButtons[i].text);
        Serial.println("' pressionado.");
        
        ledcWrite(LEDC_CHANNEL, brightnessButtons[i].brightnessValue);
      }
    }
    
    while (ts.touched()) {
      delay(10);
    }
  }
}
