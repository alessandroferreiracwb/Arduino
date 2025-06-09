//#include <ledc.h>

const int ledcChannel = 0; // Choose a channel (0-15)
const int ledPin = 2;     // Choose a GPIO pin for PWM output
const int frequency = 200000;   // 100 kHz frequency
const int resolution = 8;     // 8-bit resolution

void setup() {
  ledcSetup(ledcChannel, frequency, resolution);
  ledcAttachPin(ledPin, ledcChannel);
}

void loop() {
  // Change the duty cycle to control the PWM signal
  ledcWrite(ledcChannel, 127);
//  for (int duty = 0; duty <= 255; duty++) {
//    ledcWrite(ledcChannel, duty);
//    delay(10);
//  }
//  for (int duty = 255; duty >= 0; duty--) {
//    ledcWrite(ledcChannel, duty);
//    delay(10);
//  }
}
