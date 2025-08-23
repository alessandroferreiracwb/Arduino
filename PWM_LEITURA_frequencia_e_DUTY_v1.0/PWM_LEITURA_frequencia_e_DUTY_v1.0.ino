// versao do ESP32 2.0.17
#include <Arduino.h>

#define PULSE_INPUT_PIN 27

// Variaveis volateis para a interrupcao
volatile uint64_t lastRisingEdgeMicros = 0;
volatile uint64_t lastFallingEdgeMicros = 0;

volatile uint64_t measuredHighTime = 0;
volatile uint64_t measuredPeriod = 0;

volatile bool newMeasurementReady = false;

// Rotina de interrupcao que eh chamada na borda de subida e descida
void IRAM_ATTR onPulseChange() {
    uint64_t currentTime = micros();
    
    // Borda de subida (sinal de LOW para HIGH)
    if (digitalRead(PULSE_INPUT_PIN) == HIGH) {
        // Calcula o periodo como o tempo entre duas bordas de subida consecutivas
        if (lastRisingEdgeMicros > 0) {
            measuredPeriod = currentTime - lastRisingEdgeMicros;
        }
        lastRisingEdgeMicros = currentTime;
    } 
    // Borda de descida (sinal de HIGH para LOW)
    else {
        // Calcula o tempo HIGH como o tempo entre a subida e a descida
        if (lastRisingEdgeMicros > 0) {
            measuredHighTime = currentTime - lastRisingEdgeMicros;
            newMeasurementReady = true; // Sinaliza que a medicão esta completa
        }
        lastFallingEdgeMicros = currentTime;
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("Medindo Frequencia e Ciclo de Trabalho com Interrupcoes...");
    
    // Configura o pino como entrada
    pinMode(PULSE_INPUT_PIN, INPUT_PULLDOWN);
    
    // Anexa a interrupcao para qualquer mudanca de estado
    attachInterrupt(digitalPinToInterrupt(PULSE_INPUT_PIN), onPulseChange, CHANGE);
}

void loop() {
    if (newMeasurementReady) {
        // Bloco atomico para ler as variaveis volateis de forma segura
        noInterrupts();
        uint64_t tempHighTime = measuredHighTime;
        uint64_t tempPeriod = measuredPeriod;
        newMeasurementReady = false;
        interrupts();

        if (tempPeriod > 0) {
            double frequencyHz = 1000000.0 / tempPeriod;
            double dutyCycle = (double)tempHighTime / (double)tempPeriod * 100.0;
            
            Serial.print("Frequencia: ");
            Serial.print(frequencyHz, 1);
            Serial.print(" Hz | ");
            Serial.print("Ciclo de Trabalho: ");
            Serial.print(dutyCycle, 2);
            Serial.println(" %");
        }
    }
}
