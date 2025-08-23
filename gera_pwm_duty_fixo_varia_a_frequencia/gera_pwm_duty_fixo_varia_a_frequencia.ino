#include "driver/ledc.h"

// Define os parâmetros fixos do PWM
#define PWM_DUTY_RESOLUTION LEDC_TIMER_10_BIT // 10 bits de resolução (0-1023)
#define PWM_DUTY_VALUE      (712)            // 50% de 1023 (1023/2 = 511.5), usando 512
#define PWM_PIN             (2)
#define PWM_TIMER           LEDC_TIMER_0
#define PWM_CHANNEL         LEDC_CHANNEL_0
#define PWM_MODE            LEDC_LOW_SPEED_MODE

// Define os parâmetros de frequência
#define PWM_FREQ_START_HZ   (60)
#define PWM_FREQ_STEP_HZ    (10)
#define PWM_FREQ_MAX_HZ     (1000)

void setup() {
    // --- 1. Configura o Timer com a frequência inicial ---
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = PWM_MODE,
        .duty_resolution  = PWM_DUTY_RESOLUTION,
        .timer_num        = PWM_TIMER,
        .freq_hz          = PWM_FREQ_START_HZ, // Frequência inicial
    };
    ledc_timer_config(&ledc_timer);

    // --- 2. Configura o Canal com o duty cycle fixo ---
    ledc_channel_config_t ledc_channel = {
        .gpio_num       = PWM_PIN,
        .speed_mode     = PWM_MODE,
        .channel        = PWM_CHANNEL,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = PWM_TIMER,
        .duty           = PWM_DUTY_VALUE, // Duty cycle fixo em 50%
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);
    
    // Atualiza o duty cycle no canal para começar com 50%
    ledc_update_duty(PWM_MODE, PWM_CHANNEL);
}

void loop() {
    // Fica na frequência inicial por 2 segundos
    ledc_set_freq(PWM_MODE, PWM_TIMER, PWM_FREQ_START_HZ);
    delay(2000); // 2 segundos

    // Aumenta a frequência de 10 em 10 Hz até o valor máximo
    for (int freq_hz = PWM_FREQ_START_HZ; freq_hz <= PWM_FREQ_MAX_HZ; freq_hz += PWM_FREQ_STEP_HZ) {
        ledc_set_freq(PWM_MODE, PWM_TIMER, freq_hz);
        delay(100); // Pequeno atraso entre as mudanças de frequência
    }
}
