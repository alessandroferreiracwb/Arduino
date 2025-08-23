#include "driver/ledc.h"

// Define the PWM parameters
#define PWM_FREQUENCY       (500)
#define PWM_DUTY_RESOLUTION LEDC_TIMER_10_BIT // 10 bits of resolution (0-1023)
#define PWM_PIN             (2)
#define PWM_TIMER           LEDC_TIMER_0
#define PWM_CHANNEL         LEDC_CHANNEL_0
#define PWM_MODE            LEDC_LOW_SPEED_MODE

void setup() {
    // --- 1. Configure the PWM Timer ---
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = PWM_MODE,
        .duty_resolution  = PWM_DUTY_RESOLUTION,
        .timer_num        = PWM_TIMER,
        .freq_hz          = PWM_FREQUENCY,
    };
    ledc_timer_config(&ledc_timer);

    // --- 2. Configure the PWM Channel ---
    ledc_channel_config_t ledc_channel = {
        .gpio_num       = PWM_PIN,
        .speed_mode     = PWM_MODE,
        .channel        = PWM_CHANNEL,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = PWM_TIMER,
        .duty           = 0, // Start with 0% duty cycle
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);
}

void loop() {
    // --- 3. Main loop to vary the duty cycle ---
    int duty_value = 0;
    int step = 10;
    
    // Increase the duty cycle
    for (duty_value = 0; duty_value <= 1023; duty_value += step) {
        ledc_set_duty(PWM_MODE, PWM_CHANNEL, duty_value);
        ledc_update_duty(PWM_MODE, PWM_CHANNEL);
        delay(100); // Using standard Arduino delay
    }

    // Decrease the duty cycle
    for (duty_value = 1023; duty_value >= 0; duty_value -= step) {
        ledc_set_duty(PWM_MODE, PWM_CHANNEL, duty_value);
        ledc_update_duty(PWM_MODE, PWM_CHANNEL);
        delay(100);
    }
}
