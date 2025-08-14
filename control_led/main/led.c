#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"

typedef struct {
    gpio_num_t pin;
    int64_t interval_ms;
    int64_t last_toggle;
    bool state;
} led_t;

#define LED_COUNT 5

led_t leds[LED_COUNT] = {
    {GPIO_NUM_2, 270, 0, false},   // Merah
    {GPIO_NUM_4, 440, 0, false},   // Kuning
    {GPIO_NUM_5, 710, 0, false},   // Hijau
    {GPIO_NUM_18, 1330, 0, false}, // Biru
    {GPIO_NUM_19, 1850, 0, false}  // Putih
};

void init_leds() {
    for (int i = 0; i < LED_COUNT; i++) {
        gpio_reset_pin(leds[i].pin);
        gpio_set_direction(leds[i].pin, GPIO_MODE_OUTPUT);
        gpio_set_level(leds[i].pin, 0);
        leds[i].last_toggle = esp_timer_get_time() / 1000; // millis
    }
}

void led_task(void *arg) {
    while (1) {
        int64_t now = esp_timer_get_time() / 1000; // millis
        for (int i = 0; i < LED_COUNT; i++) {
            if ((now - leds[i].last_toggle) >= leds[i].interval_ms) {
                leds[i].state = !leds[i].state;
                gpio_set_level(leds[i].pin, leds[i].state);
                leds[i].last_toggle = now;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}

void app_main(void) {
    init_leds();
    xTaskCreate(led_task, "led_task", 2048, NULL, 5, NULL);
}
