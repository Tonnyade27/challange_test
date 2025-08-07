#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "modbus_master.h"
#include "cJSON.h"

#define TAG "NODE"

#define UART_PORT UART_NUM_1
#define TX_PIN 17
#define RX_PIN 16
#define BAUD_RATE 9600

#define FAN_GPIO 25
#define MQTT_BROKER "mqtt://192.168.1.100"  // Local Mosquitto

float voltage, current, power, temperature;
uint32_t last_read_time = 0;

esp_mqtt_client_handle_t client;

void read_modbus_data() {
    modbus_read_float(UART_PORT, 0x01, 0x0000, &voltage);   
    modbus_read_float(UART_PORT, 0x01, 0x0002, &current);
    modbus_read_float(UART_PORT, 0x01, 0x0004, &power);
    modbus_read_float(UART_PORT, 0x02, 0x0000, &temperature); 
}

void control_fan() {
    if (temperature > 27.0 * 1.02) {
        gpio_set_level(FAN_GPIO, 1);
    } else {
        gpio_set_level(FAN_GPIO, 0);
    }
}

void publish_mqtt() {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "voltage", voltage);
    cJSON_AddNumberToObject(root, "current", current);
    cJSON_AddNumberToObject(root, "power", power);
    cJSON_AddNumberToObject(root, "temperature", temperature);

    char *payload = cJSON_PrintUnformatted(root);
    esp_mqtt_client_publish(client, "DATA/LOCAL/SENSOR/PANEL_1", payload, 0, 1, 0);
    cJSON_Delete(root);
    free(payload);
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
}

void app_main() {

    gpio_set_direction(FAN_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(FAN_GPIO, 0);


    modbus_init(UART_PORT, TX_PIN, RX_PIN, BAUD_RATE);

 
    esp_mqtt_client_config_t mqtt_cfg = { .uri = MQTT_BROKER };
    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);

    while (1) {
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (now - last_read_time >= 1000) {
            last_read_time = now;

            read_modbus_data();    
            control_fan();          
            publish_mqtt();         
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);  
    }
}
