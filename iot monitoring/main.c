#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "modbus_master.h"
#include "cJSON.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"

#define TAG "NODE"

// ---------------- WiFi Config ----------------
#define WIFI_SSID "Home2_4G"
#define WIFI_PASS "123456789"

// ---------------- RS485 Config ----------------
#define UART_PORT UART_NUM_1
#define TX_PIN 17
#define RX_PIN 16
#define BAUD_RATE 9600

// ---------------- GPIO ----------------
#define FAN_GPIO 25

// ---------------- MQTT Config ----------------
#define MQTT_BROKER "mqtt://192.168.160.168"  // IP RPi

float voltage, current, power, temperature;
uint32_t last_read_time = 0;
esp_mqtt_client_handle_t client;

// ---------------- Modbus Read ----------------
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

// ---------------- MQTT Event ----------------
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    // Bisa tambahkan log di sini
}

// ---------------- WiFi Init ----------------
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(TAG, "📶 WiFi Connected, Got IP");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi Disconnected, reconnecting...");
        esp_wifi_connect();
    }
}

void wifi_init_sta(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "WiFi init finished.");
}

// ---------------- Main ----------------
void app_main() {
    // Init NVS (Wajib sebelum WiFi)
    ESP_ERROR_CHECK(nvs_flash_init());

    // Init WiFi
    wifi_init_sta();

    // Init GPIO Fan
    gpio_set_direction(FAN_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(FAN_GPIO, 0);

    // Init RS485 Modbus
    modbus_init(UART_PORT, TX_PIN, RX_PIN, BAUD_RATE);

    // Init MQTT
    esp_mqtt_client_config_t mqtt_cfg = { .uri = MQTT_BROKER };
    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);

    // Main Loop
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
