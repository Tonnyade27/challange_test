#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "mqtt_client.h"
#include "cJSON.h"

#define TAG "NODE"

// RS485
#define UART_PORT UART_NUM_1
#define TX_PIN 17
#define RX_PIN 16
#define DE_RE_PIN 4
#define BAUD_RATE 9600

// FAN
#define FAN_GPIO 25

// WiFi & MQTT
#define WIFI_SSID "Home2_4G"
#define WIFI_PASS "123456789"
#define MQTT_BROKER "mqtt://192.168.160.168"

// Global vars
float voltage, current, power, temperature;
esp_mqtt_client_handle_t client;

// RS485 DE/RE Control
void rs485_set_tx(bool tx_mode) {
    gpio_set_level(DE_RE_PIN, tx_mode ? 1 : 0);
}

// CRC16 Modbus
uint16_t crc16_modbus(const uint8_t *buf, int len) {
    uint16_t crc = 0xFFFF;
    for (int pos = 0; pos < len; pos++) {
        crc ^= buf[pos];
        for (int i = 0; i < 8; i++) {
            if (crc & 1) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// Modbus Read Float
bool modbus_read_float(uint8_t slave_addr, uint16_t reg_addr, float *out_val) {
    uint8_t req[8];
    req[0] = slave_addr;
    req[1] = 0x03; // Read Holding Registers
    req[2] = reg_addr >> 8;
    req[3] = reg_addr & 0xFF;
    req[4] = 0x00;
    req[5] = 0x02; // 2 register = float
    uint16_t crc = crc16_modbus(req, 6);
    req[6] = crc & 0xFF;
    req[7] = crc >> 8;

    uart_flush(UART_PORT);
    rs485_set_tx(true);
    uart_write_bytes(UART_PORT, (const char *)req, 8);
    uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(20));
    rs485_set_tx(false);

    uint8_t resp[9];
    int len = uart_read_bytes(UART_PORT, resp, sizeof(resp), pdMS_TO_TICKS(200));
    if (len != 9) {
        ESP_LOGW(TAG, "Short/No response (got %d / want 9)", len);
        return false;
    }

    uint16_t crc_recv = resp[7] | (resp[8] << 8);
    uint16_t crc_calc = crc16_modbus(resp, 7);
    if (crc_recv != crc_calc) {
        ESP_LOGE(TAG, "CRC mismatch");
        return false;
    }

    uint16_t raw_hi = (resp[3] << 8) | resp[4];
    uint16_t raw_lo = (resp[5] << 8) | resp[6];
    uint32_t raw = ((uint32_t)raw_hi << 16) | raw_lo;
    memcpy(out_val, &raw, sizeof(float));
    return true;
}

// FAN Control
void control_fan() {
    if (temperature > 27.0 * 1.02) {
        gpio_set_level(FAN_GPIO, 1);
    } else {
        gpio_set_level(FAN_GPIO, 0);
    }
}

// MQTT Publish
void publish_mqtt() {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "voltage", voltage);
    cJSON_AddNumberToObject(root, "current", current);
    cJSON_AddNumberToObject(root, "power", power);
    cJSON_AddNumberToObject(root, "temperature", temperature);

    char *payload = cJSON_PrintUnformatted(root);
    esp_mqtt_client_publish(client, "DATA/LOCAL/SENSOR/PANEL_1", payload, 0, 1, 0);
    ESP_LOGI(TAG, "Publish: %s", payload);

    cJSON_Delete(root);
    free(payload);
}

// MQTT Event
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    switch (event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT Connected");
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT Disconnected, reconnecting...");
            ESP_LOGW(TAG, "MQTT Disconnected, retry in 10s...");
            vTaskDelay(pdMS_TO_TICKS(10000));
            esp_mqtt_client_reconnect(client);
            break;
    }
}

// WiFi Event
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi Disconnected, retry in 10s...");
        vTaskDelay(pdMS_TO_TICKS(10000));
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(TAG, "📶 WiFi got IP");
    }
}

// WiFi Init
void wifi_init() {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    esp_wifi_start();
    esp_wifi_connect();
}

// Main
void app_main() {
    nvs_flash_init();
    wifi_init();

    gpio_set_direction(FAN_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(DE_RE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(FAN_GPIO, 0);
    gpio_set_level(DE_RE_PIN, 0);

    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_driver_install(UART_PORT, 256, 256, 0, NULL, 0);
    uart_param_config(UART_PORT, &uart_config);
    uart_set_pin(UART_PORT, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER
    };
    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);

    while (1) {
        if (!modbus_read_float(1, 0x0000, &voltage)) voltage = -1;
        if (!modbus_read_float(1, 0x0002, &current)) current = -1;
        if (!modbus_read_float(1, 0x0004, &power))   power   = -1;
        if (!modbus_read_float(2, 0x0000, &temperature)) temperature = -1;

        control_fan();
        publish_mqtt();

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
