#include "modbus_master.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "MODBUS_MASTER";

static uint16_t crc16_modbus(const uint8_t *buf, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; i++) {
        crc ^= buf[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
            else              crc >>= 1;
        }
    }
    return crc;
}

static inline void rs485_set_tx(int dere_pin, bool en) {
    if (dere_pin >= 0) gpio_set_level(dere_pin, en ? 1 : 0); // DE/RE high=TX, low=RX
}

esp_err_t modbus_init(const modbus_port_t *cfg) {
    uart_config_t uc = {
        .baud_rate = cfg->baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = cfg->parity,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    ESP_ERROR_CHECK(uart_param_config(cfg->uart_num, &uc));
    ESP_ERROR_CHECK(uart_set_pin(cfg->uart_num, cfg->tx_pin, cfg->rx_pin,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(cfg->uart_num, 512, 512, 0, NULL, 0));

    if (cfg->dere_pin >= 0) {
        gpio_config_t io = {
            .pin_bit_mask = 1ULL << cfg->dere_pin,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = 0, .pull_down_en = 0, .intr_type = GPIO_INTR_DISABLE
        };
        ESP_ERROR_CHECK(gpio_config(&io));
        rs485_set_tx(cfg->dere_pin, false); // default RX
    }
    ESP_LOGI(TAG, "UART%d init @%d, parity=%d, DE/RE=%d, timeout=%dms",
             cfg->uart_num, cfg->baudrate, cfg->parity, cfg->dere_pin, cfg->timeout_ms);
    return ESP_OK;
}

esp_err_t modbus_read_regs(uart_port_t uart_num, int dere_pin,
                           uint8_t slave_addr, uint16_t start_reg,
                           uint16_t qty, uint16_t *out_regs) {
    uint8_t req[8];
    req[0] = slave_addr;
    req[1] = 0x03; // Read Holding Registers
    req[2] = (start_reg >> 8) & 0xFF;
    req[3] = start_reg & 0xFF;
    req[4] = (qty >> 8) & 0xFF;
    req[5] = qty & 0xFF;
    uint16_t crc = crc16_modbus(req, 6);
    req[6] = crc & 0xFF;       // CRC low
    req[7] = (crc >> 8) & 0xFF;// CRC high

    uart_flush(uart_num);
    rs485_set_tx(dere_pin, true);
    uart_write_bytes(uart_num, (const char*)req, sizeof(req));
    uart_wait_tx_done(uart_num, pdMS_TO_TICKS(20));
    rs485_set_tx(dere_pin, false);

    // Resp: addr(1) fc(1) bytecount(1) data(2*qty) crc(2)
    int expect_len = 5 + 2 * qty;
    uint8_t resp[256];
    int rlen = uart_read_bytes(uart_num, resp, expect_len, pdMS_TO_TICKS(250));
    if (rlen < expect_len) {
        ESP_LOGW(TAG, "Short/No response (got %d / want %d)", rlen, expect_len);
        return ESP_FAIL;
    }

    uint16_t crc_calc = crc16_modbus(resp, rlen - 2);
    uint16_t crc_recv = resp[rlen - 2] | (resp[rlen - 1] << 8);
    if (crc_calc != crc_recv) {
        ESP_LOGE(TAG, "CRC mismatch");
        return ESP_FAIL;
    }
    if (resp[0] != slave_addr || resp[1] != 0x03) {
        ESP_LOGE(TAG, "Bad addr/fc (%u/%u)", resp[0], resp[1]);
        return ESP_FAIL;
    }
    uint8_t bc = resp[2];
    if (bc != 2 * qty) {
        ESP_LOGE(TAG, "Bad byte count %u (qty=%u)", bc, qty);
        return ESP_FAIL;
    }

    for (int i = 0; i < qty; i++) {
        out_regs[i] = (resp[3 + 2*i] << 8) | resp[4 + 2*i];
    }
    return ESP_OK;
}

esp_err_t modbus_read_float(uart_port_t uart_num, int dere_pin,
                            uint8_t slave_addr, uint16_t start_reg,
                            bool swap_words, float *out_val) {
    uint16_t regs[2] = {0};
    esp_err_t err = modbus_read_regs(uart_num, dere_pin, slave_addr, start_reg, 2, regs);
    if (err != ESP_OK) return err;

    // Default: big-endian word order: regs[0]=MSW, regs[1]=LSW
    uint32_t raw;
    if (!swap_words) raw = ((uint32_t)regs[0] << 16) | regs[1];
    else             raw = ((uint32_t)regs[1] << 16) | regs[0];

    memcpy(out_val, &raw, sizeof(float));
    return ESP_OK;
}
