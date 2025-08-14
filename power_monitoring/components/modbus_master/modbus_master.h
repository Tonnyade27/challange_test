#ifndef MODBUS_MASTER_H
#define MODBUS_MASTER_H

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

// Konfigurasi RS485
typedef struct {
    uart_port_t uart_num;
    int tx_pin;
    int rx_pin;
    int dere_pin;       // DE/RE pin untuk transceiver (MAX485). Set -1 jika tidak dipakai
    int baudrate;
    uart_parity_t parity;   // UART_PARITY_DISABLE / EVEN / ODD sesuai alat
    int timeout_ms;     // timeout baca respon
} modbus_port_t;

// Inisialisasi UART + RS485
esp_err_t modbus_init(const modbus_port_t *cfg);

// Read Holding Registers  (0x03): baca 'qty' register 16-bit mentah
esp_err_t modbus_read_regs(uart_port_t uart_num, int dere_pin,
                           uint8_t slave_addr, uint16_t start_reg,
                           uint16_t qty, uint16_t *out_regs);

// Helper: baca 2 register jadi float (big-endian word order by default)
// Jika alat kamu pakai urutan word terbalik, set swap_words=true
esp_err_t modbus_read_float(uart_port_t uart_num, int dere_pin,
                            uint8_t slave_addr, uint16_t start_reg,
                            bool swap_words, float *out_val);

#endif
