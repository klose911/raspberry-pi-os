#include <stddef.h>
#include <stdint.h>

#ifndef UART_H
#define UART_H

typedef union uart_flags {
        struct {
                uint8_t clear_to_send: 1;
                uint8_t data_set_ready: 1;
                uint8_t data_carrier_detected: 1;
                uint8_t busy: 1;
                uint8_t recieve_queue_empty: 1;
                uint8_t transmit_queue_full: 1;
                uint8_t recieve_queue_full: 1;
                uint8_t transmit_queue_empty: 1;
                uint8_t ring_indicator: 1;
                uint32_t padding: 23;
        };
        uint32_t as_int;
} uart_flags_t;

typedef union uart_control {
        struct {
                uint8_t uart_enabled: 1;
                uint8_t sir_enabled: 1;
                uint8_t sir_low_power_mode: 1;
                uint8_t reserved: 4;
                uint8_t loop_back_enabled: 1;
                uint8_t transmit_enabled: 1;
                uint8_t receive_enabled: 1;
                uint8_t data_transmit_ready: 1;
                uint8_t request_to_send: 1;
                uint8_t out1: 1;
                uint8_t out2: 1;
                uint8_t rts_hardware_flow_control_enabled: 1;
                uint8_t cts_hardware_flow_control_enabled: 1;
                uint16_t padding;
        };
        uint32_t as_int;
} uart_control_t;

void uart_init( void );

uart_flags_t read_flags(void);

void uart_putc(unsigned char c);

unsigned char uart_getc();

void uart_puts(const char* str);
#endif
