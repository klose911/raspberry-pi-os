#include "utils.h"
#include "uart.h"
#include "peripherals/uart.h"
#include "peripherals/gpio.h"

static void bzero(void*, int);

static void bzero(void * dest, int bytes) {
        char * d = dest;
        while (bytes--) {
                *d++ = 0;
        }
}

void uart_init()
{
        uart_control_t control;
        // Disable UART0.
        bzero(&control, 4);
        put32(UART0_CR, control.as_int);

        // Setup the GPIO pin 14 && 15.
        // Disable pull up/down for all GPIO pins & delay for 150 cycles.
        put32(GPPUD, 0x00000000);
        delay(150);

        // Disable pull up/down for pin 14,15 & delay for 150 cycles.
        put32(GPPUDCLK0, (1 << 14) | (1 << 15));
        delay(150);

        // Write 0 to GPPUDCLK0 to make it take effect.
        put32(GPPUDCLK0, 0x00000000);

        // Clear pending interrupts.
        put32(UART0_ICR, 0x7FF);

        // Set integer & fractional part of baud rate.
        // Divider = UART_CLOCK/(16 * Baud)
        // Fraction part register = (Fractional part * 64) + 0.5
        // UART_CLOCK = 3000000; Baud = 115200.

        // Divider = 3000000 / (16 * 115200) = 1.627 = ~1.
        put32(UART0_IBRD, 1);
        // Fractional part register = (.627 * 64) + 0.5 = 40.6 = ~40.
        put32(UART0_FBRD, 40);

        // Enable FIFO & 8 bit data transmissio (1 stop bit, no parity).
        put32(UART0_LCRH, (1 << 4) | (1 << 5) | (1 << 6));

        // Mask all interrupts.
        put32(UART0_IMSC, (1 << 1) | (1 << 4) | (1 << 5) | (1 << 6) |
              (1 << 7) | (1 << 8) | (1 << 9) | (1 << 10));

        // Enable UART0, receive & transfer part of UART.
        control.uart_enabled = 1;
        control.transmit_enabled = 1;
        control.receive_enabled = 1;
        put32(UART0_CR, control.as_int);
}


uart_flags_t read_flags(void) {
        uart_flags_t flags;
        flags.as_int = get32(UART0_FR);
        return flags;
}

void uart_putc(unsigned char c)
{
        uart_flags_t flags;
        // Wait for UART to become ready to transmit.

        do {
                flags = read_flags();
        }
        while ( flags.transmit_queue_full );
        put32(UART0_DR, c);
}

unsigned char uart_getc()
{
        // Wait for UART to have received something.
        uart_flags_t flags;
        do {
                flags = read_flags();
        }
        while ( flags.recieve_queue_empty );
        return get32(UART0_DR);
}

void uart_puts(const char* str) 
{
        for(int i = 0; str[i] != '\0'; i++) {
                uart_putc((char) str[i]);
        }
        
}

