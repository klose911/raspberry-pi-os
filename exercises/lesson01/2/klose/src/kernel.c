#include "uart.h"

void kernel_main(void)
{
        /* mini_uart_init();  */
        /* mini_uart_send_string("Hello, world!\r\n");  */

        /*  while (1) {  */
        /*  	mini_uart_send(mini_uart_recv());  */
        /*  }  */

        uart_init();
        uart_puts("Hello, world!\r\n");

        while (1) {
                uart_putc(uart_getc());
        }

}
