#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"

// uart0-tx has gp0 (default)
// uart0-rx has gp1 (default)
// uart1-tx can go to pins gp4 and gp8
// uart1-rx can go to pins gp5 and gp9
#define UART_ID uart1
#define BAUD_RATE 115200

// We are using pins 0 and 1, but see the GPIO function select table in the
// datasheet for information on which other pins can be used.
#define UART_TX_PIN 8
#define UART_RX_PIN 9

int main() {
    // Set up our UART with the required speed.
    uart_init(UART_ID, BAUD_RATE);

    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Use some the various UART functions to send out data
    // In a default system, printf will also output via the default UART

    while(true)
    {
        // Send out a string, with CR/LF conversions
        sleep_ms(500);
        uart_puts(UART_ID, "Hello, UART!\r\n");
    }

}
