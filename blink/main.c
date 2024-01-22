#include "pico/stdlib.h"


int main() {
    const uint led_pin = 24;

/*     // initialize led pin:
    gpio_init(led_pin);
    gpio_set_dir(led_pin, GPIO_OUT);

    // loop:
    while(true) {
        gpio_put(led_pin, true);
        sleep_ms(500);
        gpio_put(led_pin, false);
        sleep_ms(500);
    } */

    // set uart pins as inputs to make them high impedance:
    gpio_init(8);
    gpio_init(9);
    gpio_set_dir(8, GPIO_IN);
    gpio_set_dir(9, GPIO_IN);

}