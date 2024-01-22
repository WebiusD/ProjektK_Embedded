#include "pico/stdlib.h"


int main() {
    const uint valve_pin = 25;

    // initialize led pin:
    gpio_init(valve_pin);
    gpio_set_dir(valve_pin, GPIO_OUT);

    // loop:
    while(true) {
        gpio_put(valve_pin, true);
        sleep_ms(3000);
        gpio_put(valve_pin, false);
        sleep_ms(3000);
    }

}