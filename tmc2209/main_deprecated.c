#include "pico/stdlib.h"

const uint TMC_EN = 10;
const uint PDN_PIN = 8;
const uint STP_PIN = 11;
const uint DIR_PIN = 12;
const uint LED_PIN = 24;

int main() {
    gpio_init(TMC_EN);
    gpio_init(PDN_PIN);
    gpio_init(STP_PIN);
    gpio_init(DIR_PIN);
    gpio_init(LED_PIN);

    gpio_set_dir(TMC_EN, GPIO_OUT);
    gpio_set_dir(PDN_PIN, GPIO_OUT);
    gpio_set_dir(STP_PIN, GPIO_OUT);
    gpio_set_dir(DIR_PIN, GPIO_OUT);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // enable the tmc:
    gpio_put(TMC_EN, 0);

    // pull pdn and dir low:
    gpio_put(PDN_PIN, 0);
    gpio_put(DIR_PIN, 0);

    uint count = 0;
    bool led_state = false;

    while(true) {
        gpio_put(STP_PIN, 0);
        sleep_us(300);
        gpio_put(STP_PIN, 1);
        sleep_us(300);
        count++;
        if(count > 10) {
            led_state = !led_state;
            count = 0;
        }
        gpio_put(LED_PIN, led_state);
    }
}