#include "pico/stdlib.h"
#include "hardware/pwm.h"

uint8_t max_duty = 100;

int main() {
    /// \tag::setup_pwm[]

    // Tell GPIO 24 its allocated to the PWM
    gpio_set_function(24, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO 24
    uint slice_num = pwm_gpio_to_slice_num(24);

    // Set period of 128 cycles (0 to 127 inclusive)
    pwm_set_wrap(slice_num, 127);
    // Set channel A output high for one cycle before dropping
    // pwm_set_chan_level(slice_num, PWM_CHAN_A, 1);
    
    // Set the PWM running
    pwm_set_enabled(slice_num, true);
    /// \end::setup_pwm[]

    uint8_t duty = 0;
    uint8_t inc = 1;

    while(true) {
        if (duty == max_duty) {
            inc = -1;
        } else if(duty == 0) {
            inc = 1;
        }
        duty += inc;
        // Set gpio 24 duty cycle
        pwm_set_gpio_level(24, duty);
        sleep_ms(20);
    }

    // Note we could also use pwm_set_gpio_level(gpio, x) which looks up the
    // correct slice and channel for a given GPIO.
}
