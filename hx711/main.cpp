#include <stdio.h>
#include "HX711.h"

const uint8_t GP_DOUT = 7;
const uint8_t GP_SCK = 6;

const uint8_t WCALIB = 76;

int main() {
    // important for any kind of communication:
    stdio_init_all();
    sleep_ms(5000);

    HX711 hx711(GP_DOUT, GP_SCK, PgaGain::A128);
    hx711.tare();
    hx711.calibrate_gramm(0);
    printf("Finished calibration step 1...\n");

    double value = hx711.read(10);
    printf("Measured value: %.2f g\n", value);
    printf("Please place the second weight...\n");
    sleep_ms(8000);

    hx711.calibrate_gramm(WCALIB);
    printf("Finished calibration step 2...\n");

    printf("Conversion factor: %.2f\n", hx711.get_conversion());
    printf("Offset: %d\n", hx711.get_offset());

    while(true) {
        value = hx711.read(5);
        printf("Measured: %.2f\n", value);
        sleep_ms(1000);
    }
}