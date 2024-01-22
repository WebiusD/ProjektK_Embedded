#include "mcp4725.h"

const uint8_t led_pin = 24;
uint16_t value = 0;

int main() {
    // i2c_inst_t* i2c = i2c0;
    MCP4725 mcp(i2c0, I2cFreq::FastMode);

    gpio_init(led_pin);
    gpio_set_dir(led_pin, GPIO_OUT);

    stdio_init_all();

    // write 401 to dac and to eeprom:
    mcp.setVoltage(401, true);
    sleep_ms(2000);

    // write 4095 to dac only:
    mcp.setVoltage(4095, false);
    sleep_ms(2000);

    // trigger reset event this should load the eeprom value, retaining the 401:
    mcp.triggerResetEvent();

    while(true) {
        for(int i=0; i<101; i++) {
            mcp.setVoltagePercent(i);
            sleep_ms(1);
        }
    }
    /* while(true) {
        mcp.setVoltage(3100);
        value = mcp.getVoltage(false);
        printf("Read value: %d", value);
        gpio_put(led_pin, true);
        sleep_ms(1000);

        mcp.setVoltage(278);
        value = mcp.getVoltage(false);
        printf("Read value: %d", value);
        gpio_put(led_pin, false);
        sleep_ms(1000);
    } */

    /* uint8_t read_id = mcp.readDeviceId();
    if(read_id != DEVICD_ID) {
        printf("ERROR: Could not read the expected address");
    } else {
        printf("Success");
    } */
}