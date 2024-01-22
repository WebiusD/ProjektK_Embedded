#include "MAX31865.h"
#include <stdio.h>

const uint8_t GP_CS = 17;
const uint8_t GP_SCK = 18;
const uint8_t GP_MOSI = 19;
const uint8_t GP_MISO = 16;


int main() {
    stdio_init_all();

    MAX31865 max(spi0, GP_CS, GP_SCK, GP_MOSI, GP_MISO, RtdWires::WIRE2);
    max.enable_filter(FilterFreq::F50Hz);

    // with auto-convert true the phase where data is not ready is 12 ms,
    // with auto-convert false the phase is 2 ms.
    max.auto_convert(false);

    // max read freq: 1000 ms / 80 ms = 12 Hz
    while(true) {
        uint16_t rtd = max.read_rtd();
        printf("Read rtd value: %u\n", rtd);
        sleep_ms(100);
    }

    /*
    while(true) {
        uint16_t rtd = max.read_rtd();
        // use u for unsigned integers in printf:
        printf("Read rtd value: %u\n", rtd);

        float t = max.read_in_C();
        printf("Read temperture one-shot: %.2f\n", t);

        uint8_t fault_code = max.read_fault_code(FaultCycle::FAULT_AUTO);
        printf("Fault code: %d\n", fault_code);

        std::string fault_str = max.decode_fault(fault_code, max.read_rtd());
        printf("Possible faults:\n%s\n", fault_str.c_str());

        // max.clear_fault();
        sleep_ms(1000);
    
    }
    */
}