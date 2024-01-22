#ifndef HX711_H
#define HX711_H

#include <stdint.h>
#include "pico/stdlib.h"

// to enable/disable interrupts:
#include "hardware/sync.h"

// select a pga channel and gain:
enum class PgaGain: uint8_t {
    // Channel/Gain - Additional clock pulses
    A128 = 1,
    B32 = 2,
    A64 = 3,
};

class HX711 {
public:
    HX711(uint8_t gp_dout, uint8_t gp_sck, PgaGain gain);

    bool is_ready(long timeout_ms=0);
    void set_pga_gain(PgaGain gain);
    void set_offset(int32_t offset);
    int32_t get_offset();
    float get_conversion();

    int32_t read_raw();
    int32_t read_raw_avg(uint8_t times);
    void tare(uint8_t times=10);
    double read(uint8_t times=1);

    void power_down();
    void power_up();

    // will automatically set the conversion factor, to match the given weight in gramms
    // make sure to call tare before placing the weight onto the scale 
    void calibrate_gramm(uint32_t weight_in_gramms);

private:
    uint8_t _gp_dout;
    uint8_t _gp_sck;

    PgaGain _pga_gain;
    int32_t _offset = 0;
    float _conversion_factor = 1.0f;

    uint8_t _shift_in(bool lsbFirst);
    void _pulse_clock(uint8_t);
};

#endif