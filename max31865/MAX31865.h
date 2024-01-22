#ifndef MAX31865_H
#define MAX31865_H

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include <stdint.h>
#include <string>

// Callendar-Van Dusen coefficients according to IEC 751
// see datasheet page 10:
#define CVD_A 3.9083e-3
#define CVD_B -5.775e-7

// MAX31865 register addresses:
namespace MaxReg {
    const uint8_t CONFIG_REG = 0x00;
    const uint8_t CONFIG_BIAS = 0x80;
    const uint8_t CONFIG_MODEAUTO = 0x40;
    const uint8_t CONFIG_MODEOFF = 0x00;
    const uint8_t CONFIG_1SHOT = 0x20;
    const uint8_t CONFIG_3WIRE = 0x10;
    const uint8_t CONFIG_24WIRE = 0x00;
    const uint8_t CONFIG_FAULTSTAT = 0x02;

    const uint8_t RTDMSB_REG = 0x01;
    const uint8_t RTDLSB_REG = 0x02;
    const uint8_t HFAULTMSB_REG = 0x03;
    const uint8_t HFAULTLSB_REG = 0x04;
    const uint8_t LFAULTMSB_REG = 0x05;
    const uint8_t LFAULTLSB_REG = 0x06;
    const uint8_t FAULTSTAT_REG = 0x07;
};

// the C++ way:
enum class RtdWires: uint8_t {
    WIRE2 = 0,
    WIRE3 = 1,
    WIRE4 = 0,
};

enum class OpMode: uint8_t {
    OneShot = 0,
    Auto = 1,
};

enum class FaultCycle: uint8_t {
    // do not read the fault status register <-- needed? TODO:
    FAULT_NONE = 0,

    // trigger an automatic fault detection cycle, after 100us the fault status
    // will be returned. The fault cycle control bits D3 and D2, self clear upon completion.
    FAULT_AUTO = 1,

    // when an input filter with a time constant > 100us is used. This allows
    // to read the fault status within 10 filter time constants:
    FAULT_MANUAL = 2,
};

enum class FilterFreq: uint8_t {
    F60Hz = 0,
    F50Hz = 1,
};

class MAX31865 {
public:
    MAX31865(
        spi_inst_t* spi_ch, 
        uint8_t spi_cs, 
        uint8_t spi_sck, 
        uint8_t spi_mosi, 
        uint8_t spi_miso, 
        RtdWires wires,
        float rtd_nominal=100.0f,
        float r_ref=430.0f
    );

    // choose if a 2, 3 or 4 wire setup is used:
    void set_wires(RtdWires wires);

    // use the data ready pin of the device to see if data is available:
    void set_ready_pin(uint8_t gp_ready);

    // select either one shot or auto conversion mode. auto conversion mode is much
    // faster, but requires V_bias to remain on, resuling in more self heating of the rtd.
    // void set_op_mode(OpMode mode);

    // read the rdt resistance value from the register. 
    uint16_t read_rtd();

    // enable/disable V_bias (will only have an effect when in one shot mode)
    void enable_bias(bool b);

    void auto_convert(bool b);
    // when using one shot mode, the rc time constant of the input filter network
    // determines how long to wait after V_bias is turned on:
    /* void set_rc_time_const_us(uint16_t rc_time_const_us); */

    // set the lower and upper thresholds used for fault detection:
    void set_thresholds(uint16_t lower, uint16_t upper);

    uint16_t get_lower_threshold();
    uint16_t get_upper_threshold();

    // enable either a 50 or 60 Hz notch filter:
    void enable_filter(FilterFreq freq);

    // triggers a rtd reading and calls _calc_temperature to convert
    // the result into degree celsius
    float read_in_C();

    // read out the fault code register, this can be done automatically after 100us
    // when your rc input time constant is longer than 100us the manual mode should be used. 
    uint8_t read_fault_code(FaultCycle fault_cycle=FaultCycle::FAULT_AUTO);
    
    // returns a list of possible faults, of a certain fault code.
    std::string decode_fault(uint8_t fault_code, int16_t rtd_raw);

    // the fault code register will latch any detected faults, call clear fault
    // to reset the register and clear old recordings.
    void clear_fault();
    
private:
    // the spi channel to use:
    spi_inst_t* _spi_ch;

    // the cs pin of to use:
    uint8_t _gp_cs;

    // the wire configuration:
    RtdWires _wires;

    // the data ready pin, only used if set explicitly:
    int8_t _gp_ready = -1;

    // the nominal value of the rtd. Usually 100.0 ohm for a pt100:
    float _rtd_nominal = 100.0f;

    // the reference resistor used:
    float _r_ref;
    
    // the operation mode (one shot or auto)
    OpMode _op_mode = OpMode::Auto;

    // the time constant of the rc input network in microseconds.
    // relevant when using the one shot operation mode.
    uint16_t _rc_time_const_us = 60;

    // the tolerance used to determine if a rtd reading is full scale (i.e. all 15 bits set).
    uint16_t _full_scale_tolerance = 255;
    
    // the tolerance used to determine if a rtd reading is near zero.
    uint16_t _zero_tolerance = 255;

    // register read and write operations:
    // data is read and written MSB first:
    void read_registers(uint8_t reg_addr, uint8_t* buf, uint8_t nbytes);
    uint8_t read_register8(uint8_t reg_addr);
    uint16_t read_register16(uint8_t reg_addr);
    void write_register8(uint8_t reg_addr, uint8_t value);
    // defined here, because they shall be inlined:
    void _cs_select() {
        asm volatile("nop \n nop \n nop");
        gpio_put(_gp_cs, 0);  // Active low
        asm volatile("nop \n nop \n nop");
    };
    void _cs_deselect() {
        asm volatile("nop \n nop \n nop");
        gpio_put(_gp_cs, 1);
        asm volatile("nop \n nop \n nop");
    };

    // convert the rtd_raw value into a temperature value using Callendar-Van Dusen coefficients.
    // the conversion is done according to: 
    // http://www.analog.com/media/en/technical-documentation/application-notes/AN709_0.pdf
    float _calculate_temperature(uint16_t rtd_raw, float rtd_nominal, float r_ref);

    // check if a rtd value is close to a full scale reading:
    bool _is_full_scale(uint16_t rtd_raw);

    // check if a rtd value is close to zero:
    bool _is_zero(uint16_t rtd_raw);
};

#endif