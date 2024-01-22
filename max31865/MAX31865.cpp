#include "MAX31865.h"
#include <cmath>
#include <stdio.h>

MAX31865::MAX31865(spi_inst_t* spi_ch, uint8_t spi_cs, uint8_t spi_sck, uint8_t spi_mosi, 
    uint8_t spi_miso, RtdWires wires, float rtd_nominal, float r_ref) 
    : _spi_ch(spi_ch), _gp_cs(spi_cs), _rtd_nominal(rtd_nominal), _r_ref(r_ref) {
    
    // init with 100 kHz frequency:
    spi_init(_spi_ch, 100 * 1000);
    // use spi mode 1 (i.e. CPOL=0, CPHA=1) and MSB first:
    spi_set_format(_spi_ch, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);
    gpio_set_function(spi_sck, GPIO_FUNC_SPI);
    gpio_set_function(spi_mosi, GPIO_FUNC_SPI);
    gpio_set_function(spi_miso, GPIO_FUNC_SPI);

    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(spi_cs);
    gpio_set_dir(spi_cs, GPIO_OUT);
    gpio_put(spi_cs, 1);

    set_wires(wires);
    // enable_bias(false);
    auto_convert(true);
    set_thresholds(0, 0xFFFF);
    clear_fault();
}

void MAX31865::enable_bias(bool b) {
    // the bias can only be controlled in one shot mode
    // in auto mode, bias is always on!
    uint8_t cfg = read_register8(MaxReg::CONFIG_REG);
    if(b) {
        cfg |= MaxReg::CONFIG_BIAS; // enable bias
    } else {
        cfg &= ~MaxReg::CONFIG_BIAS; // disable bias
    }
    write_register8(MaxReg::CONFIG_REG, cfg);
}

void MAX31865::set_thresholds(uint16_t lower, uint16_t upper) {
    write_register8(MaxReg::LFAULTLSB_REG, lower & 0xFF);
    write_register8(MaxReg::LFAULTMSB_REG, lower >> 8);
    write_register8(MaxReg::HFAULTLSB_REG, upper & 0xFF);
    write_register8(MaxReg::HFAULTMSB_REG, upper >> 8);
}

uint16_t MAX31865::get_lower_threshold() {
    return read_register16(MaxReg::LFAULTMSB_REG);
}

uint16_t MAX31865::get_upper_threshold() {
    return read_register16(MaxReg::HFAULTMSB_REG);
}

void MAX31865::set_wires(RtdWires wires) {
    // store setting in private field:
    _wires = wires;

    uint8_t cfg = read_register8(MaxReg::CONFIG_REG);
    if(wires == RtdWires::WIRE3) {
        // 3 wire rtd setup:
        cfg |= MaxReg::CONFIG_3WIRE;
    } else {
        // 2 or 4 wire rtd setup:
        cfg &= ~MaxReg::CONFIG_3WIRE;
    }
    write_register8(MaxReg::CONFIG_REG, cfg);
}

void MAX31865::set_ready_pin(uint8_t gp_ready) {
    _gp_ready = gp_ready;
}

void MAX31865::enable_filter(FilterFreq freq) {
    uint8_t cfg = read_register8(MaxReg::CONFIG_REG);
    
    // filter frequency should not be changed when in auto conversion.
    // see datasheet p. 14.
    /* bool reset_mode_to_auto = false;
    if(_op_mode == OpMode::Auto) {
        // set_op_mode(OpMode::OneShot);
        reset_mode_to_auto = true;
    } */

    if(freq == FilterFreq::F50Hz) {
        cfg |= 0x01; // enalbe 50Hz filter
    } else {
        cfg &= ~0x01; // enable 60Hz filter
    }
    write_register8(MaxReg::CONFIG_REG, cfg);

    /* if(reset_mode_to_auto)
        set_op_mode(OpMode::Auto); */
}

void MAX31865::auto_convert(bool b) {
    uint8_t cfg = read_register8(MaxReg::CONFIG_REG);
    if(b) {
        cfg |= MaxReg::CONFIG_MODEAUTO;
        _op_mode = OpMode::Auto;
    } else {
        cfg &= ~MaxReg::CONFIG_MODEAUTO;
        _op_mode = OpMode::OneShot;
    }
    write_register8(MaxReg::CONFIG_REG, cfg);
}

/* void MAX31865::set_op_mode(OpMode mode) {
    uint8_t cfg = read_register8(MaxReg::CONFIG_REG);

    if(mode == OpMode::OneShot) {
        // disable auto conversion:
        cfg &= ~MaxReg::CONFIG_MODEAUTO;
        // configure one shot mode:
        cfg |= MaxReg::CONFIG_1SHOT;
        _op_mode = OpMode::OneShot;
    } else {
        //disable one shot mode:
        cfg &= ~MaxReg::CONFIG_1SHOT;
        // enable auto conversion:
        cfg |= MaxReg::CONFIG_MODEAUTO;
        _op_mode = OpMode::Auto;
    }

    write_register8(MaxReg::CONFIG_REG, cfg);
    sleep_ms(10);
} */


uint16_t MAX31865::read_rtd() {
    uint16_t rtd;

    clear_fault();

    enable_bias(true);
    sleep_ms(10);

    uint8_t cfg = read_register8(MaxReg::CONFIG_REG);
    cfg |= MaxReg::CONFIG_1SHOT;
    write_register8(MaxReg::CONFIG_REG, cfg);
    sleep_ms(65); // also worked with 30 ms

    rtd = read_register16(MaxReg::RTDMSB_REG);
    enable_bias(false);

    // remove fault bit:
    rtd >>= 1;
    return rtd;
}

float MAX31865::read_in_C() {
    uint16_t rtd_raw = read_rtd();
/*     if(rtd_value == -1) {
        // there was an error:
        return -1.0f;
    } */
    float t = _calculate_temperature(rtd_raw, _rtd_nominal, _r_ref);

    return t;
}

float MAX31865::_calculate_temperature(uint16_t rtd_raw, float rtd_nominal, float r_ref) {
    float Z1, Z2, Z3, Z4, rt, temp;

    rt = rtd_raw;
    rt = (rt / 32768) * r_ref;

    Z1 = -CVD_A;
    Z2 = CVD_A * CVD_A - (4 * CVD_B);
    Z3 = (4 * CVD_B) / rtd_nominal;
    Z4 = 2 * CVD_B;

    temp = Z2 + (Z3 * rt);
    temp = (std::sqrt(temp) + Z1) / Z4;

    if(temp >= 0)
        return temp;
    
    // temperature below zero:
    rt /= rtd_nominal;
    rt *= 100; // normalize to 100 ohm.

    float rpoly = rt;

    temp = -242.02;
    temp += 2.2228 * rpoly;
    rpoly *= rt; // ^2 term
    temp += 2.5859e-3 * rpoly;
    rpoly *= rt; // ^3 term
    temp -= 4.8260e-6 * rpoly;
    rpoly *= rt; // ^4 term
    temp -= 2.8183e-8 * rpoly;
    rpoly *= rt; // ^5 term
    temp += 1.5243e-10 * rpoly;

    return temp;
}

uint8_t MAX31865::read_register8(uint8_t reg_addr) {
    uint8_t value = 0;
    read_registers(reg_addr, &value, 1);
    return value;
}

uint16_t MAX31865::read_register16(uint8_t reg_addr) {
    uint8_t buf[2] = {0, 0};
    read_registers(reg_addr, buf, 2);

    uint16_t value = (buf[0] << 8) | buf[1];
    return value;
}

void MAX31865::read_registers(uint8_t reg_addr, uint8_t buf[], uint8_t n) {
    reg_addr &= 0x7F; // clear the top bit, indicating a read.

    _cs_select();
    spi_write_blocking(_spi_ch, &reg_addr, 1);
    spi_read_blocking(_spi_ch, 0, buf, n);
    _cs_deselect();
}

void MAX31865::write_register8(uint8_t reg_addr, uint8_t data) {
    reg_addr |= 0x80; // set the top bit, indicating a write.
    uint8_t buf[2] = {reg_addr, data};

    _cs_select();
    spi_write_blocking(_spi_ch, buf, 2);
    _cs_deselect();
}

void MAX31865::clear_fault() {
    uint8_t cfg = read_register8(MaxReg::CONFIG_REG);
    cfg &= ~0x2C; // clear bits D5, D3 and D2: 0010 1100 
    cfg |= MaxReg::CONFIG_FAULTSTAT; // set bit D1: 0000 0010;
    write_register8(MaxReg::CONFIG_REG, cfg);
}

uint8_t MAX31865::read_fault_code(FaultCycle fault_cycle) {
    // the objective of this function is to read the fault status register. In auto mode
    // this is done 100us after the read is requested. When a rc-input filter with a higher
    // time constant is present, the manual mode should be used. Whenever a fault is detected
    // the respective bit will be latched to the fault status register. To clear the register
    // clear_fault must be called.

    if(fault_cycle != FaultCycle::FAULT_NONE) {
        uint8_t cfg = read_register8(MaxReg::CONFIG_REG);
        cfg &= 0x11; // keep only wire and filter configuration bits.

        switch(fault_cycle) {
            case FaultCycle::FAULT_AUTO:
                write_register8(MaxReg::CONFIG_REG, (cfg | 0b1000'0100));
                // wait 1ms before making the reading:
                sleep_ms(1);
                break;
            case FaultCycle::FAULT_MANUAL:
                // intially FORCE- input switch is closed.
                // enable V_bias and wait 1ms plus at least 5 time constants for the capacitor to charge:
                enable_bias(true);
                sleep_us(1000 + 5*_rc_time_const_us);

                // initiate check with FORCE- closed:
                write_register8(MaxReg::CONFIG_REG, (cfg | 0b1000'1000));

                // when the check completed the FORCE- input switch is opened
                // again wait 1ms plus 5 time constants (for the capacitor to settle).
                sleep_us(1000 + 5*_rc_time_const_us);

                // initiate check with FORCE- opened (this will set fault cycle control bits D3 and D2)
                write_register8(MaxReg::CONFIG_REG, (cfg | 0b1000'1100));
                
                // when test is finished FORCE- input closes and the fault cycle control bits 
                // D3 and D2 are cleared:
                while(read_register8(MaxReg::CONFIG_REG) & 0x0C) {
                    tight_loop_contents();
                }
                break;
            case FaultCycle::FAULT_NONE:
            default:
                break;
        }
    }
    return read_register8(MaxReg::FAULTSTAT_REG);
}

std::string MAX31865::decode_fault(uint8_t fault_code, int16_t rtd_raw) {
    // A string to store faults
    std::string faultString;

    // The individual status bits of the fault_code:
    bool sb[8];
    for (uint8_t i = 0; i < 8; i++) {
        sb[i] = (fault_code & (1 << i)) != 0;
    }

    // 2-Wire setup:
    if (_wires == RtdWires::WIRE2) {
        if (sb[7] && _is_full_scale(rtd_raw)) {
            faultString += "RTD Circuit Open\n";
        }
        if (sb[6] && _is_zero(rtd_raw)) {
            faultString += "RTD Circuit Shorted\n";
            faultString += "RTDin+ Shorted Low\n";
        }
        if (sb[5]) {
            if (_is_full_scale(rtd_raw)) {
                faultString += "RTD Circuit Open\n";
            } else {
                faultString += "RTDin+ Shorted High\n";
                faultString += "RTDin- Shorted High\n";
            }
        }
        if (sb[4]) {
            faultString += "RTDin- Shorted Low\n";
        }
        if (sb[3]) {
            if (_is_zero(rtd_raw)) {
                faultString += "RTDin+ Shorted Low\n";
            } else {
                faultString += "RTDin- Shorted Low\n";
            }
        }
        if (sb[2]) {
            faultString += "Over or Under Voltage\n";
        }

        return faultString;
    }

    // 3-Wire setup:
    if (_wires == RtdWires::WIRE3) {
        if (sb[7]) {
            faultString += "RTD Circuit Open\n";
            faultString += "RTDin+ Shorted High\n";
            faultString += "Force+ Shorted High\n";
        }
        if (sb[6]) {
            faultString += "RTDin+ Shorted to RTDin-\n";
            faultString += "RTDin+ Shorted Low\n";
            faultString += "Force+ Shorted Low\n";
        }
        if (sb[5]) {
            if (_is_full_scale(rtd_raw)) {
                faultString += "RTD Circuit Open\n";
                faultString += "Force+ Shorted High\n";
            } else {
                faultString += "Force+ Unconnected\n";
                faultString += "Force+ Shorted High\n";
                faultString += "RTDin- Shorted High\n";
            }
        }
        if (sb[4]) {
            faultString += "RTDin- Shorted High\n";
        }
        if (sb[3]) {
            if (_is_zero(rtd_raw)) {
                faultString += "Force+ Shorted Low\n";
                faultString += "RTDin+ Shorted Low\n";
            } else {
                faultString += "RTDin- Shorted Low\n";
            }
        }
        if (sb[2]) {
            faultString += "Over or Under Voltage\n";
        }

        return faultString;
    }

    // 4-Wire setup:
    if (_wires == RtdWires::WIRE4) {
        if (sb[7]) {
            faultString += "RTD Circuit Open\n";
            faultString += "RTDin+ Shorted High\n";
            faultString += "Force+ Shorted High\n";
        }
        if (sb[6]) {
            faultString += "RTDin+ Shorted to RTDin-\n";
            faultString += "RTDin+ Shorted Low\n";
            faultString += "RTDin- Shorted High\n";
            faultString += "Force+ Shorted Low\n";
        }
        if (sb[5]) {
            if (_is_full_scale(rtd_raw)) {
                faultString += "RTD Circuit Open\n";
                faultString += "Force+ Shorted High\n";
            } else {
                faultString += "Force- Unconnected\n";
                faultString += "Force+ Unconnected\n";
                faultString += "Force+ Shorted High\n";
                faultString += "Force- Shorted High\n";
                faultString += "Force- Shorted Low\n";
            }
        }
        if (sb[4]) {
            faultString += "Force- Shorted Low\n";
            faultString += "RTDin- Shorted Low\n";
        }
        if (sb[3]) {
            if (_is_zero(rtd_raw)) {
                faultString += "Force+ Shorted Low\n";
                faultString += "RTDin+ Shorted Low\n";
            } else {
                faultString += "RTDin- Shorted Low\n";
                faultString += "Force- Shorted Low\n";
            }
        }
        if (sb[2]) {
            faultString += "Over or Under Voltage\n";
        }

        return faultString;
    }

    return faultString;
}

bool MAX31865::_is_full_scale(uint16_t rtd_raw) {
    // only 15 bits of 16 bits are used:
    uint16_t full_scale = (1 << 15) - 1;
    return rtd_raw == full_scale;
}

bool MAX31865::_is_zero(uint16_t rtd_raw) {
    return rtd_raw == 0;
}

// ################################################
// ################################################









/* void MAX31865::set_rc_time_const_us(uint16_t rc_time_const_us) {
    _rc_time_const_us = rc_time_const_us;
} */










