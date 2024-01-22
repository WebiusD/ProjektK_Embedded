#include "hardware/uart.h"
#include "TMC2209.h"

uint8_t Datagram::_calculateCrc() {
    uint8_t crc = 0;
    uint8_t byte;

    for (uint8_t i=0; i<(size-1); ++i) {
        byte = buf[i];

        for (uint8_t j=0; j<8; ++j) {
            if ((crc >> 7) ^ (byte & 0x01)) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc = crc << 1;
            }
            byte = byte >> 1;
        }
    }
    return crc;
};

WriteDatagram::WriteDatagram(uint8_t reg_addr, int32_t data, uint8_t node_addr) {
    size = 8;
    // Attention: the datagrams depicted in the datasheet p.18-19 show all bytes
    // with their LSBit at the left and MSBit at the right.
    // So the bits are ordered with increasing significance from left to right.

    // byte3 contains a 7 bit register address and a single read/write bit:
    // for a write datagram the bit is always 1:
    uint8_t reg_rw = (reg_addr | 0x80);

    // the data bytes are ordered, high byte first (big endian):
    uint8_t data3 = (data >> 24) & 0xFF;
    uint8_t data2 = (data >> 16) & 0xFF;
    uint8_t data1 = (data >> 8) & 0xFF;
    uint8_t data0 = data & 0xFF;

    // write the bytes into the buffer, like shown on p. 18
    buf[0] = SYNC;
    buf[1] = node_addr;
    buf[2] = reg_rw;
    buf[3] = data3;
    buf[4] = data2;
    buf[5] = data1;
    buf[6] = data0;

    // must only be called after the first 7 bytes have been loaded to the buffer:
    uint8_t crc = _calculateCrc();
    buf[7] = crc;
};

ReadDatagram::ReadDatagram(uint8_t reg_addr, uint8_t node_addr) {
    size = 4;
    // Attention: the datagrams depicted in the datasheet p.18-19 show all bytes
    // with their LSBit at the left and MSBit at the right.
    // So the bits are ordered with increasing significance from left to right.

    // byte3 contains a 7 bit register address and a single read/write bit:
    // for a write datagram the bit is always 0:
    uint8_t reg_rw = reg_addr & 0x7F;

    // write the bytes into the buffer, like shown on p. 18
    buf[0] = SYNC;
    buf[1] = node_addr;
    buf[2] = reg_rw;

    // must only be called after the first 7 bytes have been loaded to the buffer:
    uint8_t crc = _calculateCrc();
    buf[3] = crc;
};

// construct a reply datagram by reading in the message into the buffer:
ReplyDatagram::ReplyDatagram(uint8_t* msg) {
    size = 8;

    // master address should be 0xFF:
    if(msg[1] == _master_addr) {
        is_reply = true;
    }

    reg_addr = msg[2];

    /*printf("Data byte 1: %d\n", msg[3]); // 33
    printf("Data byte 2: %d\n", msg[4]); // 0
    printf("Data byte 3: %d\n", msg[5]); // 0
    printf("Data byte 4: %d\n", msg[6]); // 64 */
    // construct the uint32_t data out of the four bytes:
    data = 0;
    for(uint8_t i=0; i<4; i++) {
        // the lowest byte is read first:
        data |= msg[3 + i] << (i * 8);
    }
};

// Initialize the static member NODE_COUNTER:
uint8_t TMC2209::NODE_COUNTER = 0;

TMC2209::TMC2209(uart_inst_t* uart_ch, long baudrate, uint8_t gp_enable) 
    : _uart_ch(uart_ch), _gp_enable(gp_enable), _direction(Direction::CW) {

    // set this instances address to that of the current NODE_COUNTER:
    _node_address = NODE_COUNTER++;

    uart_init(_uart_ch, baudrate);
    uart_set_format(_uart_ch, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(_uart_ch, true);

    const uint8_t uart1_tx_gpio = 8;
    const uint8_t uart1_rx_gpio = 9;
    gpio_set_function(uart1_tx_gpio, GPIO_FUNC_UART);
    gpio_set_function(uart1_rx_gpio, GPIO_FUNC_UART);

    if(uart_is_enabled(_uart_ch)) {
        printf("Uart1 is enabled\n");
    } else {
        printf("Could not enable uart1\n");
    }

    // setup the enable pin as an output:
    gpio_init(_gp_enable);
    gpio_set_dir(_gp_enable, GPIO_OUT);

    // set the communication mode to serial by default:
    _set_mode_to_serial();
}

void TMC2209::enable() {
    if(_gp_enable >= 0)
        gpio_put(_gp_enable, 0);

    // use internally stored value for toff:
    _chopper_config.toff = _toff;
    write(TmcReg::CHOPCONF, _chopper_config.bytes);
}

void TMC2209::disable() {
    if(_gp_enable >= 0)
        gpio_put(_gp_enable, 1);

    // toff = 0 will disable the driver stage:
    _chopper_config.toff = 0;
    write(TmcReg::CHOPCONF, _chopper_config.bytes);
}

void TMC2209::_set_mode_to_serial() {
    _global_config.bytes = 0;
    _global_config.i_scale_analog = 0;
    _global_config.pdn_disable = 1;
    _global_config.mstep_reg_select = 1;
    _global_config.multistep_filt = 1;

    // set the global configuration register:
    WriteDatagram wDatagram(TmcReg::GCONF, _global_config.bytes);
    _send_unidirectional(wDatagram);
};

uint8_t TMC2209::get_version() {
  Input input;
  input.bytes = read(TmcReg::IOIN);

  return input.version;
}

bool TMC2209::is_responding() {
    return get_version() == 0x21;
}

uint8_t TMC2209::_clamp(uint8_t value, uint8_t min, uint8_t max) {
    if(value < min) {
        return min;
    }
    if(value > max) {
        return max;
    }
    return value;
}

void TMC2209::_write_driver_config() {
    write(TmcReg::IHOLD_IRUN, _driver_config.bytes);

    // set minimum current for smart current control (when cool step is enabled):
    if(_driver_config.irun >= 20) {
        _cool_config.seimin = 1;
    } else {
        _cool_config.seimin = 0;
    }

    if(_coolstep_enabled) {
        write(TmcReg::COOLCONF, _cool_config.bytes);
    }
}

void TMC2209::set_irun_percent(uint8_t percent) {
    uint8_t percent_clamped = _clamp(percent, 0, 100);

    // irun scaling factor is in the range of [0, 31]
    uint8_t irun = 31 * percent_clamped / 100;
    printf("Percent: %d. Calculated irun: %d\n", percent, irun);
    _driver_config.irun = irun;

    _write_driver_config();
}

void TMC2209::set_ihold_percent(uint8_t percent) {
    uint8_t percent_clamped = _clamp(percent, 0, 100);

    // ihold scaling factor is in the range of [0, 31]
    uint8_t ihold = 31 * percent_clamped / 100;
    _driver_config.ihold = ihold;

    _write_driver_config();
}

void TMC2209::set_hold_delay_percent(uint8_t percent) {
    uint8_t percent_clamped = _clamp(percent, 0, 100);

    // hold-delay factor is in the range of [0, 15]
    uint8_t iholddelay = 15 * percent_clamped / 100;
    _driver_config.iholddelay = iholddelay;

    _write_driver_config();
}

void TMC2209::set_standstill_mode(StandstillMode mode) {
    _pwm_config.freewheel = static_cast<uint8_t>(mode);
    write(TmcReg::PWMCONF, _pwm_config.bytes);
}

void TMC2209::set_powerdown_delay(float sec) {        
    // setting in in range [0, 256] which correspondes to 0-5.6 seconds
    uint8_t pwd_delay;

    if(sec >= 5.6f) {
        pwd_delay = 255;
    } else {
        pwd_delay = sec / 5.6f * 255;
    }

    // A minimum setting of 2 is required to allow automatic tuning of PWM_OFFS_AUTO:
    pwd_delay = _clamp(pwd_delay, 2, 255);
    write(TmcReg::TPOWERDOWN, pwd_delay);
}

void TMC2209::invert_direction_logic() {
    _shaft = !_shaft;

    _global_config.shaft = _shaft ? 1 : 0;
    write(TmcReg::GCONF, _global_config.bytes);
}

void TMC2209::index_fn_select(IndexFn index_fn) {
    uint8_t fn = static_cast<uint8_t>(index_fn);

    if(fn < 2) {
        _global_config.index_step = 0;
        _global_config.index_otpw = fn;
    } else {
        // ShowStepPulses: fn == 3
        _global_config.index_step = 1;
    }
    write(TmcReg::GCONF, _global_config.bytes);
}

void TMC2209::set_reply_delay(uint8_t bit_times) {
    // a minimum of two bit times should be used in a multi node configuration (i.e. more than one tmc device).
    if(TMC2209::NODE_COUNTER > 1) {
        bit_times = _clamp(bit_times, 2, bit_times);
    }

    // the maximum value is 15:
    bit_times = _clamp(bit_times, 0, 15);

    ReplyDelay reply_delay;
    reply_delay.bytes = 0;
    reply_delay.replydelay = bit_times;
    write(TmcReg::NODECONF, reply_delay.bytes);
}

void TMC2209::enable_stealth_chop() {
    _global_config.enable_spread_cycle = 0;
    write(TmcReg::GCONF, _global_config.bytes);
}

void TMC2209::disable_stealth_chop() {
    _global_config.enable_spread_cycle = 1;
    write(TmcReg::GCONF, _global_config.bytes);
}

void TMC2209::use_external_rsense() {
    _global_config.internal_rsense = 0;
    write(TmcReg::GCONF, _global_config.bytes);
}

void TMC2209::use_internal_rsense() {
    _global_config.internal_rsense = 1;
    write(TmcReg::GCONF, _global_config.bytes);
}

void TMC2209::set_auto_current_scaling(bool on) {
    uint8_t state = on ? 1 : 0;
    _pwm_config.pwm_autoscale = state;
    write(TmcReg::PWMCONF, _pwm_config.bytes);
}

void TMC2209::set_auto_gradient(bool on) {
    uint8_t state = on ? 1 : 0;
    _pwm_config.pwm_autograd = state;
    write(TmcReg::PWMCONF, _pwm_config.bytes);
}

void TMC2209::set_coolstep_and_stalldiag_lower_threshold(uint32_t thrs) {
    write(TmcReg::TCOOLTHRS, thrs);
}

void TMC2209::set_stealthchop_upper_threshold(uint32_t thrs) {
    write(TmcReg::TPWMTHRS, thrs);
}

void TMC2209::set_stallguard_threshold(uint8_t load_thrs_percent) {
    uint8_t stall_value = 255 * (1.0f - _clamp(load_thrs_percent, 0, 100) / 100.0f);

    stall_value = _clamp(stall_value, 1, 255);
    write(TmcReg::SGTHRS, stall_value);
}

void TMC2209::enable_cool_step_in_load_interval(uint8_t lower_thrs_percent, uint8_t upper_thrs_percent) {
    // TODO impelement.
    // cool step should be used only with stealth chop (see datasheet p. 29)
};

Settings TMC2209::get_settings() {
    Settings settings;

    settings.is_responding = is_responding();
    if(settings.is_responding) {
        settings.software_enabled = _chopper_config.toff > 0;
        settings.use_vref_current_reference = _global_config.i_scale_analog;
        settings.use_internal_rsense = _global_config.internal_rsense;
        settings.direction_logic_inverted = _shaft;

        settings.spreadcycle_enabled = _global_config.enable_spread_cycle;
        settings.stealthchop_enabled = !_global_config.enable_spread_cycle;
        settings.coolstep_enabled = _coolstep_enabled;
        settings.automatic_current_scaling_enabled = _pwm_config.pwm_autoscale;
        settings.automatic_gradient_enabled = _pwm_config.pwm_autograd;

        settings.irun_percent = _byte_to_percent(_driver_config.irun);
        settings.irun_register_value = _driver_config.irun;
        settings.ihold_percent = _byte_to_percent(_driver_config.ihold);
        settings.ihold_register_value = _driver_config.ihold;
        settings.iholddelay_percent = _byte_to_percent(_driver_config.iholddelay);
        settings.iholddelay_register_value = _driver_config.iholddelay;

        MicrostepResolution ms_res;
        switch(_chopper_config.mres) {
            case 0:
                ms_res = MicrostepResolution::MRES_001;
                break;
            case 1:
                ms_res = MicrostepResolution::MRES_002;
                break;
            case 2:
                ms_res = MicrostepResolution::MRES_004;
                break;
            case 3:
                ms_res = MicrostepResolution::MRES_008;
                break;
            case 4:
                ms_res = MicrostepResolution::MRES_016;
                break;
            case 5:
                ms_res = MicrostepResolution::MRES_032;
                break;
            case 6:
                ms_res = MicrostepResolution::MRES_064;
                break;
            case 7:
                ms_res = MicrostepResolution::MRES_128;
                break;
            case 8:
                ms_res = MicrostepResolution::MRES_256;
                break;
            default:
                ms_res = MicrostepResolution::MRES_001;
        };
        settings.microsteps_per_step = ms_res;

        StandstillMode ss_mode;
        switch(_pwm_config.freewheel) {
            case 0:
                ss_mode = StandstillMode::Normal;
                break;
            case 1:
                ss_mode = StandstillMode::Freewheeling;
                break;
            case 2:
                ss_mode == StandstillMode::StrongBrake;
                break;
            case 3:
                ss_mode = StandstillMode::Brake;
                break;
            default:
                ss_mode = StandstillMode::Normal;
        }
        settings.standstill_mode = ss_mode;
    }
    return settings;
}

// TODO: add option to check for the IFCNT:
void TMC2209::_send_unidirectional(Datagram &datagram) {
    uart_write_blocking(_uart_ch, datagram.buf, datagram.size);
}

void TMC2209::_send_bidirectional(Datagram& datagram) {
    uint8_t byte;

    // wait till all outgoing transmissions have finished:
    uart_tx_wait_blocking(_uart_ch);

    // clear the receive buffer:
    while(uart_is_readable(_uart_ch)) {
        byte = uart_getc(_uart_ch);
    }

    // printf("Sending %d bytes to device.\n", datagram.size);

    // send the read request datagram:
    uart_write_blocking(_uart_ch, datagram.buf, datagram.size);

    // wait till all bytes have been transmitted:
    uart_tx_wait_blocking(_uart_ch);

    // the bytes send on tx will be echoed on rx, so read them back to clear the buffer
    // if the buffer is not readable within the max echo delay, the transmission likely failed:
    for(uint8_t i=0; i<datagram.size; i++) {
        if(uart_is_readable_within_us(_uart_ch, ECHO_DELAY_MAX_MICROSECONDS)){
            byte = uart_getc(_uart_ch);
            printf("Read echoed byte number %d, value: %d\n", i, byte);
        } else {
            printf("Read of echo timed out. Max echo delay exceeded!");
            return;            
        }
    }
}

void TMC2209::write(uint8_t reg_addr, uint32_t data) {
    WriteDatagram wDatagram(reg_addr, data);

    _send_unidirectional(wDatagram);
}

uint32_t TMC2209::read(uint8_t reg_addr) {
    ReadDatagram rDatagram(reg_addr);
    _send_bidirectional(rDatagram);

    // wait till the reply has arrived, except the max reply times out:
    uint8_t bytes[8];

    for(uint8_t i=0; i<8; i++) {
        if(uart_is_readable_within_us(_uart_ch, REPLY_DELAY_MAX_MICROSECONDS)) {
            bytes[i] = uart_getc(_uart_ch);
        } else {
            printf("Read of reply timed out. Max reply delay exceeded\n");
            return 0;
        }
    }

    ReplyDatagram datagram(bytes);
    if(!datagram.is_reply) {
        printf("The received datagram, was not addressed to the master. Data is: %d\n", datagram.data);
        return 0;
    }
    return datagram.data;
}


/* void TMC2209::setRegistersToDefaults() {
    _driver_current.bytes = 0;
    _driver_current.ihold = IHOLD_DEFAULT;
    _driver_current.irun = IRUN_DEFAULT;
    _driver_current.iholddelay = IHOLDDELAY_DEFAULT;
    _write(ADDRESS_IHOLD_IRUN, _driver_current.bytes);

    _chopper_config.bytes = CHOPPER_CONFIG_DEFAULT;
    _chopper_config.tbl = TBL_DEFAULT;
    _chopper_config.hend = HEND_DEFAULT;
    _chopper_config.hstart = HSTART_DEFAULT;
    _chopper_config.toff = TOFF_DEFAULT;
    _write(ADDRESS_CHOPCONF, _chopper_config.bytes);

    _pwm_config.bytes = PWM_CONFIG_DEFAULT;
    _write(ADDRESS_PWMCONF, _pwm_config.bytes);

    _cool_config.bytes = COOLCONF_DEFAULT;
    _write(ADDRESS_COOLCONF, _cool_config.bytes);

    _write(ADDRESS_TPOWERDOWN, TPOWERDOWN_DEFAULT);
    _write(ADDRESS_TPWMTHRS, TPWMTHRS_DEFAULT);
    _write(ADDRESS_VACTUAL, VACTUAL_DEFAULT);
    _write(ADDRESS_TCOOLTHRS, TCOOLTHRS_DEFAULT);
    _write(ADDRESS_SGTHRS, SGTHRS_DEFAULT);
    _write(ADDRESS_COOLCONF, COOLCONF_DEFAULT);
}
*/

void TMC2209::set_step_dir_pins(uint8_t gp_step, uint8_t gp_dir) {
    gpio_init(gp_step);
    gpio_set_dir(gp_step, GPIO_OUT);
    gpio_init(gp_dir);
    gpio_set_dir(gp_dir, GPIO_OUT);

    _gp_step = gp_step;
    _gp_dir = gp_dir;
}

void TMC2209::move_at_velocity(int32_t microsteps_per_period) {
    WriteDatagram wDatagram(TmcReg::VACTUAL, microsteps_per_period);
    _send_unidirectional(wDatagram);
}

void TMC2209::move_steps(uint32_t steps, Direction dir) {
    if(_gp_step < 0 || _gp_dir < 0) {
        printf("Setup step and direction pins first!\n");
        return;
    }
    _enable_step_dir_interface();

    // set movement direction:
    gpio_put(_gp_dir, static_cast<uint8_t>(dir));

    uint8_t state = false;
    for(uint32_t i=0; i<steps; i++) {
        gpio_put(_gp_step, state);
        state = !state;
        sleep_us(1000);
    }
}

void TMC2209::_enable_step_dir_interface() {
    // is enabled by setting VACTUAL to zero (datasheet p. 28)
    WriteDatagram wDatagram(TmcReg::VACTUAL, 0);
    _send_unidirectional(wDatagram);
}

void TMC2209::set_microstep_res(MicrostepResolution ms_res) {
    // cast ms_res enum type to its underlying field:
    _chopper_config.mres = static_cast<uint8_t>(ms_res);
    write(TmcReg::CHOPCONF, _chopper_config.bytes);
}

uint8_t TMC2209::_byte_to_percent(uint8_t byte) {
    return 100 * byte / 255.0f;
}
