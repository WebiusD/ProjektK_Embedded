#ifndef TMC2209_H
#define TMC2209_H

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include <stdio.h>

#include "TMC2209_Registers.h"

class Datagram {
public:
    // needs to be overriden by subclasses (pure virtual function would be to much boiler plate).
    uint8_t size = 8;
    uint8_t buf[8];

protected:
    // byte1 contains sync nibble and reserved bits:
    const uint8_t SYNC = 0x05;

    uint8_t _calculateCrc();
};

class WriteDatagram : public Datagram {
public:
    WriteDatagram(uint8_t reg_addr, int32_t data, uint8_t node_addr=0);
};

class ReadDatagram : public Datagram {
public:
    ReadDatagram(uint8_t reg_addr, uint8_t node_addr=0);
};

class ReplyDatagram: public Datagram {
public:
    uint8_t reg_addr;
    uint32_t data;
    bool is_reply = false;

    // construct a reply datagram by reading in the message into the buffer:
    ReplyDatagram(uint8_t* msg);

private:
    const uint8_t _master_addr = 0xFF;
};

class TMC2209 {
public:
    static uint8_t NODE_COUNTER;
    TMC2209(uart_inst_t* uart_ch, long baudrate, uint8_t gp_enable);

    void enable();
    void disable();

    void invert_rotation_dir();

    void write(uint8_t reg_addr, uint32_t data);
    uint32_t read(uint8_t reg_addr);

    uint8_t get_version();
    
    bool is_responding();

    void set_step_dir_pins(uint8_t gp_step, uint8_t gp_dir);
    void set_irun_percent(uint8_t percent);
    void set_ihold_percent(uint8_t percent);

    // controls how fast or slow the motor transitions from irun to ihold:
    void set_hold_delay_percent(uint8_t percent);

    // what time after the motor stops shall reduction of irun to ihold begin:
    void set_powerdown_delay(float sec);

    // globally invert the motor direction:
    void invert_direction_logic();

    // note a negative value will make the motor turn in the opposite direction:
    void move_at_velocity(int32_t microsteps_per_period);

    void move_steps(uint32_t steps, Direction dir);
    void set_microstep_res(MicrostepResolution ms_res);

    // only relevant when IHOLD=0
    void set_standstill_mode(StandstillMode mode);

    // how fast after getting a read request, shall the device answer:
    void set_reply_delay(uint8_t reply_delay);

    // enable stealth chop:
    void enable_stealth_chop();

    // disable stealth chop, use spread instead:
    void disable_stealth_chop();

    void use_external_rsense();

    void use_internal_rsense();

    // select the function of the index pin:
    void index_fn_select(IndexFn index_fn);

    // enable/disable automatic current scaling:
    void set_auto_current_scaling(bool on);

    // enable/disable automatic gradient adpation:
    void set_auto_gradient(bool on);

    // set the velocity threshold above which cool step is enabled and a stall
    // is signaled via the DIAG output.
    void set_coolstep_and_stalldiag_lower_threshold(uint32_t thrs);

    // set the velocity below which stealth chop can be used:
    void set_stealthchop_upper_threshold(uint32_t thrs);

    // the motor load value upon which a stall is detected:
    void set_stallguard_threshold(uint8_t load_thrs_percent);

    // enable cool step when within a certain load interval:
    void enable_cool_step_in_load_interval(uint8_t lower_thrs_percent, uint8_t upper_thrs_percent);

    // get main settings:
    Settings get_settings();

private:
    // counter for how many TMC2209 have been instantiated:
    const static uint32_t ECHO_DELAY_MAX_MICROSECONDS = 1'000;
    const static uint32_t REPLY_DELAY_MAX_MICROSECONDS = 10'000;

    // the uart node address (0 when only one TMC2209 driver is used)
    uint8_t _node_address;

    // the uart channel of the rp2040
    uart_inst_t* _uart_ch;

    // the gpio connected to the enable pin of the device:
    int8_t _gp_enable = -1;

    // the gpio connected to the step pin of the device:
    int8_t _gp_step = -1;

    // the gpio connected to the the direction pin of the device:
    int8_t _gp_dir = -1;

    // flag holding the value of shaft. When set to true the motor direction is globally inverted:
    bool _shaft = false;

    // the direction of rotation:
    Direction _direction;

    // flag showing if cool step is enabled:
    bool _coolstep_enabled = false;

    // use toff = 3 for steath chop:
    uint8_t _toff = 3;

    // #######################################
    // REGISTER VALUES:
    // #######################################
    // field for the global configuration register
    GlobalConfig _global_config;
    ChopperConfig _chopper_config;
    PwmConfig _pwm_config;
    CoolConfig _cool_config;
    DriverConfig _driver_config;

    // #######################################
    // Private methods:
    // #######################################
    uint8_t _clamp(uint8_t value, uint8_t min, uint8_t max);
    uint8_t _byte_to_percent(uint8_t byte);

    void _set_mode_to_serial();
    void _enable_step_dir_interface();
    void _write_driver_config();
    void _send_unidirectional(Datagram& datagram);
    void _send_bidirectional(Datagram& datagram);
};


#endif