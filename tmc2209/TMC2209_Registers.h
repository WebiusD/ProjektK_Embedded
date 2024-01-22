#ifndef TMC2209_Registers_H
#define TMC2209_Registers_H

#include <stdint.h>

// Adresses of TMC2209 registers:
namespace TmcReg {
    // general configuration registers:
    const uint8_t GCONF = 0x00; // general configuration 
    const uint8_t GSTAT = 0x01; // global status 
    const uint8_t IFNCT = 0x02; // interface transmission counter
    const uint8_t NODECONF = 0x03; // send delay when reading
    const uint8_t OTP_PROG = 0x04; // OPT programming, set start-up defaults
    const uint8_t OPT_READ = 0x05; // OPT memory, read start-up defaults
    const uint8_t IOIN = 0x06;  // gpio input
    // const uint8_t FACTORY_CONF = 0x07; // factory clock settings (do not use)

    // velocity dependend control registers:
    const uint8_t IHOLD_IRUN = 0x10; // hold and run current
    const uint8_t TPOWERDOWN = 0x11; // power down delay after standstill (must be > 2)
    const uint8_t TSTEP = 0x12; // measured time between two microsteps
    const uint8_t TPWMTHRS = 0x13; // stealth chop pwm threshold (above the driver switches to spread cycle)  
    const uint8_t VACTUAL = 0x22; // actual velocity (or zero to enable STEP input via pin).

    // stall guard control registers:
    const uint8_t TCOOLTHRS = 0x14; // minimum velocity for cool step and stall guard
    const uint8_t SGTHRS = 0x40; // stall guard detection threshold
    const uint8_t SG_RESULT = 0x41; // stall guard value (is compared to SGTHRS)
    const uint8_t COOLCONF = 0x42; // cool step configuration

    // sequencer registers (microstepping):
    const uint8_t MSCNT = 0x6A; // microstep counter value
    const uint8_t MSCURACT = 0x6B; // actual microstep currents

    // chopper control registers:
    const uint8_t CHOPCONF = 0x6C; // chopper and driver configuration
    const uint8_t DRV_STATUS = 0x6F; // driver status
    const uint8_t PWMCONF = 0x70; // stealth chop pwm configuration
    const uint8_t PWM_SCALE = 0x71; // pwm scaling
    const uint8_t PWM_AUTO = 0x72; // suggested pwm values for power up 
};

// Enum classes:
enum class Direction: uint8_t {
    CW = 0,
    CCW = 1,
};

enum class MicrostepResolution: uint8_t {
    MRES_256 = 0b0000,
    MRES_128 = 0b0001,
    MRES_064 = 0b0010,
    MRES_032 = 0b0011,
    MRES_016 = 0b0100,
    MRES_008 = 0b0101,
    MRES_004 = 0b0110,
    MRES_002 = 0b0111,
    MRES_001 = 0b1000,
};

enum class StandstillMode: uint8_t {
    Normal = 0,
    Freewheeling = 1,
    StrongBrake = 2,
    Brake = 3,
};

enum class IndexFn: uint8_t {
    ShowMicroStepPos = 0,
    ShowOvertempWarning = 1,
    ShowStepPulses = 2,
};

// Data structures (most represent the value of a certain register)
union GlobalConfig
{
    struct
    {
      uint32_t i_scale_analog : 1;
      uint32_t internal_rsense : 1;
      uint32_t enable_spread_cycle : 1;
      uint32_t shaft : 1;
      uint32_t index_otpw : 1;
      uint32_t index_step : 1;
      uint32_t pdn_disable : 1;
      uint32_t mstep_reg_select : 1;
      uint32_t multistep_filt : 1;
      uint32_t test_mode : 1;
      uint32_t reserved : 22;
    };
    uint32_t bytes;
}; 

struct Settings
{
    bool is_responding;
    bool software_enabled;
    bool use_vref_current_reference;
    bool use_internal_rsense;
    bool direction_logic_inverted;

    bool spreadcycle_enabled;
    bool stealthchop_enabled;
    bool coolstep_enabled;
    bool automatic_current_scaling_enabled;
    bool automatic_gradient_enabled;

    uint8_t irun_percent;
    uint8_t irun_register_value;
    uint8_t ihold_percent;
    uint8_t ihold_register_value;
    uint8_t iholddelay_percent;
    uint8_t iholddelay_register_value;

    MicrostepResolution microsteps_per_step;
    StandstillMode standstill_mode;
}; 

struct Status
{
    uint32_t over_temperature_warning : 1;
    uint32_t over_temperature_shutdown : 1;
    uint32_t short_to_ground_a : 1;
    uint32_t short_to_ground_b : 1;
    uint32_t low_side_short_a : 1;
    uint32_t low_side_short_b : 1;
    uint32_t open_load_a : 1;
    uint32_t open_load_b : 1;
    uint32_t over_temperature_120c : 1;
    uint32_t over_temperature_143c : 1;
    uint32_t over_temperature_150c : 1;
    uint32_t over_temperature_157c : 1;
    uint32_t reserved0 : 4;
    uint32_t current_scaling : 5;
    uint32_t reserved1 : 9;
    uint32_t stealth_chop_mode : 1;
    uint32_t standstill : 1;
};

union ReplyDelay
{
  struct
  {
    uint32_t reserved_0 : 8;
    uint32_t replydelay : 4;
    uint32_t reserved_1 : 20;
  };
  uint32_t bytes;
};

union Input
{
  struct
  {
    uint32_t enn : 1;
    uint32_t reserved_0 : 1;
    uint32_t ms1 : 1;
    uint32_t ms2 : 1;
    uint32_t diag : 1;
    uint32_t reserved_1 : 1;
    uint32_t pdn_serial : 1;
    uint32_t step : 1;
    uint32_t spread_en : 1;
    uint32_t dir : 1;
    uint32_t reserved_2 : 14;
    uint32_t version : 8;
  };
  uint32_t bytes;
};

union DriverConfig
{
  struct
  {
    uint32_t ihold : 5;
    uint32_t reserved_0 : 3;
    uint32_t irun : 5;
    uint32_t reserved_1 : 3;
    uint32_t iholddelay : 4;
    uint32_t reserved_2 : 12;
  };
  uint32_t bytes;
};

union ChopperConfig
{
  struct
  {
    uint32_t toff : 4;
    uint32_t hstart : 3;
    uint32_t hend : 4;
    uint32_t reserved_0 : 4;
    uint32_t tbl : 2;
    uint32_t vsense : 1;
    uint32_t reserved_1 : 6;
    uint32_t mres : 4;
    uint32_t interpolation : 1;
    uint32_t double_edge : 1;
    uint32_t diss2g : 1;
    uint32_t diss2vs : 1;
  };
  uint32_t bytes;
};

union CoolConfig
{
  struct
  {
    uint32_t semin : 4;
    uint32_t reserved_0 : 1;
    uint32_t seup : 2;
    uint32_t reserved_1 : 1;
    uint32_t semax : 4;
    uint32_t reserved_2 : 1;
    uint32_t sedn : 2;
    uint32_t seimin : 1;
    uint32_t reserved_3 : 16;
  };
  uint32_t bytes;
};

union PwmConfig
{
  struct
  {
    uint32_t pwm_offset : 8;
    uint32_t pwm_grad : 8;
    uint32_t pwm_freq : 2;
    uint32_t pwm_autoscale : 1;
    uint32_t pwm_autograd : 1;
    uint32_t freewheel : 2;
    uint32_t reserved : 2;
    uint32_t pwm_reg : 4;
    uint32_t pwm_lim : 4;
  };
  uint32_t bytes;
};

union PwmScale
{
  struct
  {
    uint32_t pwm_scale_sum : 8;
    uint32_t reserved_0 : 8;
    uint32_t pwm_scale_auto : 9;
    uint32_t reserved_1 : 7;
  };
  uint32_t bytes;
};

union PwmAuto
{
  struct
  {
    uint32_t pwm_offset_auto : 8;
    uint32_t reserved_0 : 8;
    uint32_t pwm_gradient_auto : 8;
    uint32_t reserved_1 : 8;
  };
  uint32_t bytes;
};
#endif