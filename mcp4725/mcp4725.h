#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "time.h"

// device address:
// 1100 000 = 6 * 16 = 0x60

static const uint8_t MCP4725_ADDR = 0x60;

static const uint8_t DAC_ONLY = 0x40;
static const uint8_t DAC_AND_EEPROM = 0x60;
static const uint8_t GC_RESET = 0x06;

static const uint16_t MIN_VALUE = 0;
static const uint16_t MAX_VALUE = 4095;

enum class I2cFreq: uint {
    StandardMode = 100,
    FastMode = 400
};

enum class PowerDownMode: uint {
    normal = 0,
    pulldown1k = 1,
    pulldown100k = 2,
    pulldown500k = 4,
};

class MCP4725 {
private:
    i2c_inst_t* _i2c_ch;
    PowerDownMode _powerdownmode;
    uint16_t _lastValue;

    uint8_t _readRegister(uint8_t* buf, uint8_t nbytes);
    uint8_t _generalCall(uint8_t gc);

/*     int __reg_write(const uint8_t reg_addr, uint8_t* buf, const uint8_t nbytes);
    int __reg_read(const uint8_t reg_addr, uint8_t* buf, const uint8_t nbytes); */

public:
    MCP4725(i2c_inst_t* i2c_ch, I2cFreq mode=I2cFreq::StandardMode);

    bool isReady();

    void setVoltage(uint16_t value, bool store_to_eeprom=false);
    void setVoltagePercent(uint8_t percent, bool store_to_eeprom=false);

    uint16_t getVoltage(bool use_cache=true);
    uint8_t getVoltagePercent(bool use_cache=true);

    uint16_t getEEPROM();

    void setPowerDownMode(PowerDownMode mode, bool store_to_eeprom=true);

    // trigger a reset (automatically triggered on powered up)
    // this will load the data from the EEPROM into the DAC register.
    void triggerResetEvent();
};