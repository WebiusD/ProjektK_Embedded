#include "mcp4725.h"

MCP4725::MCP4725(i2c_inst_t* i2c_ch, I2cFreq freq) {
    // Constructor initializes the I2C channel and pins
    if (!i2c_ch) {
        fprintf(stderr, "Invalid I2C channel.\n");
        // TODO: signal an error: error_led(true);
    }

    _i2c_ch = i2c_ch;
    i2c_init(_i2c_ch, static_cast<uint>(freq) * 1000);  // 100 kHz transmission clock
    // TODO: Lookup which pins correspond to the channel in use:
    uint8_t sda_pin = 4;
    uint8_t scl_pin = 5;
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);

    // set the internal fields:
    _powerdownmode = PowerDownMode::normal;
    // get the Voltage, but do not use the cache:
    _lastValue = getVoltage(false);
}

bool MCP4725::isReady() {
    uint8_t buf[1];
    // timeout of 3 ms:
    // absolute_time_t timeout = delayed_by_us(get_absolute_time(), 3'000);
    int bytes_read = i2c_read_blocking(_i2c_ch, MCP4725_ADDR, buf, 1, true);
    if(bytes_read == 1) {
        return (buf[0] & 0x80) > 0;
    }
    return false;
}

uint8_t MCP4725::_readRegister(uint8_t* buf, uint8_t nbytes) {
    // wait until device is ready:
    while(!isReady());

    uint8_t bytes_read = i2c_read_blocking(_i2c_ch, MCP4725_ADDR, buf, nbytes, false);
    return bytes_read;
}

uint8_t MCP4725::_generalCall(uint8_t gc) {
    // generell calls are issued with a device address of zero, i.e. 0x00:
    uint8_t buf[1] = {gc};

    return i2c_write_blocking(_i2c_ch, 0x00, buf, 1, true);
}

void MCP4725::setVoltage(uint16_t value, bool store_to_eeprom) {
    if(value < MIN_VALUE || value > MAX_VALUE) {
        fprintf(stderr, "Value %d exceeded the range of allowed values [0-4095]", value);
        return;
    }

    uint8_t reg_addr = store_to_eeprom ? DAC_AND_EEPROM : DAC_ONLY;
    uint8_t h = value / 16;
    uint8_t l = (value & 0x0F) << 4;
    
    // prepend the register address to the buffer:
    uint8_t data[3] = {reg_addr, h, l};
    // uint8_t data[3];
    /* data[0] = DAC_REG_ADDR;
    data[1] = h;
    data[2] = l; */

    // nostop=false means end of message but not end of transmission:
    // nostop=true means end of transmission:
    int bytes_written = i2c_write_blocking(_i2c_ch, MCP4725_ADDR, data, 3, false);
    if(bytes_written == 3)
        _lastValue = value;
}

void MCP4725::setVoltagePercent(uint8_t percent, bool store_to_eeprom) {
    if(percent < 0 || percent > 100) {
        // TODO emit an error:
        return;
    }
    uint16_t actual_value = MAX_VALUE * percent / 100;
    setVoltage(actual_value, store_to_eeprom);
}

uint16_t MCP4725::getVoltage(bool use_cache) {
    if(use_cache) {
        return _lastValue;
    }

    uint8_t buf[3];
    _readRegister(buf, 3);
    uint16_t value = buf[1];
    value = value << 4;
    value = value + (buf[2] >> 4);
    
    _lastValue = value;
    return value;
}

uint8_t MCP4725::getVoltagePercent(bool use_cache) {
    uint16_t actual_value = getVoltage(use_cache);

    uint8_t percent = actual_value / MAX_VALUE * 100;
    return percent;
}

uint16_t MCP4725::getEEPROM() {
    uint8_t buf[5];
    _readRegister(buf, 5);
    uint16_t value = buf[3] & 0x0F;
    value = value << 8;
    value = value + buf[4];
    return value;
}

void MCP4725::setPowerDownMode(PowerDownMode mode, bool store_to_eeprom) {
    uint8_t reg_addr = store_to_eeprom ? DAC_AND_EEPROM : DAC_ONLY;
    uint8_t mode_bits = static_cast<uint>(mode);
    reg_addr |= (mode_bits << 1);

    uint8_t data[3] = {reg_addr, 0x00, 0x00};

    int bytes_written = i2c_write_blocking(_i2c_ch, MCP4725_ADDR, data, 3, false);
    if(bytes_written == 3)
        _powerdownmode = mode;
}

void MCP4725::triggerResetEvent() {
    _generalCall(GC_RESET);
    // update the _lastValue chache:
    getVoltage(false);
}

/* int MCP4725::__reg_write(const uint8_t reg_addr, uint8_t* buf, const uint8_t nbytes) {
    // Private member function for writing data to a register
    if (nbytes < 1) {
        fprintf(stderr, "Invalid number of bytes to write: %d\n", nbytes);
        // Handle error, throw exception, or exit gracefully
        return -1;
    }

    uint8_t data[nbytes + 1];
    data[0] = reg_addr;
    for (int i = 0; i < nbytes; i++) {
        data[i + 1] = buf[i];
    }

    int ret = i2c_write_blocking(_i2c_ch, MCP4725_ADDR, data, (nbytes + 1), false);
    if (ret != PICO_ERROR_NONE) {
        fprintf(stderr, "I2C write error: %d\n", ret);
        // Handle error, throw exception, or exit gracefully
    }

    return ret;
}

int MCP4725::__reg_read(const uint8_t reg_addr, uint8_t* buf, const uint8_t nbytes) {
    // Private member function for reading data from a register
    if (nbytes < 1) {
        fprintf(stderr, "Invalid number of bytes to read: %d\n", nbytes);
        // Handle error, throw exception, or exit gracefully
        return -1;
    }

    // Begin transmission by sending the register to be read
    int ret = i2c_write_blocking(_i2c_ch, MCP4725_ADDR, &reg_addr, 1, true);
    if (ret != PICO_ERROR_NONE) {
        fprintf(stderr, "I2C write error: %d\n", ret);
        // Handle error, throw exception, or exit gracefully
        return ret;
    }

    uint8_t num_bytes_read = i2c_read_blocking(_i2c_ch, MCP4725_ADDR, buf, nbytes, false);
    if (num_bytes_read != nbytes) {
        fprintf(stderr, "I2C read error: expected %d bytes, got %d\n", nbytes, num_bytes_read);
        // Handle error, throw exception, or exit gracefully
        return -1;
    }

    return num_bytes_read;
} */
