#include "HX711.h"

volatile bool timer_fired = false;
int64_t alarm_callback(alarm_id_t id, void *user_data) {
    timer_fired = true;
    // required:
    return 0;
}

HX711::HX711(uint8_t gp_dout, uint8_t gp_sck, PgaGain gain) 
    : _gp_dout(gp_dout), _gp_sck(gp_sck), _offset(0), _conversion_factor(1.0f) {
    
    // setup dout as input cause bits are read by it, default is high, so use a pull-up.
    gpio_init(_gp_dout);
    gpio_set_dir(_gp_dout, GPIO_IN);
    gpio_pull_up(_gp_dout);

    // set the sck as output, cause it has to create clock pulses:
    gpio_init(_gp_sck);
    gpio_set_dir(_gp_sck, GPIO_OUT);

    power_up();

    // set the channel and gain, by sending clock pulses:
    set_pga_gain(gain);
};

bool HX711::is_ready(long timeout_ms) {
    if(timeout_ms <= 0) {
        // when data is ready the dout line is pulled low:
        return gpio_get(_gp_dout) == 0;
    }

    // create a timer
    add_alarm_in_ms(timeout_ms, alarm_callback, NULL, false);
    // if dout goes low before the timer fires, the device is ready
    while (!timer_fired) {
        if(!gpio_get(_gp_dout))
            return true;
    }

    // timer timed out before data was ready:
    return false;
}

void HX711::set_pga_gain(PgaGain gain) {
    _pga_gain = gain;

    // set the gain for the next measurement by sending 24 
    // + 1 (channel A - gain 128)
    // + 2 (channel B - gain 32)
    // + 3 (channel A - gain 64)
    // clock pulses.
    uint8_t clock_pulses = 24 + static_cast<uint8_t>(_pga_gain);
    _pulse_clock(clock_pulses);
}

void HX711::set_offset(int32_t offset) {
    _offset = offset;
}

int32_t HX711::get_offset() {
    return _offset;
}

float HX711::get_conversion() {
    return _conversion_factor;
}

// make sure to check the device is ready:
int32_t HX711::read_raw() {
    uint32_t value = 0;
    // shift in the data in byte sized chunks: 
    uint8_t data[3];

    // wait till the device signals that its ready:
    while(!is_ready()) {
        tight_loop_contents();
    }
    
    // when an interrupts occurs during shifting in the bits, the clock cycle will
    // be stretched, when a high level is present for longer than 60us the hx711 will
    // enter power-down mode. To prevent this we temporarily disable interrupts:
    uint32_t state = save_and_disable_interrupts();

    data[2] = _shift_in(false);
    data[1] = _shift_in(false);
    data[0] = _shift_in(false);

    // Shifting in data requires 24 clock pulses. The channel and gain of the next reading is set
    // by sending one (channel A - gain 124), two (channel B - gain 32) or three (channel A - gain 64)
    // additional clock pulses respectively.
    uint8_t additional_pulses = static_cast<uint8_t>(_pga_gain);
    _pulse_clock(additional_pulses);

    // enable interrupts and restore the previous state:
    restore_interrupts(state);
    
    // construct a 32 bit signed integer out of the three bytes:
    // note if the most significant bit of data[2] is set, the number is negative!
    // so extend the sign of the 24 bit number:
    uint8_t sign_ext;
	if (data[2] & 0x80) {
		sign_ext = 0xFF;
	} else {
		sign_ext = 0x00;
	}

	// Construct a 32-bit signed integer
	value = ( static_cast<uint32_t>(sign_ext)  << 24
			| static_cast<uint32_t>(data[2]) << 16
			| static_cast<uint32_t>(data[1]) << 8
			| static_cast<uint32_t>(data[0]) );

	return static_cast<int32_t>(value);
}

int32_t HX711::read_raw_avg(uint8_t times) {
    uint32_t sum = 0;
    for(uint8_t i=0; i<times; i++) {
        sum += read_raw();
    }

    return sum / times;
}

void HX711::tare(uint8_t times) {
    int32_t avg = read_raw_avg(times);

    _offset = avg;
}

double HX711::read(uint8_t times) {
    return (double)((read_raw_avg(times) - _offset) / _conversion_factor);
}

void HX711::power_down() {
    gpio_put(_gp_sck, 0);
    gpio_put(_gp_sck, 1);

    // after reset or power-down the channel and gain selection defaults to A/128
    // datasheet p. 5
    _pga_gain = PgaGain::A128;
}

void HX711::power_up() {
    gpio_put(_gp_sck, 0);
}

void HX711::calibrate_gramm(uint32_t weight_in_gramms) {
    // set the conversion factor to one such that, read() returns just the raw value minus the offset:
    _conversion_factor = 1.0f;

    // 10 times averaged reading of the scale without conversion:
    double unconverted_value = read(10);

    // adjust the conversion factor, such that read, will now yield weight in gramms:
    // read function will now return: unconverted_value / (unconverted_value / weight_in_gramms) = weight_in_gramms:
    _conversion_factor = unconverted_value / weight_in_gramms;
}

uint8_t HX711::_shift_in(bool lsbFirst) {
    uint8_t value = 0;

    for(uint8_t i=0; i<8; i++) {
        gpio_put(_gp_sck, 1);
        // sleep_us(1) <-- needed?
        if(lsbFirst) {
            value |= (gpio_get(_gp_dout) << i); 
        } else {
            value |= (gpio_get(_gp_dout) << (7-i));
        }
        gpio_put(_gp_sck, 0);
        // sleep_us(1) <-- needed?
    }
    return value;
}

void HX711::_pulse_clock(uint8_t times) {
    for(uint8_t i=0; i<times; i++) {
        gpio_put(_gp_sck, 1);
        sleep_us(1);
        gpio_put(_gp_sck, 0);
        sleep_us(1);
    }
}