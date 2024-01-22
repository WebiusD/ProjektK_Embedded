#include "TMC2209.h"

const uint8_t GP_ENABLE = 10;
const uint8_t GP_STEP = 11;
const uint8_t GP_DIR = 12;
const uint8_t GP_OVTEMP = 21; // Index gpio
const uint8_t GP_DIAG = 20;

int main() {
    // important for any kind of serial io:
    stdio_init_all();
    
    TMC2209 tmc(uart1, 115200, GP_ENABLE);
    tmc.index_fn_select(IndexFn::ShowOvertempWarning);
    gpio_init(GP_DIAG);
    gpio_set_dir(GP_DIAG, GPIO_IN);

    uint8_t version = tmc.get_version();
    printf("Version: %d\n");

    // tmc.set_step_dir_pins(GP_STEP, GP_DIR);
    tmc.enable_stealth_chop();
    tmc.use_external_rsense();

    tmc.set_coolstep_and_stalldiag_lower_threshold(12000);
    tmc.set_stallguard_threshold(85);

    tmc.set_ihold_percent(5);
    tmc.set_hold_delay_percent(10);
    tmc.set_powerdown_delay(0.0f);

    // tmc.set_standstill_mode(StandstillMode::Brake);

    // enable the driver stage:
    tmc.enable();
    tmc.set_irun_percent(80);

    tmc.move_at_velocity(20000);
    sleep_ms(8000);

    tmc.move_at_velocity(0);
    /* // use full steps:
    tmc.set_microstep_res(MicrostepResolution::MRES_001);
    tmc.move_steps(1000, Direction::CW);
    sleep_ms(1000);
    tmc.move_steps(1000, Direction::CCW);
    sleep_ms(1000);

    // use 64 microsteps:
    tmc.set_microstep_res(MicrostepResolution::MRES_064);
    tmc.move_steps(1000, Direction::CW);
    sleep_ms(1000);
    tmc.move_steps(1000, Direction::CCW);
    sleep_ms(1000);

    // use 256 microsteps:
    tmc.set_microstep_res(MicrostepResolution::MRES_256);
    tmc.move_steps(1000, Direction::CW);
    sleep_ms(1000);
    tmc.move_steps(1000, Direction::CCW); */

    sleep_ms(5000);
    // Disable the dirver:
    tmc.disable();

    /* for(uint8_t i=0; i<4; i++) {
        tmc.move_at_velocity(512);
        sleep_ms(1000);

        tmc.move_at_velocity(1024);
        sleep_ms(1000);
    }

    tmc.disable(); */
}