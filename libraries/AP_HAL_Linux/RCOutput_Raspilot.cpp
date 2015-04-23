
#include <AP_HAL.h>
#include "GPIO.h"

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RASPILOT

#include "RCOutput_Raspilot.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

using namespace Linux;

#define PWM_CHAN_COUNT 8

static const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

static uint8_t reg_FS[4] = {50, 1, 75, 0};
static uint8_t reg_FA[4] = {50, 12, 11, 86};

void LinuxRCOutput_Raspilot::init(void* machtnicht)
{
    _i2c_sem = hal.i2c->get_semaphore();
    if (_i2c_sem == NULL) {
        hal.scheduler->panic(PSTR("PANIC: RCOutput_Raspilot did not get "
                                  "valid I2C semaphore!"));
        return; // never reached
    }

    // Set the initial frequency
    set_freq(0, 50);
    
    hal.scheduler->register_timer_process(AP_HAL_MEMBERPROC(&LinuxRCOutput_Raspilot::_update));
}

void LinuxRCOutput_Raspilot::set_freq(uint32_t chmask, uint16_t freq_hz)
{    
    if (!_i2c_sem->take(10)) {
        return;
    }
    
    uint8_t data[4] = {50, 3, freq_hz & 0xff, freq_hz >> 8};
    hal.i2c->write(RPILOTIO_ADDRESS, sizeof(data), data);
    
    _frequency = freq_hz;
    
    _i2c_sem->give();
}

uint16_t LinuxRCOutput_Raspilot::get_freq(uint8_t ch)
{
    return _frequency;
}

void LinuxRCOutput_Raspilot::enable_ch(uint8_t ch)
{
    
}

void LinuxRCOutput_Raspilot::disable_ch(uint8_t ch)
{
    write(ch, 0);
}

void LinuxRCOutput_Raspilot::write(uint8_t ch, uint16_t period_us)
{   
    if(ch >= PWM_CHAN_COUNT){
        return;
    }
    
    _period_us[ch] = period_us;
}

void LinuxRCOutput_Raspilot::write(uint8_t ch, uint16_t* period_us, uint8_t len)
{
    for (int i = 0; i < len; i++)
        write(ch + i, period_us[i]);
}

uint16_t LinuxRCOutput_Raspilot::read(uint8_t ch)
{
    if(ch >= PWM_CHAN_COUNT){
        return 0;
    }
    
    return _period_us[ch];
}

void LinuxRCOutput_Raspilot::read(uint16_t* period_us, uint8_t len)
{
    for (int i = 0; i < len; i++) 
        period_us[i] = read(0 + i);
}

void LinuxRCOutput_Raspilot::_update(void)
{
    int i;
    
    if (hal.scheduler->micros() - _last_update_timestamp < 10000) {
        return;
    }
    
    _last_update_timestamp = hal.scheduler->micros();
    
    if (!_i2c_sem->take_nonblocking()) {
        return;
    }
    
    hal.i2c->write(RPILOTIO_ADDRESS, sizeof(reg_FS), reg_FS);
    hal.i2c->write(RPILOTIO_ADDRESS, sizeof(reg_FA), reg_FA);
    
    uint8_t data[18];
    data[0] = 54;
    data[1] = 0;
    
    for (i=0; i<8; i++) {
        data[2+2*i] = _period_us[i] & 0xff;
        data[3+2*i] = _period_us[i] >> 8;
    }
    
    hal.i2c->write(RPILOTIO_ADDRESS, sizeof(data), data);
    
    _i2c_sem->give();
}

#endif // CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RASPILOT
