#include <AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RASPILOT

#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <assert.h>

#include "RCInput_Raspilot.h"

static const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

using namespace Linux;

void LinuxRCInput_Raspilot::init(void*)
{
    _i2c_sem = hal.i2c->get_semaphore();
    if (_i2c_sem == NULL) {
        hal.scheduler->panic(PSTR("PANIC: RCIutput_Raspilot did not get "
                                  "valid I2C semaphore!"));
        return; // never reached
    }
    
    // start the timer process to read samples
    hal.scheduler->register_timer_process(AP_HAL_MEMBERPROC(&LinuxRCInput_Raspilot::_poll_data));
}

void LinuxRCInput_Raspilot::_poll_data(void)
{
    // Throttle read rate to 100hz maximum.
    if (hal.scheduler->micros() - _last_timer < 10000) {
        return;
    }
    
    _last_timer = hal.scheduler->micros();
    
    if (!_i2c_sem->take_nonblocking()) {
        return;
    }
    
    uint8_t data[28];
    
    if ( hal.i2c->read_rpio(RPILOTIO_ADDRESS, 4, 0, 28, data) == 0 )
    {
        _process_rpio_bytes(data);
    }
    
    _i2c_sem->give();
}

#endif // CONFIG_HAL_BOARD_SUBTYPE
