
#ifndef __AP_HAL_LINUX_RCINPUT_RASPILOT_H__
#define __AP_HAL_LINUX_RCINPUT_RASPILOT_H__

#include <AP_HAL_Linux.h>
#include "RCInput.h"

#define RPILOTIO_ADDRESS             0x1a //Raspilotio default

class Linux::LinuxRCInput_Raspilot : public Linux::LinuxRCInput
{
public:
    void init(void*);
    
private:
    uint32_t _last_timer;
    
    AP_HAL::Semaphore *_i2c_sem;
    
    void _poll_data(void);
};

#endif // __AP_HAL_LINUX_RCINPUT_RASPILOT_H__
