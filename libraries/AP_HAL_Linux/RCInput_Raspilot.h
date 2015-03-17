
#ifndef __AP_HAL_LINUX_RCINPUT_RASPILOT_H__
#define __AP_HAL_LINUX_RCINPUT_RASPILOT_H__

#include <AP_HAL_Linux.h>
#include "RCInput.h"

class Linux::LinuxRCInput_Raspilot : public Linux::LinuxRCInput
{
public:
    void init(void*);
    void _timer_tick(void);
    
private:
    int _rd_fd;
    int _wr_fd;
    
    void _openUART(void);
    void _closeUART(void);
};

#endif // __AP_HAL_LINUX_RCINPUT_RASPILOT_H__
