#include <AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RASPILOT

#include <stdio.h>
#include <errno.h>
#include <termios.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <assert.h>
#include <sys/ioctl.h>

#include "RCInput_Raspilot.h"

extern const AP_HAL::HAL& hal;

using namespace Linux;

void LinuxRCInput_Raspilot::init(void*)
{
    _openUART();
    //_closeUART();
    //_openUART();
}

void LinuxRCInput_Raspilot::_openUART()
{
    _rd_fd = open("/dev/ttyAMA0", O_RDWR);
    _wr_fd = _rd_fd;
    if (_rd_fd == -1) {
        ::fprintf(stdout, "Failed to open UART device %s - %s\n",
                  "/dev/ttyAMA0", strerror(errno));
        hal.scheduler->panic("Failed to open UART device for RCInput");
        return;
    }
    
    // always run the file descriptor non-blocking, and deal with
    // blocking IO in the higher level calls
    fcntl(_rd_fd, F_SETFL, fcntl(_rd_fd, F_GETFL, 0) | O_NONBLOCK);
    
    // set the baud rate
    struct termios t;
    memset(&t, 0, sizeof(t));
    tcgetattr(_rd_fd, &t);
    cfsetspeed(&t, 100000);
    // disable LF -> CR/LF
    t.c_iflag &= ~(BRKINT | ICRNL | IMAXBEL | IXON | IXOFF);
    t.c_oflag &= ~(OPOST | ONLCR);
    t.c_lflag &= ~(ISIG | ICANON | IEXTEN | ECHO | ECHOE | ECHOK | ECHOCTL | ECHOKE);
    t.c_cc[VMIN] = 0;
    //8E2
    t.c_cflag |= PARENB;
    t.c_cflag &= ~PARODD;
    t.c_cflag |= CSTOPB;
    t.c_iflag |= INPCK;
    tcsetattr(_rd_fd, TCSANOW, &t);
}

void LinuxRCInput_Raspilot::_closeUART()
{
    hal.scheduler->delay(1);
    if (_rd_fd == _wr_fd && _rd_fd != -1) {
        close(_rd_fd);
    }
    _rd_fd = -1;
    _wr_fd = -1;
}

void LinuxRCInput_Raspilot::_timer_tick()
{
    uint8_t rx;
    static uint8_t buffer[25];
    static uint8_t buffer_index = 0;
    
    while ( ::read(_rd_fd, &rx, 1) ) {
        if (buffer_index == 0 && rx != 0x0f) {
            //incorrect start byte, out of sync
            continue;
        }
        
        buffer[buffer_index++] = rx;
        
        if (buffer_index == 25) {
            buffer_index = 0;
            
            _process_sbus_bytes(buffer);
        }
    }
}

#endif // CONFIG_HAL_BOARD_SUBTYPE
