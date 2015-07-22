/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef AP_OpticalFlow_I2C_H
#define AP_OpticalFlow_I2C_H

#include "OpticalFlow.h"
#include <AP_HAL.h>

class AP_OpticalFlow_I2C : public OpticalFlow_backend
{
public:
    /// constructor
    AP_OpticalFlow_I2C(OpticalFlow &_frontend);

    // init - initialise the sensor
    void init();

    // update - read latest values from sensor and fill in x,y and totals.
    void update(void);

private:
    uint64_t    _last_timestamp;    // time of last update (used to avoid processing old reports)
};

#endif
