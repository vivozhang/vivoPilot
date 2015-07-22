// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       AP_RangeFinder_PX4FLOW.cpp - Arduino Library for PX4FLOW I2C sonar
 *
 *
 *       Sensor should be connected to the I2C port
 */

#include "AP_RangeFinder_PX4FLOW.h"
#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX && CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RASPILOT

/* Configuration Constants */
#define I2C_FLOW_ADDRESS 		0x42	///< 7-bit address. 8-bit address is 0x84, range 0x42 - 0x49

#define PX4FLOW_CONVERSION_INTERVAL	100000	///< in microseconds! 20000 = 50 Hz 100000 = 10Hz
#define PX4FLOW_I2C_MAX_BUS_SPEED	400000	///< 400 KHz maximum speed

typedef struct i2c_frame
{
    uint16_t frame_count;
    int16_t pixel_flow_x_sum;
    int16_t pixel_flow_y_sum;
    int16_t flow_comp_m_x;
    int16_t flow_comp_m_y;
    int16_t qual;
    int16_t gyro_x_rate;
    int16_t gyro_y_rate;
    int16_t gyro_z_rate;
    uint8_t gyro_range;
    uint8_t sonar_timestamp;
    int16_t ground_distance;
} i2c_frame;

#define I2C_FRAME_SIZE (sizeof(i2c_frame))

extern const AP_HAL::HAL& hal;

/*
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_PX4FLOW::AP_RangeFinder_PX4FLOW(RangeFinder &_ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state) :
    AP_RangeFinder_Backend(_ranger, instance, _state)
{
}

/*
   detect if a PX4FLOW is connected. We'll detect by
   trying to take a reading on I2C. If we get a result the sensor is
   there.
*/
bool AP_RangeFinder_PX4FLOW::detect(RangeFinder &_ranger, uint8_t instance)
{
    // get pointer to i2c bus semaphore
    AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();

    // exit immediately if we can't take the semaphore
    if (i2c_sem == NULL || !i2c_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
    } else {
        i2c_sem->give();
        return true;
    }
}

// read - return last value measured by sensor
bool AP_RangeFinder_PX4FLOW::get_reading(uint16_t &reading_cm)
{
    struct i2c_frame f;

    uint8_t val[I2C_FRAME_SIZE] = { 0 };

    // get pointer to i2c bus semaphore
    AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();

    // exit immediately if we can't take the semaphore
    if (i2c_sem == NULL || !i2c_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
    }

    if ( hal.i2c->readRegisters(I2C_FLOW_ADDRESS, 0, I2C_FRAME_SIZE, &val[0]) != 0 ) {
        i2c_sem->give();
        return false;
    }

    memcpy(&f, val, I2C_FRAME_SIZE);

    i2c_sem->give();

    // combine results into distance
    //if ( (f.qual>0) && (f.ground_distance>0) ) reading_cm = f.ground_distance / 10;
    reading_cm = f.ground_distance / 10;

    return true;
}


/*
   update the state of the sensor
*/
void AP_RangeFinder_PX4FLOW::update(void)
{
    if (get_reading(state.distance_cm)) {
        // update range_valid state based on distance measured
        update_status();
    } else {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_LINUX && CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RASPILOT
