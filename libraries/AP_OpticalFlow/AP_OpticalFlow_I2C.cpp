/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
 *       AP_OpticalFlow_I2C.cpp - ardupilot library for PX4Flow sensor
 *
 */

#include <AP_HAL.h>
#include "OpticalFlow.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX && CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RASPILOT

/* Configuration Constants */
#define I2C_FLOW_ADDRESS 		0x42	///< 7-bit address. 8-bit address is 0x84, range 0x42 - 0x49

/* PX4FLOW Registers addresses */
#define PX4FLOW_REG			0x16	///< Measure Register 22

#define PX4FLOW_CONVERSION_INTERVAL	100000	///< in microseconds! 20000 = 50 Hz 100000 = 10Hz
#define PX4FLOW_I2C_MAX_BUS_SPEED	400000	///< 400 KHz maximum speed

typedef struct i2c_integral_frame
{
    uint16_t frame_count_since_last_readout;
    int16_t pixel_flow_x_integral;
    int16_t pixel_flow_y_integral;
    int16_t gyro_x_rate_integral;
    int16_t gyro_y_rate_integral;
    int16_t gyro_z_rate_integral;
    uint32_t integration_timespan;
    uint32_t sonar_timestamp;
    uint16_t ground_distance;
    int16_t gyro_temperature;
    uint8_t qual;
} i2c_integral_frame;

#define I2C_INTEGRAL_FRAME_SIZE (sizeof(i2c_integral_frame))

extern const AP_HAL::HAL& hal;

AP_OpticalFlow_I2C::AP_OpticalFlow_I2C(OpticalFlow &_frontend) :
OpticalFlow_backend(_frontend)
{}


void AP_OpticalFlow_I2C::init(void)
{
    hal.scheduler->suspend_timer_procs();
    hal.scheduler->delay(10);

    // get pointer to i2c bus semaphore
    AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();

    // exit immediately if we can't take the semaphore
    if (i2c_sem == NULL || !i2c_sem->take(1)) {
        hal.scheduler->panic(PSTR("Failed to get PX4Flow semaphore"));
    }

    i2c_sem->give();
    hal.scheduler->resume_timer_procs();

    return;
}

// update - read latest values from sensor and fill in x,y and totals.
void AP_OpticalFlow_I2C::update(void)
{
    struct i2c_integral_frame f_integral;

    uint8_t val[I2C_INTEGRAL_FRAME_SIZE] = { 0 };

    /*if (hal.scheduler->micros() - _last_timestamp < 100000) {
        return;
    }*/

    // get pointer to i2c bus semaphore
    AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();

    // exit immediately if we can't take the semaphore
    if (i2c_sem == NULL || !i2c_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return;
    }

    if ( hal.i2c->readRegisters(I2C_FLOW_ADDRESS, PX4FLOW_REG, I2C_INTEGRAL_FRAME_SIZE, &val[0]) != 0 ) {
        i2c_sem->give();
        return;
    }

    memcpy(&f_integral, val, I2C_INTEGRAL_FRAME_SIZE);

    struct OpticalFlow::OpticalFlow_state state;
    state.device_id = 0;
    state.surface_quality = f_integral.qual;

    if (f_integral.integration_timespan > 0) {
        float yawAngleRad = _yawAngleRad();
        float cosYaw = cosf(yawAngleRad);
        float sinYaw = sinf(yawAngleRad);
        const Vector2f flowScaler = _flowScaler();
        float flowScaleFactorX = 1.0f + 0.001f * flowScaler.x;
        float flowScaleFactorY = 1.0f + 0.001f * flowScaler.y;
        float integralToRate = 1e6f / float(f_integral.integration_timespan);
        // rotate sensor measurements from sensor to body frame through sensor yaw angle
        state.flowRate.x = flowScaleFactorX * integralToRate * (cosYaw * float(f_integral.pixel_flow_x_integral) - sinYaw * float(f_integral.pixel_flow_y_integral)) / 10000.0f; // rad/sec measured optically about the X body axis
        state.flowRate.y = flowScaleFactorY * integralToRate * (sinYaw * float(f_integral.pixel_flow_x_integral) + cosYaw * float(f_integral.pixel_flow_y_integral)) / 10000.0f; // rad/sec measured optically about the Y body axis
        state.bodyRate.x = integralToRate * (cosYaw * float(f_integral.gyro_x_rate_integral) - sinYaw * float(f_integral.gyro_y_rate_integral)) / 10000.0f; // rad/sec measured inertially about the X body axis
        state.bodyRate.y = integralToRate * (sinYaw * float(f_integral.gyro_x_rate_integral) + cosYaw * float(f_integral.gyro_y_rate_integral)) / 10000.0f; // rad/sec measured inertially about the Y body axis
    } else {
        state.flowRate.zero();
        state.bodyRate.zero();
    }

    i2c_sem->give();
    _last_timestamp = hal.scheduler->micros();

    _update_frontend(state);
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_LINUX && CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RASPILOT
