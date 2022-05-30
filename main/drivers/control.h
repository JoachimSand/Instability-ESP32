#ifndef CONTROL_H
#define CONTROL_H

#include "driver/spi_master.h"
#include "motor_driver.h"
#include "optical_flow_sensor.h"

typedef struct rover_position
{
    int32_t x;
    int32_t y;
} rover_position_t;

typedef struct controller
{
    //paramaters
    float kp;
    float ki;
    float kd;

    float Ts;

    //input
    int32_t setpoint;

    // internal state
    rover_position_t pos;

    int32_t error_integral_x;
    int32_t prev_error_x;

    // output
    int16_t motor_delta;

}controller_t;


void update_rover_position(spi_device_handle_t* hspi, rover_position_t* pos);

void update_controller(spi_device_handle_t* hspi, controller_t* controller);

void init_controller(float kp_, float ki_, float kd_, float Ts_, int32_t setpoint_, controller_t* controller);

#endif
