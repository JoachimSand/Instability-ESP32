#ifndef CONTROL_H
#define CONTROL_H

#include "driver/spi_master.h"
#include "motor_driver.h"
#include "optical_flow_sensor.h"

typedef struct rover_position
{
    int32_t x;
    int32_t y;
    uint8_t squal;
} rover_position_t;

typedef struct controller
{
    //paramaters
    float kp;
    float ki;
    float kd;

    uint8_t control_axis;

    float Ts;

    //input
    int32_t setpoint;

    // internal state
    rover_position_t* pos;

    int32_t error_integral;
    int32_t prev_error;

    // output
    int16_t output;

}controller_t;

#define AXIS_X 1
#define AXIS_ROTATE 1
#define AXIS_Y 0

#define CONTROLER_IN_PROGRESS 0 
#define CONTROLLER_REACHED_SETPOINT 1

#define CONTROLLER_THRESHOLD 25

#define FORWARD_CNT_PER_CM 48
#define ROTATION_CNT_PER_DEGRES 12.9f
// #define ROTATION_CNT_PER_DEGRES 13.0f


void update_rover_position(spi_device_handle_t* hspi, rover_position_t* pos, uint8_t curr_dir);

uint8_t update_controller(spi_device_handle_t* hspi, controller_t* controller);

void init_controller(float kp_, float ki_, float kd_, float Ts_, int32_t setpoint_, uint8_t control_axis_, rover_position_t* pos_, controller_t* controller);

#endif
