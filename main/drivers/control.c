#include "control.h"

#include "esp_log.h"

// #define NSQUAL

void update_rover_position(spi_device_handle_t* hspi, rover_position_t *pos)
{
    optical_flow_data_t op_flow_data;
    optical_flow_sensor_read(hspi, &op_flow_data);

    // TODO: add check for squal > SQUAL_THRESHOLD
    pos->x +=  (int32_t) op_flow_data.delta_x;
    pos->y +=  (int32_t) op_flow_data.delta_y;

#ifndef NSQUAL
    ESP_LOGI(TAG, "squal : %d", op_flow_data.squal);
#endif
}


void update_controller(spi_device_handle_t* hspi, controller_t* controller)
{
    // Don't update here to avoid double op flow read
    // update_rover_position(hspi, &(controller->pos));

    int32_t error;
    if (controller->control_axis == AXIS_X)
    {
        error = controller->setpoint - controller->pos->x;
    }
    else
    {
        error = controller->setpoint - controller->pos->y;
    }

    controller->error_integral +=  error * controller->Ts;

    float p = controller->kp * error; 

    float i = controller->ki * controller->error_integral;

    float d = controller->kd * (error - controller->prev_error) / controller->Ts;

    controller->output = p + i + d;
}

void init_controller(float kp_, float ki_, float kd_, float Ts_, int32_t setpoint_, uint8_t control_axis_, rover_position_t* pos_ , controller_t* controller)
{
    controller->kp = kp_;
    controller->ki = ki_;
    controller->kd = kd_;

    controller->setpoint = setpoint_;
    controller->Ts = Ts_;

    controller->pos = pos_;

    // controller->pos.x = 0;
    // controller->pos.y = 0;

    controller->control_axis = control_axis_;

    controller->error_integral = 0;
    controller->prev_error = 0;
    
    controller->output = 0;
}
