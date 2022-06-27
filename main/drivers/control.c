#include "control.h"

#include "esp_log.h"
#include "pathfinding.h"

#define NSQUAL

void update_rover_position(spi_device_handle_t* hspi, rover_position_t* pos, uint8_t curr_dir)
{
	optical_flow_data_t op_flow_data;
	optical_flow_sensor_read(hspi, &op_flow_data);

    if (curr_dir == POS_X)
    {
        pos->x += (int32_t)op_flow_data.delta_y;
        pos->y += (int32_t)op_flow_data.delta_x;
    }
    if (curr_dir == NEG_X)
    {
        pos->x -= (int32_t)op_flow_data.delta_y;
        pos->y -= (int32_t)op_flow_data.delta_x;
    }
    if (curr_dir == POS_Y)
    {
        pos->x += (int32_t)op_flow_data.delta_x;
        pos->y += (int32_t)op_flow_data.delta_y;
    }
    if (curr_dir == NEG_Y)
    {
        pos->x -= (int32_t)op_flow_data.delta_x;
        pos->y -= (int32_t)op_flow_data.delta_y;
    }

    pos->squal = op_flow_data.squal;

#ifndef NSQUAL
	ESP_LOGI(TAG, "squal : %d", op_flow_data.squal);
#endif
}

uint8_t update_controller(spi_device_handle_t *hspi, controller_t *controller)
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

	controller->error_integral += error * controller->Ts;

	float p = controller->kp * error;

	float i = controller->ki * controller->error_integral;

	float d = controller->kd * (error - controller->prev_error) / controller->Ts;

	controller->output = p + i + d;

    return (abs(error) < CONTROLLER_THRESHOLD);
}

void init_controller(float kp_, float ki_, float kd_, float Ts_, int32_t setpoint_, uint8_t control_axis_, rover_position_t *pos_, controller_t *controller)
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
