#include "control.h"

#include "esp_log.h"

#define NSQUAL

void update_rover_position(spi_device_handle_t *hspi, rover_position_t *pos)
{
	optical_flow_data_t op_flow_data;
	optical_flow_sensor_read(hspi, &op_flow_data);

	// TODO: add check for squal > SQUAL_THRESHOLD
	pos->x += (int32_t)op_flow_data.delta_x;
	pos->y += (int32_t)op_flow_data.delta_y;

#ifndef NSQUAL
	ESP_LOGI(TAG, "squal : %d", op_flow_data.squal);
#endif
}

void update_controller(spi_device_handle_t *hspi, controller_t *controller)
{
	update_rover_position(hspi, &(controller->pos));

	int32_t error_x = controller->setpoint - controller->pos.x;

	controller->error_integral_x += error_x * controller->Ts;

	float p = controller->kp * error_x;

	float i = controller->ki * controller->error_integral_x;

	float d = controller->kd * (error_x - controller->prev_error_x) / controller->Ts;

	controller->motor_delta = p + i + d;
}

void init_controller(float kp_, float ki_, float kd_, float Ts_, int32_t setpoint_, controller_t *controller)
{
	controller->kp = kp_;
	controller->ki = ki_;
	controller->kd = kd_;

	controller->setpoint = setpoint_;
	controller->Ts = Ts_;

	controller->error_integral_x = 0;
	controller->prev_error_x = 0;

	controller->motor_delta = 0;
}
