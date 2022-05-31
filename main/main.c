#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/ledc.h"

#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/portmacro.h"
#include "hal/gpio_types.h"
#include "hal/ledc_types.h"

// Include FreeRTOS for delay
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sys/types.h>

#include "esp_log.h"
#include "rom/ets_sys.h"

#include "platform.h"
#include "drivers/motor_driver.h"
#include "drivers/optical_flow_sensor.h"
#include "drivers/backend_connect.h"
#include "drivers/control.h"

#define MAX_FORWARD_VEL 180

void app_main(void)
{
	// init_motor_drivers();
	init_WIFI();
	const char *str = "Challenger?";
	send_debug_backend(str, strlen(str));

	spi_device_handle_t spi_handle;
	init_opt_flow_sensor(&spi_handle, GPIO_NUM_12, GPIO_NUM_13, GPIO_NUM_14, GPIO_NUM_25, GPIO_NUM_26);
	init_motor_drivers();

	rover_position_t rover_pos;

	// TODO: separate into init pos function
	rover_pos.x = 0;
	rover_pos.y = 0;

	controller_t controller_sideways;
	controller_t controller_forward;
	// controller_t controller;

	init_controller(5, 0, 0, 0.01, 0, AXIS_X, &rover_pos, &controller_sideways);
	init_controller(1, 0, 0, 0.01, 4700, AXIS_Y, &rover_pos, &controller_forward);

	// init_controller(1, 0, 0, 0.01, 984, AXIS_ROTATE, &controller);

	// TURNING
	// init_controller(1, 0, 0, 0.01, 981, &controller);
	// init_controller(1, 0, 0, 0.01, 1962, &controller);

	// optical_flow_data_t op_flow_data;

	while (1)
	{
		// update_rover_position(&spi_handle, &pos);
		// ESP_LOGI(TAG, "x: %d  | y: %d" , pos.x, pos.y);
		update_rover_position(&spi_handle, &rover_pos);
		update_controller(&spi_handle, &controller_forward);
		update_controller(&spi_handle, &controller_sideways);

		// PID MOTOR CONTROL FORWARD
		motor_move(DIR_FORWARD, saturate_uint8_to_val(controller_forward.output, MAX_FORWARD_VEL), controller_sideways.output);
		// motor_move(DIR_FORWARD, 180, controller_sideways.output);

		// motor_rotate_in_place(DIR_LEFT, saturate_to_uint8(controller.output));

		ESP_LOGI(TAG, "x: %d  | y: %d", controller_sideways.pos->x, controller_sideways.pos->y);
		// ESP_LOGI(TAG, "controller output: %d | saturated uint8_t output: %d" , controller_sideways.output, saturate_to_uint8(controller_sideways.output));

		// motor_move(DIR_FORWARD, 200);
		// vTaskDelay(5000 / portTICK_PERIOD_MS);
		// motor_stop();
	}
}
