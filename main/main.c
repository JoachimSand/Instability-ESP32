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

void app_main(void)
{
	// init_motor_drivers();
	init_WIFI();
	const char *str = "Challenger?";
	send_debug_backend(str, strlen(str));

	spi_device_handle_t spi_handle;
	init_opt_flow_sensor(&spi_handle, GPIO_NUM_12, GPIO_NUM_13, GPIO_NUM_14, GPIO_NUM_25, GPIO_NUM_26);
	init_motor_drivers();

	rover_position_t pos;

	controller_t controller;
	init_controller(1, 0, 0, 0.01, 0, &controller);

	// optical_flow_data_t op_flow_data;

	// motor_move(DIR_BACKWARD, 255);
	// motor_rotate_in_place(DIR_RIGHT,  255);
	motor_stop();

	while (1)
	{
		// update_rover_position(&spi_handle, &pos);
		// ESP_LOGI(TAG, "x: %d  | y: %d" , pos.x, pos.y);
		update_controller(&spi_handle, &controller);
		motor_move(DIR_FORWARD, 180, controller.motor_delta);

		// ESP_LOGI(TAG, "x: %d  | y: %d" , controller.pos.x, controller.pos.y);
		ESP_LOGI(TAG, "motor delta: %d", controller.motor_delta);

		// ESP_LOGI(TAG, "dx: %d  | dy %d  | squal: %u" , op_flow_data.delta_x, op_flow_data.delta_y, op_flow_data.squal);

		vTaskDelay(10 / portTICK_PERIOD_MS);

		// motor_move(DIR_FORWARD, 200);
		// vTaskDelay(5000 / portTICK_PERIOD_MS);
		// motor_stop();
	}

	// motor_move(DIR_FORWARD, 200);
	// vTaskDelay(5000 / portTICK_PERIOD_MS);
	// motor_stop();
}
