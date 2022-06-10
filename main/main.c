//#include <bits/stdint-intn.h>
//#include <bits/stdint-uintn.h>
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
	// WIFI stuffs
	init_WIFI();

	spi_device_handle_t spi_handle;
	init_opt_flow_sensor(&spi_handle, GPIO_NUM_12, GPIO_NUM_13, GPIO_NUM_14, GPIO_NUM_25, GPIO_NUM_26);
	init_motor_drivers();

	motor_stop();

	rover_position_t rover_pos;
	// TODO: separate into init pos function
	rover_pos.x = 0;
	rover_pos.y = 0;
	rover_pos.squal = 0;

	controller_t controller_sideways;
	controller_t controller_forward;
	// controller_t controller_rotate;

	init_controller(0.60, 0, 0, 0.01, 0, AXIS_X, &rover_pos, &controller_sideways);
	init_controller(1, 0, 0, 0.01, 4700, AXIS_Y, &rover_pos, &controller_forward);

	// init_controller(5, 0, 0, 0.01, 1960, AXIS_ROTATE, &rover_pos, &controller);
	//

	while (1)
	{
		// TODO: main controller code here
		update_rover_position(&spi_handle, &rover_pos);
		update_controller(&spi_handle, &controller_forward);
		update_controller(&spi_handle, &controller_sideways);
		// update_controller(&spi_handle, &controller);

		// PID MOTOR CONTROL FORWARD
		uint8_t motor_base_speed = saturate_uint8_to_val(controller_forward.output, MAX_FORWARD_VEL);
		int16_t unsaturated_motor_delta = controller_sideways.output;

		motor_move(DIR_FORWARD, motor_base_speed, unsaturated_motor_delta);

		// motor_rotate_in_place(DIR_LEFT, saturate_to_uint8(controller.output));

		// ESP_LOGI(TAG, "x: %d  | y: %d", controller_sideways.pos->x, controller_sideways.pos->y);
		// ESP_LOGI(TAG, "controller output: %d | saturated uint8_t output: %d" , controller.output, saturate_to_uint8(controller.output));

		vTaskDelay(10 / portTICK_PERIOD_MS);

		// TODO: Alex wifi testing code here
		// if (!has_sent_path)
		// {
		// while (curr_pos.x < 100)
		// {
		// send_live_position(&curr_pos);
		// curr_pos.x += 10;
		// curr_pos.y += 10;
		// vTaskDelay(500 / portTICK_PERIOD_MS);
		// }
		// vTaskDelay(500 / portTICK_PERIOD_MS);
		// send_path(&init_pos, &curr_pos);
		// init_pos.x = curr_pos.x;
		// init_pos.y = curr_pos.y;
		// vTaskDelay(5000 / portTICK_PERIOD_MS);
		// while (curr_pos.x > 0 && !has_sent_path)
		// {
		// send_live_position(&curr_pos);
		// curr_pos.x -= 10;
		// vTaskDelay(500 / portTICK_PERIOD_MS);
		// }
		// vTaskDelay(500 / portTICK_PERIOD_MS);
		// send_path(&init_pos, &curr_pos);
		// vTaskDelay(500 / portTICK_PERIOD_MS);
		//
		// send_alien_position(&alien_pos);
		//
		// has_sent_path = 1;
		// }
		// vTaskDelay(500 / portTICK_PERIOD_MS);
	}
}
