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
	// const char *str = "Challenger?";
	// send_debug_backend(str, strlen(str));

	spi_device_handle_t spi_handle;
	init_opt_flow_sensor(&spi_handle, GPIO_NUM_12, GPIO_NUM_13, GPIO_NUM_14, GPIO_NUM_25, GPIO_NUM_26);
	init_motor_drivers();

    motor_stop();
    
	rover_position_t rover_pos;
	// TODO: separate into init pos function
	rover_pos.x = 0;
	rover_pos.y = 0;

    rover_position_t test_pos;
	// TODO: separate into init pos function
	test_pos.x = 250;
	test_pos.y = 25;


	// controller_t controller_sideways;
	// controller_t controller_forward;
    controller_t controller;

	// init_controller(5, 0, 0, 0.01, 0, AXIS_X, &rover_pos, &controller_sideways);
	// init_controller(1, 0, 0, 0.01, 4700, AXIS_Y, &rover_pos, &controller_forward);

    init_controller(5, 0, 0, 0.01, 1960, AXIS_ROTATE, &rover_pos, &controller);
    

	while (1)
	{
        // TODO: main controller code here
		// update_rover_position(&spi_handle, &rover_pos);
		// update_controller(&spi_handle, &controller_forward);
		// update_controller(&spi_handle, &controller_sideways);
		// update_controller(&spi_handle, &controller);

		// PID MOTOR CONTROL FORWARD
		// motor_move(DIR_FORWARD, saturate_uint8_to_val(controller_forward.output, MAX_FORWARD_VEL), controller_sideways.output);

        // motor_rotate_in_place(DIR_LEFT, saturate_to_uint8(controller.output));

		// ESP_LOGI(TAG, "x: %d  | y: %d", controller_sideways.pos->x, controller_sideways.pos->y);
        // ESP_LOGI(TAG, "controller output: %d | saturated uint8_t output: %d" , controller.output, saturate_to_uint8(controller.output));

        // vTaskDelay(10 / portTICK_PERIOD_MS);
        //


        // TODO: Alex wifi testing code here
        //
        if (test_pos.x <= 400)
        {
            send_live_position(&test_pos);
            test_pos.x += 10;
        }
        vTaskDelay(2000 / portTICK_PERIOD_MS);
	}
}
