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
#include "drivers/fpga_connect.h"
#include "drivers/backend_connect.h"
#include "drivers/control.h"

#define SEND_LIVE_UPDATE_RATE 100

void app_main(void)
{
	// WIFI stuffs
	// init_WIFI();
	spi_device_handle_t spi_handle;
	alien_collection_t alien_collect;
	init_fpga_connection(&spi_handle);
	while (1)
	{
		get_vision_data(&spi_handle, &alien_collect);
	}
}

/*
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

	rover_position_t init_pos;
	// TODO: separate into init pos function
	init_pos.x = 0;
	init_pos.y = 0;
    init_pos.squal = 0;

    controller_t controller_sideways;
    controller_t controller_forward;
    // controller_t controller_rotate;

	init_controller(0.60, 0, 0, 0.01, 0, AXIS_X, &rover_pos, &controller_sideways);
	init_controller(1, 0, 0, 0.01, 4700, AXIS_Y, &rover_pos, &controller_forward);

    uint16_t ticks_since_last_live_pos = 0;



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

        uint8_t speed_channel_0 = saturate_to_uint8((int16_t)motor_base_speed - (int16_t) unsaturated_motor_delta);
        uint8_t speed_channel_1 = saturate_to_uint8((int16_t)motor_base_speed + (int16_t) unsaturated_motor_delta);
        
        motor_move(DIR_FORWARD, motor_base_speed, unsaturated_motor_delta);

        // motor_rotate_in_place(DIR_LEFT, saturate_to_uint8(controller.output));

        // ESP_LOGI(TAG, "x: %d  | y: %d", controller_sideways.pos->x, controller_sideways.pos->y);
        // ESP_LOGI(TAG, "controller output: %d | saturated uint8_t output: %d" , controller.output, saturate_to_uint8(controller.output));

        if (ticks_since_last_live_pos == SEND_LIVE_UPDATE_RATE)
        {
            send_live_update(&rover_pos, speed_channel_0, speed_channel_1);
            ticks_since_last_live_pos = 0;
        }

	}

}
*/