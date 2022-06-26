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
#include "esp_mac.h"
#include "rom/ets_sys.h"

#include "platform.h"
#include "drivers/motor_driver.h"
#include "drivers/optical_flow_sensor.h"
#include "drivers/fpga_connect.h"
#include "drivers/backend_connect.h"
#include "drivers/control.h"
#include "drivers/ultrasonic.h"
#include "drivers/pathfinding.h"

#define SEND_LIVE_UPDATE_RATE 100

void app_main(void)
{
	// WIFI stuffs
    // init_WIFI();
    uint8_t mac_addr_buffer[6] = {0};
    esp_read_mac(mac_addr_buffer, ESP_MAC_WIFI_STA);
    ESP_LOGI("WIFI MAC", "Wifi MAC address is: %2x:%2x:%2x:%2x:%2x:%2x", mac_addr_buffer[0], mac_addr_buffer[1], mac_addr_buffer[2], mac_addr_buffer[3], mac_addr_buffer[4], mac_addr_buffer[5]);

	spi_device_handle_t spi_handle;
	init_opt_flow_sensor(&spi_handle, GPIO_NUM_12, GPIO_NUM_13, GPIO_NUM_14, GPIO_NUM_25, GPIO_NUM_26);
	init_motor_drivers();
    // init_ultrasonic();

	motor_stop();

	rover_position_t rover_pos;
	rover_pos.x = 0;
	rover_pos.y = 0;
	rover_pos.squal = 0;

    controller_t controller_sideways;
    controller_t controller_forward;
    controller_t controller_rotate;
	// init_controller(2, 0, 0, 0.01, 0, AXIS_X, &rover_pos, &controller_sideways);
	// init_controller(1, 0, 0, 0.01, 4700, AXIS_Y, &rover_pos, &controller_forward);
    // init_controller(1, 0, 0, 0.01, 100 * FORWARD_CNT_PER_CM, AXIS_Y, &rover_pos, &controller_forward);

    // init_controller(1, 0, 0, 0.01, 90 * ROTATION_CNT_PER_DEGRES, AXIS_ROTATE, &rover_pos, &controller_rotate);

    grid_node_t current_path[MAX_PATH_LENGTH];
    
    grid_node_t current_rover_node;
    current_rover_node.x = 0;
    current_rover_node.y = 0;

    // TEST A* algo
    grid_node_t start;
    start.x = 0;
    start.y = 0;

    grid_node_t end;
    end.x = 20;
    end.y = 49;

    init_obstacle_map();
    grid_node_t new_obstacle;
    new_obstacle.x = 15;
    new_obstacle.y = 49;
    add_obstacle(&new_obstacle);
    // new_obstacle.y = 19;
    // add_obstacle(&new_obstacle);
    // new_obstacle.y = 21;
    // add_obstacle(&new_obstacle);

    uint8_t has_found_path = find_a_star_path(&start, &end);
    ESP_LOGI(TAG, "Has found path: %d", has_found_path);
    // get_path(&current_path);
    // TODO: fix get path function crash

    uint16_t ticks_since_last_live_pos = 0;

	while (1)
	{
        // UPDATE ROVER POSITION---------------------------------------------------------------------------------------
        update_rover_position(&spi_handle, &rover_pos);

        // PID MOTOR CONTROL FORWARD ---------------------------------------------------------------------------------------
        // update_controller(&spi_handle, &controller_forward);
        // update_controller(&spi_handle, &controller_sideways);

        // PID MOTOR CONTROL TURN
        // update_controller(&spi_handle, &controller_rotate);

        // uint8_t motor_base_speed = saturate_uint8_to_val(controller_forward.output, MAX_FORWARD_VEL);
        // int16_t unsaturated_motor_delta = controller_sideways.output;
//
        // uint8_t speed_channel_0 = saturate_to_uint8((int16_t)motor_base_speed - (int16_t) unsaturated_motor_delta);
        // uint8_t speed_channel_1 = saturate_to_uint8((int16_t)motor_base_speed + (int16_t) unsaturated_motor_delta);
//
        // motor_move(DIR_FORWARD, motor_base_speed, unsaturated_motor_delta);
        // motor_rotate_in_place(DIR_LEFT, saturate_to_uint8(controller_rotate.output));

        // LIVE UPDATE ---------------------------------------------------------------------------------------
        if (ticks_since_last_live_pos == SEND_LIVE_UPDATE_RATE)
        {
            ESP_LOGI(TAG, "x: %d, y: %d, squal: %d", rover_pos.x, rover_pos.y, rover_pos.squal);
            // send_live_update(&rover_pos, speed_channel_0, speed_channel_1, 0, 0);
            ticks_since_last_live_pos = 0;
        }

        ticks_since_last_live_pos++;
        vTaskDelay(10/ portTICK_PERIOD_MS);

	}

}
