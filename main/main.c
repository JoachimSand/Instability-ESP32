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

#define GRID_NODE_SIZE 60

#define MAX_FORWARD_VEL 150
#define MAX_ROTATION_VEL 150
#define MAX_MOTOR_DELTA 15

void update_vision(spi_device_handle_t* spi_handle_fpga, obstacle_collection_t* current_obstacles, alien_collection_t* current_aliens, uint8_t curr_dir,
        rover_position_t* rover_pos, uint16_t obstacle_count_map[][GRID_SIZE_Y], grid_node_t* current_rover_node, grid_node_t* end, grid_node_t current_path[], int32_t* current_path_index, uint8_t* need_to_init_controller)
{
        get_vision_data(spi_handle_fpga, current_aliens, current_obstacles);

		for (i32 i = 0; i < current_obstacles->objects_found; i++)
		{
			// Add each detected obstacle to the A star grid
			f32 rover_x_cm, rover_y_cm;
			rover_x_cm = rover_pos->x / (f32)FORWARD_CNT_PER_CM;
			rover_y_cm = rover_pos->y / (f32)FORWARD_CNT_PER_CM;

			// ESP_LOGI("ObstacleCM", "(%f, %f) (%d, %d)", rover_x_cm, rover_y_cm, rover_po->.x, rover_pos.y);

			f32 obstacle_x_cm = 1, obstacle_y_cm = 1;

			if (curr_dir == POS_X)
			{
				obstacle_x_cm = rover_x_cm + current_obstacles->obstacle_transforms[i].y;
				obstacle_y_cm = rover_y_cm - current_obstacles->obstacle_transforms[i].x;
			}
			else if (curr_dir == NEG_X)
			{
				obstacle_x_cm = rover_x_cm - current_obstacles->obstacle_transforms[i].y;
				obstacle_y_cm = rover_y_cm + current_obstacles->obstacle_transforms[i].x;
			}
			else if (curr_dir == POS_Y)
			{
				obstacle_x_cm = rover_x_cm - current_obstacles->obstacle_transforms[i].x;
				obstacle_y_cm = rover_y_cm + current_obstacles->obstacle_transforms[i].y;
			}
			else if (curr_dir == NEG_Y)
			{
				obstacle_x_cm = rover_x_cm + current_obstacles->obstacle_transforms[i].x;
				obstacle_y_cm = rover_y_cm - current_obstacles->obstacle_transforms[i].y;
			}

			ESP_LOGI("OBSTACLE", "Obstacle %i: (%f, %f), (%f, %f)", i, current_obstacles->obstacle_transforms[i].x, current_obstacles->obstacle_transforms[i].y, obstacle_x_cm, obstacle_y_cm);

			grid_node_t obstacle;

			obstacle.x = (obstacle_x_cm + (GRID_NODE_SIZE / 2.0f)) / GRID_NODE_SIZE;
			obstacle.y = (obstacle_y_cm + (GRID_NODE_SIZE / 2.0f)) / GRID_NODE_SIZE;

			obstacle_count_map[obstacle.x][obstacle.y]++;
			ESP_LOGI("OBSTACLEGRID", "(%d, %d), %d", obstacle.x, obstacle.y, obstacle_count_map[obstacle.x][obstacle.y]);

			if (obstacle_count_map[obstacle.x][obstacle.y] == 5)
			{
				ESP_LOGI("ObstacleDetected", "Obstacle added to grid at (%d, %d)", obstacle.x, obstacle.y);
				add_obstacle(&obstacle);
				find_a_star_path(current_rover_node, end);
				get_path(&current_path);
				*current_path_index = 1;
                *need_to_init_controller = 1;
			}
		}
}

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

	spi_device_handle_t spi_handle_fpga;
	init_fpga_connection(&spi_handle_fpga);

	motor_stop();

	rover_position_t rover_pos;
	// rover_pos.x = 3 * GRID_NODE_SIZE * FORWARD_CNT_PER_CM;
	// rover_pos.y = 2 * GRID_NODE_SIZE * FORWARD_CNT_PER_CM;
	rover_pos.x = 0;
	rover_pos.y = 0;
	rover_pos.squal = 0;

	rover_position_t rover_angular_pos;
	rover_angular_pos.x = 0;
	rover_angular_pos.y = 0;
	rover_angular_pos.squal = 0;

	controller_t controller_sideways;
	controller_t controller_forward;
	controller_t controller_rotate;

	// Create path array and clear it
	grid_node_t current_path[MAX_PATH_LENGTH];
	for (int i = 0; i < MAX_PATH_LENGTH; i++)
	{
		current_path[i].x = -1;
		current_path[i].y = -1;
	}

	grid_node_t current_rover_node;
	current_rover_node.x = 0;
	current_rover_node.y = 0;

	// TEST A* algo
	grid_node_t start;
	start.x = 0;
	start.y = 0;

	grid_node_t end;
	end.x = 5;
	end.y = 0;

	init_obstacle_map();
	// grid_node_t new_obstacle;
	// new_obstacle.x = 3;
	// new_obstacle.y = 3;
	// add_obstacle(&new_obstacle);
	// new_obstacle.y = 19;
	// add_obstacle(&new_obstacle);
	// new_obstacle.y = 21;
	// add_obstacle(&new_obstacle);

	uint8_t has_found_path = find_a_star_path(&start, &end);
	ESP_LOGI(TAG, "Has found path: %d", has_found_path);
	get_path(&current_path);
	// TODO: fix get path function crash

	// ESP_LOGI("ASTAR", "Path length is %d", get_path_length(current_path));

	uint16_t ticks_since_last_live_pos = 0;

	int32_t current_path_index = 1;

	uint8_t curr_dir = POS_X;

	uint8_t need_to_init_controller = 1;

	u16 obstacle_count_map[GRID_SIZE_X][GRID_SIZE_Y];

	memset(&obstacle_count_map, 0, sizeof(u16) * GRID_SIZE_X * GRID_SIZE_Y);

	while (1)
	{
		// Get data from FPGA
		obstacle_collection_t current_obstacles = {0};
		alien_collection_t current_aliens = {0};

        update_vision(&spi_handle_fpga, &current_obstacles, &current_aliens, curr_dir, &rover_pos, obstacle_count_map, &current_rover_node, &end, current_path, &current_path_index, &need_to_init_controller);

				// update_rover_position(&spi_handle, &rover_pos);
		uint8_t next_motion = get_next_motion(&current_rover_node, &(current_path[current_path_index]), curr_dir);

        // ESP_LOGI("NEXT MOTION", "next motion: %d", next_motion);

		grid_node_t next_node = current_path[current_path_index];

		if (next_motion == PATH_FINISHED)
		{
			ESP_LOGI("PATH FOLLOWING", "Finished path...");
			while (1)
			{
			}
		}
		else if (next_motion == MOTION_FORWARD)
		{
			update_rover_position(&spi_handle, &rover_pos, curr_dir);
			if (need_to_init_controller)
			{
				ESP_LOGI("PATH FOLLOWING", "Moving forward one grid space...");
				if (curr_dir == POS_X || curr_dir == NEG_X)
				{
					ESP_LOGI("PATH FOLLOWING", "Current: x: %d, y: %d", rover_pos.x, rover_pos.y);
					ESP_LOGI("PATH FOLLOWING", "Target:  x: %d, y: %d", next_node.x * GRID_NODE_SIZE * FORWARD_CNT_PER_CM, next_node.y * GRID_NODE_SIZE * FORWARD_CNT_PER_CM);
					init_controller(2, 0, 0, 0.01, next_node.y * GRID_NODE_SIZE * FORWARD_CNT_PER_CM, AXIS_Y, &rover_pos, &controller_sideways);
					init_controller(1.3, 0, 0, 0.01, next_node.x * GRID_NODE_SIZE * FORWARD_CNT_PER_CM, AXIS_X, &rover_pos, &controller_forward);
					motor_move(DIR_FORWARD, 0, 0);
					// vTaskDelay(100 / portTICK_PERIOD_MS);
				}
				else // POS_Y or NEG_Y
				{
					ESP_LOGI("PATH FOLLOWING", "Current: x: %d, y: %d", rover_pos.x, rover_pos.y);
					ESP_LOGI("PATH FOLLOWING", "Target:  x: %d, y: %d", next_node.x * GRID_NODE_SIZE * FORWARD_CNT_PER_CM, next_node.y * GRID_NODE_SIZE * FORWARD_CNT_PER_CM);
					init_controller(2, 0, 0, 0.01, next_node.x * GRID_NODE_SIZE * FORWARD_CNT_PER_CM, AXIS_X, &rover_pos, &controller_sideways);
					init_controller(1.3, 0, 0, 0.01, next_node.y * GRID_NODE_SIZE * FORWARD_CNT_PER_CM, AXIS_Y, &rover_pos, &controller_forward);
					motor_move(DIR_FORWARD, 0, 0);
				}
				need_to_init_controller = 0;
			}
			update_controller(&spi_handle, &controller_sideways);
			uint8_t is_controller_finished = update_controller(&spi_handle, &controller_forward);

			if (is_controller_finished)
			{
				current_rover_node = current_path[current_path_index++];
				need_to_init_controller = 1;
				motor_stop();
			}
			else
			{
				if (curr_dir == POS_X || curr_dir == POS_Y)
				{
					uint8_t motor_base_speed = saturate_uint8_to_val(controller_forward.output, MAX_FORWARD_VEL);
					int16_t saturated_motor_delta = saturate_int16_to_val(controller_sideways.output, MAX_MOTOR_DELTA);

					ESP_LOGI("MOTOR CONTROL", "motor delta: %d", saturated_motor_delta);
					// uint8_t speed_channel_0 = saturate_to_uint8((int16_t)motor_base_speed - (int16_t) unsaturated_motor_delta);
					// uint8_t speed_channel_1 = saturate_to_uint8((int16_t)motor_base_speed + (int16_t) unsaturated_motor_delta);
					motor_move(DIR_FORWARD, motor_base_speed, saturated_motor_delta);
				}
				else
				{
					uint8_t motor_base_speed = saturate_uint8_to_val(-controller_forward.output, MAX_FORWARD_VEL);
					int16_t saturated_motor_delta = saturate_int16_to_val(-controller_sideways.output, MAX_MOTOR_DELTA);

					ESP_LOGI("MOTOR CONTROL", "motor delta: %d", saturated_motor_delta);
					// uint8_t speed_channel_0 = saturate_to_uint8((int16_t)motor_base_speed - (int16_t) unsaturated_motor_delta);
					// uint8_t speed_channel_1 = saturate_to_uint8((int16_t)motor_base_speed + (int16_t) unsaturated_motor_delta);
					motor_move(DIR_FORWARD, motor_base_speed, saturated_motor_delta);
				}
			}
		}
		else if (next_motion == MOTION_ROTATE) // MOTION ROTATE
		{
			update_rover_position(&spi_handle, &rover_angular_pos, POS_Y);
			uint8_t next_direction = get_next_direction(&current_rover_node, &(current_path[current_path_index]));
			uint8_t rotation_type = get_rotation_type(curr_dir, next_direction);

			if (need_to_init_controller)
			{
				if (rotation_type == ROTATION_90_LEFT)
				{
					// ESP_LOGI("PATH FOLLWING", "Rotating left...");
					init_controller(1.3, 0, 0, 0.01, 90 * ROTATION_CNT_PER_DEGRES, AXIS_ROTATE, &rover_angular_pos, &controller_rotate);
					motor_rotate_in_place(DIR_LEFT, 0);
					// vTaskDelay(100 / portTICK_PERIOD_MS);
				}
				if (rotation_type == ROTATION_90_RIGHT)
				{
					// ESP_LOGI("PATH FOLLWING", "Rotating right...");
					init_controller(1.3, 0, 0, 0.01, -90 * ROTATION_CNT_PER_DEGRES, AXIS_ROTATE, &rover_angular_pos, &controller_rotate);
					motor_rotate_in_place(DIR_RIGHT, 0);
					// vTaskDelay(100 / portTICK_PERIOD_MS);
				}
				if (rotation_type == ROTATION_180)
				{
					// ESP_LOGI("PATH FOLLWING", "Rotating 180...");
					init_controller(1.3, 0, 0, 0.01, 180 * ROTATION_CNT_PER_DEGRES, AXIS_ROTATE, &rover_angular_pos, &controller_rotate);
				}
				need_to_init_controller = 0;
			}
			update_controller(&spi_handle, &controller_rotate);
			uint8_t is_controller_finished = update_controller(&spi_handle, &controller_rotate);

			if (is_controller_finished)
			{
				motor_move(DIR_FORWARD, 0, 0); // this is to avoid jerk when starting to move forward again
				motor_stop();
				rover_angular_pos.x = 0;
				rover_angular_pos.y = 0;
				curr_dir = next_direction;
				need_to_init_controller = 1;
				vTaskDelay(1000 / portTICK_PERIOD_MS);
			}
			else
			{
				if (rotation_type == ROTATION_90_LEFT)
					motor_rotate_in_place(DIR_LEFT, saturate_uint8_to_val(controller_rotate.output, MAX_ROTATION_VEL));
				if (rotation_type == ROTATION_90_RIGHT)
					motor_rotate_in_place(DIR_RIGHT, saturate_uint8_to_val(-controller_rotate.output, MAX_ROTATION_VEL));
				if (rotation_type == ROTATION_180)
					motor_rotate_in_place(DIR_RIGHT, saturate_uint8_to_val(controller_rotate.output, MAX_ROTATION_VEL));
			}
		}

		ticks_since_last_live_pos++;
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
}
