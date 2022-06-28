#include "fpga_connect.h"
#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include <string.h>
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/portmacro.h"
#include "../platform.h"
#include <math.h>

// Include FreeRTOS for delay
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "rom/ets_sys.h"


// #define NOVERFLOW

inline f32 pixel_pos_to_distance(i32 pixel_pos, f32 distance)
{
	f32 x_pixels = pixel_pos > 320 ? pixel_pos - 320 : -(320 - pixel_pos);
	f32 angle = (x_pixels / 320) * 35 * (M_PI / 180);
	f32 x = tanf(angle) * distance;
	return x;
}

void print_raw_vision_data(spi_device_handle_t *spi_handle)
{

	esp_err_t err = spi_device_acquire_bus(*spi_handle, portMAX_DELAY);
	if (err != ESP_OK)
	{
		ESP_LOGE(FPGA_TAG, "Failed to acquire SPI bus!");
		return;
	}

	i32 counter = 0;
	while (1)
	{
		spi_transaction_t t;
		memset(&t, 0, sizeof(t));
		t.flags = SPI_TRANS_USE_RXDATA;
		t.length = 0;
		t.rxlength = 32;

		spi_device_polling_transmit(*spi_handle, &t);
		u32 raw_data = (t.rx_data[0] << 24) | (t.rx_data[1] << 16) | (t.rx_data[2] << 8) | t.rx_data[3];
		spi_state_t state = (spi_state_t)(t.rx_data[0] >> 4);
		u32 x = (t.rx_data[0] << 8 | t.rx_data[1]) & 0x0FFF;
		u32 y = t.rx_data[2] << 8 | t.rx_data[3];

		if (state == SPI_READY_TO_TRANS)
		{
			ESP_LOGI(FPGA_TAG, "New frame, pixel count: %i", counter);
			counter = 0;
		}

		if (state == SPI_TRANSMIT_PIXELS)
		{
			counter++;
		}

		if (counter % 100000 == 0)
		{
			vTaskDelay(1);
			// ESP_LOGI(FPGA_TAG, "State: %i: Data: %x, X: %i, Y: %i, ", state, raw_data, x, y);
		}
	}
}

void get_vision_data(spi_device_handle_t *spi_handle, alien_collection_t *aliens, obstacle_collection_t *obstacles)
{
	esp_err_t err = spi_device_acquire_bus(*spi_handle, portMAX_DELAY);
	if (err != ESP_OK)
	{
		ESP_LOGE(FPGA_TAG, "Failed to acquire SPI bus!");
		return;
	}

	u16 col_positions[MAX_COLS];
	i32 col_index = 0;
	i32 col_count = 0;

	i32 readings_count = 0;

	u8 ready_to_trans_count = 0;

	// Static variables for unweighted averaging of alien bounding boxes
	static u8 head = 0;
	head++;
	head = head % 5;
	// ESP_LOGI(FPGA_TAG, "%i\n", head);
	static bb_collection_t bb_past_collections[NO_TAPS_AVERAGING] = {0};

	while (ready_to_trans_count < 2)
	{
		spi_transaction_t t;
		memset(&t, 0, sizeof(t));
		t.flags = SPI_TRANS_USE_RXDATA;
		t.length = 0;
		t.rxlength = 32;

		spi_device_polling_transmit(*spi_handle, &t);

		// mask away state bits
		u32 left = (t.rx_data[0] << 8 | t.rx_data[1]) & 0x0FFF;
		u32 right = t.rx_data[2] << 8 | t.rx_data[3];
		u32 raw_data = (t.rx_data[0] << 24) | (t.rx_data[1] << 16) | (t.rx_data[2] << 8) | t.rx_data[3];

		spi_state_t state = (spi_state_t)(t.rx_data[0] >> 4);

		// ESP_LOGI(FPGA_TAG, "SPI STATE: %u \t%x", state, raw_data);

		switch (state)
		{
		case SPI_READY_TO_TRANS:
		{
			ready_to_trans_count++;
			if (ready_to_trans_count < 2)
			{
				col_index = 0;
				col_count = 0;
			}
		}
		break;

		case SPI_TRANSMIT_COLS:
		{
			// ESP_LOGI(FPGA_TAG, "%x", raw_data);
			for (i32 i = 0; i < 28; i++)
			{
				if (raw_data & (0b1 << i) && col_count < MAX_COLS)
				{
					col_positions[col_count] = col_index + i;
					col_count++;
				}
			}
			col_index += 28;
		}
		break;

		case SPI_RED_BB:
		{
			bb_past_collections[head].red.left = left;
			bb_past_collections[head].red.right = right;
		}
		break;

		case SPI_BLUE_BB:
		{
			bb_past_collections[head].blue.left = left;
			bb_past_collections[head].blue.right = right;
		}
		break;

		case SPI_PINK_BB:
		{
			bb_past_collections[head].pink.left = left;
			bb_past_collections[head].pink.right = right;
		}
		break;

		case SPI_YELLOW_BB:
		{
			bb_past_collections[head].yellow.left = left;
			bb_past_collections[head].yellow.right = right;
		}
		break;

		case SPI_TEAL_BB:
		{
			bb_past_collections[head].teal.left = left;
			bb_past_collections[head].teal.right = right;
		}
		break;

		case SPI_GREEN_BB:
		{
			bb_past_collections[head].green.left = left;
			bb_past_collections[head].green.right = right;
		}
		break;

		default:
		{
		}
		break;
		}

		// Intermittently perform vtaskdelay(1) to avoid blocking.
		readings_count++;
		if (readings_count % 1000)
		{

			vTaskDelay(1);
		}

		if (readings_count >= 10000)
		{
			// ESP_LOGI(FPGA_TAG, "WARNING: Could not get vision data - FPGA not responding.");
			break;
		}
	}

	for (i32 i = 0; i < col_count; i++)
	{
		// ESP_LOGI(FPGA_TAG, "Collumn at %u", col_positions[i]);
	}

	// Compute distance between every column
	i32 col_distances[MAX_COL_DISTANCES];
	for (i32 j = 0; j < col_count - 1; j++)
	{
		col_distances[j] = col_positions[j + 1] - col_positions[j];
		// ESP_LOGI(FPGA_TAG, "ColDistance%i %i", j, col_distances[j]);
	}

	// Compute derivatives
	i32 col_derivates[MAX_COL_DERIVATES];
	for (i32 c = 0; c < col_count - 2; c++)
	{
		col_derivates[c] = col_distances[c + 1] - col_distances[c];
		// ESP_LOGI(FPGA_TAG, "ColDerivative%i %i", c, col_derivates[c]);
	}

	i32 prev_col_derivative = col_derivates[0];
	i32 gaps[MAX_OBJ_COUNT];
	i32 gap_count = 0;
	for (i32 c = 1; c < col_count - 2; c++)
	{
		if (gap_count >= MAX_OBJ_COUNT)
		{
			break;
		}
		if (prev_col_derivative > 50 && col_derivates[c] < -50)
		{
			// ESP_LOGI(FPGA_TAG, "Gap between cols (%i, %i)", c, c + 1);
			gaps[gap_count] = c;
			gap_count++;
		}
		prev_col_derivative = col_derivates[c];
	}

	i32 start_bound = 0;
	bounding_box_t obstacle_bbs[MAX_OBJ_COUNT];
	i32 obstacle_count = 0;

	// Seperate objects by looking at gaps
	for (i32 g = 0; g <= gap_count; g++)
	{
		if (obstacle_count >= MAX_OBJ_COUNT)
		{
			break;
		}
		i32 end_bound = gaps[g];

		if (g >= gap_count)
		{
			end_bound = col_count - 1;
		}

		// Only add the object if it contains more than one column
		if (end_bound - start_bound >= 2)
		{
			obstacle_bbs[obstacle_count].left = start_bound;
			obstacle_bbs[obstacle_count].right = end_bound;
			obstacle_count++;
			// ESP_LOGI(FPGA_TAG, "Obstacle between (%i %i)", start_bound, end_bound);
		}

		start_bound = gaps[g] + 1;
	}

	obstacles->objects_found = 0;
	// Determine distance to object
	for (i32 o = 0; o < obstacle_count; o++)
	{
		if (o > MAX_OBJ_COUNT)
		{
			break;
		}
		i32 avg_col_dist = 0;
		// ESP_LOGI(FPGA_TAG, "NEW OBJECT");
		for (i32 c = obstacle_bbs[o].left; c < obstacle_bbs[o].right; c++)
		{
			avg_col_dist += col_distances[c];
			// ESP_LOGI(FPGA_TAG, "Col distance: %i", col_distances[c]);
		}
		avg_col_dist /= obstacle_bbs[o].right - obstacle_bbs[o].left;
		// ESP_LOGI(FPGA_TAG, "Object avg col distance: %i", avg_col_dist);

		f32 distance = -3.45618 + 1333.54 / (f32)avg_col_dist;

		// ESP_LOGI(FPGA_TAG, "Avg col distance: %i, distance: %f", avg_col_dist, distance);
		obstacles->objects_found++;
		obstacles->obstacle_transforms[o].y = distance;

		i32 col_left = col_positions[obstacle_bbs[o].left];
		i32 col_right = col_positions[obstacle_bbs[o].right];
		i32 bb_centre = (col_left + col_right) / 2.0f;
		obstacles->obstacle_transforms[o].x = pixel_pos_to_distance(bb_centre, distance);
		obstacles->obstacle_transforms[o].diameter = pixel_pos_to_distance(col_right, distance) - pixel_pos_to_distance(col_left, distance);
		// ESP_LOGI(FPGA_TAG, "X: %f, Y: %f Diameter: %f\n", obstacles->obstacle_transforms[o].x, obstacles->obstacle_transforms[o].y, obstacles->obstacle_transforms[o].diameter);
	}

	bb_collection_t averaged_boxes = {0};
	// performed averaging
	for (i32 i = 0; i < 6; i++)
	{
		averaged_boxes.bb_list[i].left = 0;
		averaged_boxes.bb_list[i].right = 0;

		for (i32 j = 0; j < NO_TAPS_AVERAGING; j++)
		{
			averaged_boxes.bb_list[i].left += bb_past_collections[j].bb_list[i].left;
			averaged_boxes.bb_list[i].right += bb_past_collections[j].bb_list[i].right;
		}
		averaged_boxes.bb_list[i].left /= NO_TAPS_AVERAGING;
		averaged_boxes.bb_list[i].right /= NO_TAPS_AVERAGING;
	}

	// ESP_LOGI(FPGA_TAG, "Red BB: (%u, %u)", averaged_boxes.red.left, averaged_boxes.red.right);
	// ESP_LOGI(FPGA_TAG, "blue BB: (%u, %u)", averaged_boxes.blue.left, averaged_boxes.blue.right);
	// ESP_LOGI(FPGA_TAG, "pink BB: (%u, %u)", averaged_boxes.pink.left, averaged_boxes.pink.right);
	// ESP_LOGI(FPGA_TAG, "yellow BB: (%u, %u)", averaged_boxes.yellow.left, averaged_boxes.yellow.right);
	// ESP_LOGI(FPGA_TAG, "green BB: (%u, %u)", averaged_boxes.green.left, averaged_boxes.green.right);
	// ESP_LOGI(FPGA_TAG, "teal BB: (%u, %u)", averaged_boxes.teal.left, averaged_boxes.teal.right);

	for (i32 i = 0; i < 6; i++)
	{
		aliens->found_list[i] = 0;
		i32 width = abs(averaged_boxes.bb_list[i].left - averaged_boxes.bb_list[i].right);

		if (width > 320 || width < 20)
		{
			continue;
		}

		f32 distance = 18.1357 + 1167.879 / width;

		/*
		if (distance > 65 || distance < 20)
		{
			continue;
		}*/

		aliens->found_list[i] = 1;

		f32 alien_centre = (averaged_boxes.bb_list[i].left + averaged_boxes.bb_list[i].right) / 2;
		f32 x_pixels = alien_centre > 320 ? alien_centre - 320 : -(320 - alien_centre);
		f32 angle = (x_pixels / 320) * 35 * (M_PI / 180);
		f32 x = tanf(angle) * distance;

		aliens->location_list[i].x = x;
		aliens->location_list[i].y = distance;

		// ESP_LOGI(FPGA_TAG, "BB%i: (%u, %u),  W: %i A: %f, X: %f D: %f", i, averaged_boxes.bb_list[i].left, averaged_boxes.bb_list[i].right, width, angle, x, distance);
	}

	spi_device_release_bus(*spi_handle);
}

void init_fpga_connection(spi_device_handle_t *spi_handle)
{
	// ref
	ESP_LOGI(FPGA_TAG, "Initializing bus SPI%d...", FPGA_HOST + 1);

	spi_bus_config_t buscfg = {
		.miso_io_num = PIN_MISO,
		.mosi_io_num = PIN_MOSI,
		.sclk_io_num = PIN_CLK,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.max_transfer_sz = 32,
	};

	// Initialize the SPI bus
	// esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
	esp_err_t ret = spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_DISABLED);
	ESP_ERROR_CHECK(ret);

	// Configuration for the SPI device on the other side of the bus
	spi_device_interface_config_t devcfg = {
		.flags = SPI_DEVICE_HALFDUPLEX,
		.command_bits = 0,
		.address_bits = 0,
		.dummy_bits = 0,
		.clock_speed_hz = 25000000,
		.duty_cycle_pos = 128, // 50% duty cycle
		.mode = 3,
		.spics_io_num = PIN_CS,
		.cs_ena_pretrans = 1,
		.cs_ena_posttrans = 1, // Keep the CS low 3 cycles after transaction, to stop slave from missing the last bit when CS has less propagation delay than CLK
		.queue_size = 2,
	};

	ret = spi_bus_add_device(FPGA_HOST, &devcfg, spi_handle);
	ESP_ERROR_CHECK(ret);
}

void update_vision(spi_device_handle_t *spi_handle_fpga, obstacle_collection_t *current_obstacles, alien_collection_t *current_aliens, uint8_t curr_dir,
				   rover_position_t *rover_pos, uint16_t obstacle_count_map[][GRID_SIZE_Y], uint16_t alien_count_map[][GRID_SIZE_X][GRID_SIZE_Y],grid_node_t *current_rover_node, grid_node_t *end, grid_node_t current_path[], int32_t *current_path_index, uint8_t *need_to_init_controller)
{
	ESP_LOGI("VISION", "Starting update vision");
	for (int32_t i = 0; i < VISION_ITERATIONS; i++)
	{
		get_vision_data(spi_handle_fpga, current_aliens, current_obstacles);

        f32 rover_x_cm, rover_y_cm;
        rover_x_cm = rover_pos->x / (f32)FORWARD_CNT_PER_CM;
        rover_y_cm = rover_pos->y / (f32)FORWARD_CNT_PER_CM;
    
        // Process obstacles
		for (i32 i = 0; i < current_obstacles->objects_found; i++)
		{
			// Add each detected obstacle to the A star grid

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

			// ESP_LOGI("OBSTACLE", "Obstacle %i: (%f, %f), (%f, %f)", i, current_obstacles->obstacle_transforms[i].x, current_obstacles->obstacle_transforms[i].y, obstacle_x_cm, obstacle_y_cm);

			grid_node_t obstacle;

			obstacle.x = (obstacle_x_cm + (GRID_NODE_SIZE / 2.0f)) / GRID_NODE_SIZE;
			obstacle.y = (obstacle_y_cm + (GRID_NODE_SIZE / 2.0f)) / GRID_NODE_SIZE;

			obstacle_count_map[obstacle.x][obstacle.y]++;
			// ESP_LOGI("OBSTACLEGRID", "(%d, %d), %d", obstacle.x, obstacle.y, obstacle_count_map[obstacle.x][obstacle.y]);

			if (obstacle_count_map[obstacle.x][obstacle.y] == OBSTACLE_ADD_THRESHOLD)
			{
				ESP_LOGI("ObstacleDetected", "Obstacle added to grid at (%d, %d)", obstacle.x, obstacle.y);
                add_obstacle(&obstacle);
				find_a_star_path(current_rover_node, end);
				get_path(current_path);
				*current_path_index = 1;
				*need_to_init_controller = 1;

				rover_position_t obstacle_pos;
				obstacle_pos.x = obstacle_x_cm * FORWARD_CNT_PER_CM;
				obstacle_pos.y = obstacle_y_cm * FORWARD_CNT_PER_CM;
                send_obstacle_position(&obstacle_pos);
			}
		}
        
        // Process aliens
        for (int32_t i = 0; i < ALIEN_COLORS; i++)
        {
            if (current_aliens->found_list[i])
            {
                // ESP_LOGI("ALIEN", "num: %d, x: %f, y: %f: ", i, current_aliens->location_list[i].x, current_aliens->location_list[i].y );
                f32 alien_x_cm = 1, alien_y_cm = 1;

                if (curr_dir == POS_X)
                {
                    alien_x_cm = rover_x_cm + current_aliens->location_list[i].y;
                    alien_y_cm = rover_y_cm - current_aliens->location_list[i].x;
                }
                else if (curr_dir == NEG_X)
                {
                    alien_x_cm = rover_x_cm - current_aliens->location_list[i].y;
                    alien_y_cm = rover_y_cm + current_aliens->location_list[i].x;
                }
                else if (curr_dir == POS_Y)
                {
                    alien_x_cm = rover_x_cm - current_aliens->location_list[i].x;
                    alien_y_cm = rover_y_cm + current_aliens->location_list[i].y;
                }
                else if (curr_dir == NEG_Y)
                {
                    alien_x_cm = rover_x_cm + current_aliens->location_list[i].x;
                    alien_y_cm = rover_y_cm - current_aliens->location_list[i].y;
                }
                grid_node_t alien;

                alien.x = (alien_x_cm + (GRID_NODE_SIZE / 2.0f)) / GRID_NODE_SIZE;
                alien.y = (alien_y_cm + (GRID_NODE_SIZE / 2.0f)) / GRID_NODE_SIZE;
                // ESP_LOGI("ALIEN", "num: %d, x: %f, y: %f: ", i, alien_x_cm, alien_y_cm);
                // ESP_LOGI("ALIEN", "num: %d, x: %d, y: %d: ", i, alien.x, alien.y);

                alien_count_map[i][alien.x][alien.y]++;
                // ESP_LOGI("OBSTACLEGRID", "(%d, %d), %d", obstacle.x, obstacle.y, obstacle_count_map[obstacle.x][obstacle.y]);

                if (alien_count_map[i][alien.x][alien.y] == ALIEN_ADD_THRESHOLD)
                {
                    ESP_LOGI("AlienDetected", "Alien added to grid at (%d, %d)", alien.x, alien.y);
                    add_obstacle(&alien);
                    find_a_star_path(current_rover_node, end);
                    get_path(current_path);
                    *current_path_index = 1;
                    *need_to_init_controller = 1;

                    rover_position_t alien_pos;
                    alien_pos.x = alien_x_cm * FORWARD_CNT_PER_CM;
                    alien_pos.y = alien_y_cm * FORWARD_CNT_PER_CM;

                    // TODO: do this better
                    if (i == 0) send_alien_position(&alien_pos, RED);
                    if (i == 1) send_alien_position(&alien_pos, BLUE);
                    if (i == 2) send_alien_position(&alien_pos, PINK);
                    if (i == 3) send_alien_position(&alien_pos, YELLOW);
                    if (i == 4) send_alien_position(&alien_pos, GREEN);
                    if (i == 5) send_alien_position(&alien_pos, TEAL);
                }
            }
        }

	}
	ESP_LOGI("VISION", "Finished update vision");
}





