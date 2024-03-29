#ifndef FPGA_CONNECT_H
#define FPGA_CONNECT_H

#include "platform.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "control.h"
#include "pathfinding.h"
#include "backend_connect.h"

// use the SPI2 controller for the optical flow sensor.
// note that there is only one other SPI
#define FPGA_HOST SPI3_HOST

static const char FPGA_TAG[] = "FpgaConnection";

#define PIN_MISO GPIO_NUM_19 // IO5
#define PIN_MOSI GPIO_NUM_23 // IO2
#define PIN_CLK GPIO_NUM_18	 // IO6
#define PIN_CS GPIO_NUM_5	 // IO7

#define PIN_NRST GPIO_NUM_15 // IO34 (not mapped)

#define M_PI 3.14159265359
#define NO_TAPS_AVERAGING 5
#define MAX_COLS 32
#define MAX_COL_DISTANCES MAX_COLS - 1
#define MAX_COL_DERIVATES MAX_COLS - 2
#define MAX_COL_SECOND_DERIVATES MAX_COLS - 3
#define MAX_OBJ_COUNT 5

#define RED     "\"red\""
#define BLUE    "\"blue\""
#define PINK    "\"pink\""
#define YELLOW  "\"yellow\""
#define GREEN   "\"green\""
#define TEAL    "\"teal\""

#define VISION_ITERATIONS 10

#define OBSTACLE_ADD_THRESHOLD 5
#define ALIEN_ADD_THRESHOLD 6
#define ALIEN_COLORS 6

typedef enum spi_state
{
	SPI_IDLE,
	SPI_READY_TO_TRANS,
	SPI_TRANSMIT_COLS,
	SPI_RED_BB,
	SPI_BLUE_BB,
	SPI_PINK_BB,
	SPI_YELLOW_BB,
	SPI_GREEN_BB,
	SPI_TEAL_BB,
	SPI_TRANSMIT_PIXELS
} spi_state_t;

typedef struct bounding_box
{
	i32 left, right;
} bounding_box_t;

typedef struct alien_location
{
	// x is distance parallel to plane of camera, y is distance orthogonal to
	// plane of camera.
	f32 x, y;
} alien_location_t;

typedef struct alien_collection
{
	// Whether aliens have been located by the FPGA
	union
	{
		struct
		{
			u8 red_found, blue_found, pink_found, yellow_found, green_found, teal_found;
		};
		u8 found_list[6];
	};

	// Location of aliens
	union
	{
		struct
		{
			// red : 0, blue : 1, pink : 2, yellow : 3, green : 4, teal : 5,
			alien_location_t red, blue, pink, yellow, green, teal;
		};
		alien_location_t location_list[6];
	};
} alien_collection_t;

typedef struct bb_collection
{

	union
	{
		struct
		{
			// red : 0, blue : 1, pink : 2, yellow : 3, green : 4, teal : 5,
			bounding_box_t red, blue, pink, yellow, green, teal;
		};
		bounding_box_t bb_list[6];
	};

} bb_collection_t;

typedef struct obstacle_transform
{
	f32 x, y;
	f32 diameter;
} obstacle_transform_t;

typedef struct obstacle_collection
{
	obstacle_transform_t obstacle_transforms[MAX_OBJ_COUNT];
	u8 objects_found;
} obstacle_collection_t;

void init_fpga_connection(spi_device_handle_t *spi_handle);
void get_vision_data(spi_device_handle_t *spi_handle, alien_collection_t *aliens, obstacle_collection_t *obstacles);
void print_raw_vision_data(spi_device_handle_t *spi_handle);

void update_vision(spi_device_handle_t *spi_handle_fpga, obstacle_collection_t *current_obstacles, alien_collection_t *current_aliens, uint8_t curr_dir,
				   rover_position_t *rover_pos, uint16_t obstacle_count_map[][GRID_SIZE_Y], uint16_t alien_count_map[][GRID_SIZE_X][GRID_SIZE_Y],grid_node_t *current_rover_node, grid_node_t *end, grid_node_t current_path[], int32_t *current_path_index, uint8_t *need_to_init_controller);

#endif
