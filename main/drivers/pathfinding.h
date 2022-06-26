#ifndef PATHFINDING_H
#define PATHFINDING_H

#include "sys/types.h"

#define GRID_SIZE_X 50
#define GRID_SIZE_Y 50

#define OBSTACLE_VAL 100

#define MAX_OPEN_NODES 250

#define MAX_PATH_LENGTH 30

#define POS_X 0
#define NEG_X 1
#define POS_Y 2
#define NEG_Y 3

#define MOTION_FORWARD 0
#define MOTION_ROTATE 1

#define ROTATION_90_LEFT 0
#define ROTATION_90_RIGHT 1
#define ROTATION_180 2

typedef struct grid_node
{
    int32_t x;
    int32_t y;
} grid_node_t;

void init_obstacle_map();

uint32_t manhattan_dist(grid_node_t* start, grid_node_t* end);

uint8_t find_a_star_path(grid_node_t* start, grid_node_t* end);

void clear_maps();

uint8_t is_open_nodes_empty();

grid_node_t find_min_fscore();

uint8_t is_in_open_set(grid_node_t* node);

void add_in_open_set(grid_node_t* node);

uint8_t is_node_obstacle(grid_node_t* node);

void update_neighbor(grid_node_t* neighbor, grid_node_t* current, grid_node_t* end);

void get_path();

void add_obstacle(grid_node_t* obstacle_node);

uint8_t get_next_motion(grid_node_t* curr, grid_node_t* next, uint8_t curr_dir);

#endif
