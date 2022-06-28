#ifndef PATHFINDING_H
#define PATHFINDING_H

#include "sys/types.h"

#define GRID_SIZE_X 6
#define GRID_SIZE_Y 4

#define OBSTACLE_VAL 100

#define MAX_OPEN_NODES 250

#define MAX_PATH_LENGTH 30

#define POS_X 0
#define NEG_X 1
#define POS_Y 2
#define NEG_Y 3

#define PATH_FINISHED 0
#define MOTION_FORWARD 1
#define MOTION_ROTATE 2

#define NO_ROTATION 0
#define ROTATION_90_LEFT 1
#define ROTATION_90_RIGHT 2
#define ROTATION_180 3

#define GRID_NODE_SIZE 60

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

uint8_t get_path_length(grid_node_t path[]);

void get_path();

void add_obstacle(grid_node_t* obstacle_node);

uint8_t get_next_direction(grid_node_t* curr, grid_node_t* next);

uint8_t get_next_motion(grid_node_t* curr, grid_node_t* next, uint8_t curr_dir);

uint8_t get_rotation_type(uint8_t curr_dir, uint8_t next_dir);

uint8_t get_next_search_pattern_goal(grid_node_t* end);

#endif
