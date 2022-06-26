#include "pathfinding.h"
#include "drivers/optical_flow_sensor.h"
#include "esp_log.h"
#include <math.h>

static int32_t obstacle_map[GRID_SIZE_X][GRID_SIZE_Y];

static volatile grid_node_t cameFrom[GRID_SIZE_X][GRID_SIZE_Y];
static int32_t gScore[GRID_SIZE_X][GRID_SIZE_Y];
static int32_t fScore[GRID_SIZE_X][GRID_SIZE_Y];
static grid_node_t open_nodes[MAX_OPEN_NODES];

static grid_node_t start_s;
static grid_node_t end_s;

inline uint32_t manhattan_dist(grid_node_t* start, grid_node_t* end)
{
    return abs(start->x - end->x) + abs(start->y - end->y);
}

void add_obstacle(grid_node_t* obstacle_node)
{
    obstacle_map[obstacle_node->x][obstacle_node->y] = 1;
}

uint8_t find_a_star_path(grid_node_t* start, grid_node_t* end)
{
    start_s = *start;
    end_s = *end;
    clear_maps();
    open_nodes[0].x = start->x;
    open_nodes[0].y = start->y;
    
    gScore[start->x][start->y] = 0;
    fScore[start->x][start->y] = manhattan_dist(start, end);
    // ESP_LOGI("ASTAR", "start distance: %d ", manhattan_dist(start,end));

    while (!is_open_nodes_empty())
    {
        grid_node_t curr_node = find_min_fscore();
        ESP_LOGI("ASTAR", "current node -> x: %d, y: %d ", curr_node.x, curr_node.y);
        if (curr_node.x == end->x && curr_node.y == end->y)  return 1;
    

        // Neighbor left 
        grid_node_t left = curr_node;
        left.x--;
        if (!is_node_obstacle(&left)) update_neighbor(&left, &curr_node, end);

        // Neighbor right 
        grid_node_t right = curr_node;
        right.x++;
        if (!is_node_obstacle(&right)) update_neighbor(&right, &curr_node, end);

        // Neighbor up 
        grid_node_t up = curr_node;
        up.y++;
        if (!is_node_obstacle(&up)) update_neighbor(&up, &curr_node, end);


        // Neighbor down 
        grid_node_t down = curr_node;
        down.y--;
        if (!is_node_obstacle(&down)) update_neighbor(&down, &curr_node, end);
    }

    return 0;
}

uint8_t is_node_obstacle(grid_node_t* node)
{
    if (node->x < 0 || node->y < 0) return 1;
    if (node->x >= GRID_SIZE_X || node->y >= GRID_SIZE_Y) return 1;

    if (obstacle_map[node->x][node->y] == 1) return 1;
    return 0;
}

void update_neighbor(grid_node_t* neighbor, grid_node_t* current, grid_node_t* end)
{
    int32_t current_gscore = gScore[current->x][current->y];
    int32_t tentative_gscore = current_gscore + 1;

    if (tentative_gscore < gScore[neighbor->x][neighbor->y])
    {
        cameFrom[neighbor->x][neighbor->y] = *current;
        gScore[neighbor->x][neighbor->y] = tentative_gscore;
        fScore[neighbor->x][neighbor->y] = tentative_gscore + manhattan_dist(neighbor, end);

        if (!is_in_open_set(neighbor))
        {
            add_in_open_set(neighbor);
        }
    }
     
}

void clear_maps()
{
    for (int32_t i = 0; i < GRID_SIZE_X; i++)
    {
        for (int32_t j = 0; j < GRID_SIZE_Y; j++)
        {
            cameFrom[i][j].x = -1;
            cameFrom[i][j].y = -1;

            gScore[i][j] = INT32_MAX;
            fScore[i][j] = INT32_MAX;
        }
    }

    for (int32_t i = 0; i < MAX_OPEN_NODES; i++)
    {
        open_nodes[i].x = -1;
        open_nodes[i].y = -1;
    }
}

uint8_t is_open_nodes_empty()
{
    for (int32_t i = 0; i < MAX_OPEN_NODES; i++)
    {
        if (open_nodes[i].x != -1 && open_nodes[i].y != -1) return 0;
    }
    return 1;
}

grid_node_t find_min_fscore()
{
    grid_node_t min_node;
    int32_t min_fscore = INT32_MAX;
    int32_t min_index = -1;

    for (int32_t i = 0; i < MAX_OPEN_NODES; i++)
    {
        // ESP_LOGI("ASTAR", "open node: x: %d, y:%d", open_nodes[i].x, open_nodes[i].y);
        grid_node_t curr_node = open_nodes[i];
        if (curr_node.x != -1 && curr_node.y != -1)
        {
            int32_t curr_fscore = fScore[curr_node.x][curr_node.y];
            // ESP_LOGI("ASTAR", "open node fscore: %d", curr_fscore);
            if (curr_fscore <= min_fscore)
            {
                min_fscore = curr_fscore;
                min_node = curr_node;
                min_index = i;
            }
        }
    }
    open_nodes[min_index].x = -1;
    open_nodes[min_index].y = -1;

    return min_node;
}

uint8_t is_in_open_set(grid_node_t* node)
{
    for (int32_t i = 0; i < MAX_OPEN_NODES; i++)
    {
        if (open_nodes[i].x == node->x && open_nodes[i].y == node->y) return 1;
    }
    return 0;
}

void add_in_open_set(grid_node_t* node)
{
    for (int32_t i = 0; i < MAX_OPEN_NODES; i++)
    {
        if (open_nodes[i].x == -1 && open_nodes[i].y == -1)
        {
            open_nodes[i].x = node->x;
            open_nodes[i].y = node->y;
            return;
        }
    }
}


void init_obstacle_map()
{
    for (int32_t i = 0; i < GRID_SIZE_X; i++)
    {
        for (int32_t j = 0; j < GRID_SIZE_Y; j++)
        {
            obstacle_map[i][j] = 0;
        }
    }
}

void get_path(grid_node_t path[])
{
    grid_node_t curr = end_s;
    int32_t index = MAX_PATH_LENGTH - 1;
    while (curr.x != start_s.x || curr.y != start_s.y)
    {
        path[index] = curr; 
        // ESP_LOGI("ASTAR", "x: %d, y: %d", curr.x, curr.y);
        curr = cameFrom[curr.x][curr.y];
        index--;
    }
    path[index] = curr; 
    // ESP_LOGI("ASTAR", "x: %d, y: %d", curr.x, curr.y);
    // ESP_LOGI("ASTAR", "Done with path printing");
}

uint8_t get_next_direction(grid_node_t* curr, grid_node_t* next)
{
    if (curr->x == next->x)
    {
        if (curr->y < next->y)  return POS_Y;    
        else                    return NEG_Y;
    }
    else // curr->y == next->y
    {
        if (curr->x < next->x)  return POS_X;
        else                    return NEG_X;
    }
}

uint8_t get_next_motion(grid_node_t* curr, grid_node_t* next, uint8_t curr_dir)
{
   uint8_t next_direction = get_next_direction(curr, next); 

   if (next_direction == curr_dir) return MOTION_FORWARD;

   return MOTION_ROTATE;
}

uint8_t get_rotation_type(uint8_t curr_dir, uint8_t next_dir)
{
    if ( curr_dir + next_dir == 1 || curr_dir + next_dir == 5) return ROTATION_180; 
    if ( )
    return 0;
}
