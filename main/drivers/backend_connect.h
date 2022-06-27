#ifndef BACKEND_CONNECT_H
#define BACKEND_CONNECT_H

#include "platform.h"

#include "drivers/control.h"

#define LIVE_POS_STRING_MAX_LEN 300
#define PATH_STRING_MAX_LEN 300

#define MANUAL_OVERRIDE 1
#define AUTOMATIC_CONTROL 0

typedef struct tcp_task_data
{
	char *payload;
	u32 len;
} tcp_task_data_t;

void init_WIFI(void);

void send_debug_backend(const char *str, u32 len);

void send_live_update(rover_position_t *pos, uint8_t motor_speed_left, uint8_t motor_speed_right, uint8_t orientation, float ultrasonic_distance, uint16_t radar, uint8_t battery);

void send_obstacle_position(rover_position_t *pos);

void send_alien_position(rover_position_t *pos);

void send_path(rover_position_t *start_pos, rover_position_t *end_pos);

#endif
