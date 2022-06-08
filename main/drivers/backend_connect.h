#ifndef BACKEND_CONNECT_H
#define BACKEND_CONNECT_H

#include "platform.h"

#include "drivers/control.h"

#define LIVE_POS_STRING_MAX_LEN 150
#define PATH_STRING_MAX_LEN 300

typedef struct tcp_task_data
{
	char *payload;
	u32 len;
} tcp_task_data_t;

void init_WIFI(void);

void send_debug_backend(const char *str, u32 len);

void send_live_position(rover_position_t* pos);

void send_alien_position(rover_position_t* pos);

void send_path(rover_position_t* start_pos, rover_position_t* end_pos);

#endif
