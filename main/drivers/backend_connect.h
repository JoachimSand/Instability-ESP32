#ifndef BACKEND_CONNECT_H
#define BACKEND_CONNECT_H

#include "platform.h"

typedef struct tcp_task_data
{
	char *payload;
	u32 len;
} tcp_task_data_t;

void init_WIFI(void);
void send_debug_backend(const char *str, u32 len);

#endif