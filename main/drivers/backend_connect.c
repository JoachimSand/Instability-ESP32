#include "platform.h"
#include "backend_connect.h"

#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event.h"
#include "netdb.h"
#include "sys/socket.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include <stdio.h>
#include "motor_driver.h"

static const char TCP_SERVER_ADDRESS[] = "192.168.0.10";
static const char TCP_SERVER_PORT[] = "12000";
// static const char TCP_SERVER_PORT[] = "19006";
static const char TAG_WIFI[] = "BackendConnection";
#define WIFI_SSID "AndroidApp"
#define WIFI_PASS "38970269"
#define WIFI_MAX_RETRY 5
// #define WIFI_MAX_RETRY 100

/*	Indicates that the file descriptor represents an invalid (uninitialized or closed) socket
	Used in the TCP server structure `sock[]` which holds list of active clients we serve.*/
#define INVALID_SOCK (-1)

/* 	Time in ms to yield to all tasks when a non-blocking socket would block
	Non-blocking socket operations are typically executed in a separate task validating
	the socket status. Whenever the socket returns `EAGAIN` (idle status, i.e. would block)
	we have to yield to all tasks to prevent lower priority tasks from starving. */
#define YIELD_TO_ALL_MS 50

/* The event group allows multiple bits for each event, but we only care about two events:
	- we are connected to the AP with an IP
	- we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

// TCP Server Definitions
#define PORT 80
#define KEEPALIVE_IDLE 10
#define KEEPALIVE_INTERVAL 10
#define KEEPALIVE_COUNT 50

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t s_wifi_event_group;
static i32 s_retry_num = 0;

static void log_socket_error(const char *tag, const int sock, const int err, const char *message)
{
	ESP_LOGE(tag, "[sock=%d]: %s\n"
				  "error=%d: %s",
			 sock, message, err, strerror(err));
}

static int socket_send(const char *tag, const int sock, const char *data, const size_t len)
{
	int to_write = len;
	while (to_write > 0)
	{
		int written = send(sock, data + (len - to_write), to_write, 0);
		if (written < 0 && errno != EINPROGRESS && errno != EAGAIN && errno != EWOULDBLOCK)
		{
			log_socket_error(tag, sock, errno, "Error occurred during sending");
			return -1;
		}
		to_write -= written;
	}
	return len;
}

static void tcp_client_task(void *pvParameters)
{

	tcp_task_data_t *data = pvParameters;
	static const char *TAG = "nonblocking-socket-client";

	ESP_LOGI(TAG, "Trying to send data %.*s", data->len, data->payload);

	static char rx_buffer[128];

	struct addrinfo hints = {.ai_socktype = SOCK_STREAM};
	struct addrinfo *address_info;
	int sock = INVALID_SOCK;

	int res = getaddrinfo(TCP_SERVER_ADDRESS, TCP_SERVER_PORT, &hints, &address_info);
	if (res != 0 || address_info == NULL)
	{
		ESP_LOGE(TAG, "couldn't get hostname for `%s` "
					  "getaddrinfo() returns %d, addrinfo=%p",
				 TCP_SERVER_ADDRESS, res, address_info);
		goto error;
	}

	// Creating client's socket
	sock = socket(address_info->ai_family, address_info->ai_socktype, address_info->ai_protocol);
	if (sock < 0)
	{
		log_socket_error(TAG, sock, errno, "Unable to create socket");
		goto error;
	}
	ESP_LOGI(TAG, "Socket created, connecting to %s:%s", TCP_SERVER_ADDRESS, TCP_SERVER_PORT);

	// Marking the socket as non-blocking
	int flags = fcntl(sock, F_GETFL);
	if (fcntl(sock, F_SETFL, flags | O_NONBLOCK) == -1)
	{
		log_socket_error(TAG, sock, errno, "Unable to set socket non blocking");
	}

	if (connect(sock, address_info->ai_addr, address_info->ai_addrlen) != 0)
	{
		if (errno == EINPROGRESS)
		{
			ESP_LOGD(TAG, "connection in progress");
			fd_set fdset;
			FD_ZERO(&fdset);
			FD_SET(sock, &fdset);

			// Connection in progress -> have to wait until the connecting socket is marked as writable, i.e. connection completes
			res = select(sock + 1, NULL, &fdset, NULL, NULL);
			if (res < 0)
			{
				log_socket_error(TAG, sock, errno, "Error during connection: select for socket to be writable");
				goto error;
			}
			else if (res == 0)
			{
				log_socket_error(TAG, sock, errno, "Connection timeout: select for socket to be writable");
				goto error;
			}
			else
			{
				int sockerr;
				socklen_t len = (socklen_t)sizeof(int);

				if (getsockopt(sock, SOL_SOCKET, SO_ERROR, (void *)(&sockerr), &len) < 0)
				{
					log_socket_error(TAG, sock, errno, "Error when getting socket error using getsockopt()");
					goto error;
				}
				if (sockerr)
				{
					log_socket_error(TAG, sock, sockerr, "Connection error");
					goto error;
				}
			}
		}
		else
		{
			log_socket_error(TAG, sock, errno, "Socket is unable to connect");
			goto error;
		}
	}

	ESP_LOGI(TAG, "Client sends data to the server...");
	int len = socket_send(TAG, sock, data->payload, data->len);
	if (len < 0)
	{
		ESP_LOGE(TAG, "Error occurred during socket_send");
		goto error;
	}
	ESP_LOGI(TAG, "Written: %.*s", len, data->payload);

	// Keep receiving until we have a reply
	/*
	do
	{
		len = try_receive(TAG, sock, rx_buffer, sizeof(rx_buffer));
		if (len < 0)
		{
			ESP_LOGE(TAG, "Error occurred during try_receive");
			goto error;
		}
		vTaskDelay(pdMS_TO_TICKS(YIELD_TO_ALL_MS));
	} while (len == 0);
	ESP_LOGI(TAG, "Received: %.*s", len, rx_buffer);
	*/

error:
	if (sock != INVALID_SOCK)
	{
		close(sock);
	}
	free(address_info);
	free(data->payload);
	data->payload = NULL;
	vTaskDelete(NULL);
}

static void server_recieve(const int sock)
{
	int len;
	char rx_buffer[128];

	do
	{
		len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
		if (len < 0)
		{
			ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
		}
		else if (len == 0)
		{
			ESP_LOGW(TAG, "Connection closed");
		}
		else
		{
			rx_buffer[len] = 0; // Null-terminate whatever is received and treat it like a string
			ESP_LOGI(TAG, "Received %d bytes: %s", len, rx_buffer);

			// TODO: Here we can add reactions to recieved messages.

			// Transmit back whatever the backend sent us
			// send() can return less bytes than supplied length.
			// Walk-around for robust implementation.
			switch (rx_buffer[0])
			{
			case 'F':
			{
				ESP_LOGI("MANUAL CONTROL", "Forward");
				motor_move(DIR_FORWARD, 150, 0);
			}
			break;

			case 'B':
			{
				ESP_LOGI("MANUAL CONTROL", "Backwards");
				motor_move(DIR_BACKWARD, 150, 0);
			}
			break;

			case 'R':
			{
				motor_rotate_in_place(DIR_RIGHT, 100);
				ESP_LOGI("MANUAL CONTROL", "Right");
			}
			break;

			case 'L':
			{
				motor_rotate_in_place(DIR_LEFT, 100);
				ESP_LOGI("MANUAL CONTROL", "Left");
			}
			break;

			case 'S':
			{
				motor_stop();
				ESP_LOGI("MANUAL CONTROL", "Stop");
			}
			break;
			}

			int to_write = len;
			while (to_write > 0)
			{
				int written = send(sock, rx_buffer + (len - to_write), to_write, 0);
				if (written < 0)
				{
					ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
				}
				to_write -= written;
			}
		}
	} while (len > 0);
}

static void tcp_server_task(void *pvParameters)
{
	char addr_str[128];
	int addr_family = (int)pvParameters;
	int ip_protocol = 0;
	int keepAlive = 1;
	int keepIdle = KEEPALIVE_IDLE;
	int keepInterval = KEEPALIVE_INTERVAL;
	int keepCount = KEEPALIVE_COUNT;
	struct sockaddr_storage dest_addr;

	if (addr_family == AF_INET)
	{
		struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
		dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
		dest_addr_ip4->sin_family = AF_INET;
		dest_addr_ip4->sin_port = htons(PORT);
		ip_protocol = IPPROTO_IP;
	}

	int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
	if (listen_sock < 0)
	{
		ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
		vTaskDelete(NULL);
		return;
	}
	int opt = 1;
	setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

	ESP_LOGI(TAG, "Socket created");

	int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
	if (err != 0)
	{
		ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
		ESP_LOGE(TAG, "IPPROTO: %d", addr_family);
		goto CLEAN_UP;
	}
	ESP_LOGI(TAG, "Socket bound, port %d", PORT);

	err = listen(listen_sock, 1);
	if (err != 0)
	{
		ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
		goto CLEAN_UP;
	}

	while (1)
	{

		ESP_LOGI(TAG, "Socket listening");

		struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
		socklen_t addr_len = sizeof(source_addr);
		int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
		if (sock < 0)
		{
			ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
			break;
		}

		// Set tcp keepalive option
		setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
		setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
		setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
		setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));
		// Convert ip address to string
		if (source_addr.ss_family == PF_INET)
		{
			inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
		}
		ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);

		server_recieve(sock);

		shutdown(sock, 0);
		close(sock);
	}

CLEAN_UP:
	close(listen_sock);
	vTaskDelete(NULL);
}

static void event_handler(void *arg, esp_event_base_t event_base,
						  int32_t event_id, void *event_data)
{
	if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
	{
		esp_wifi_connect();
	}
	else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
	{
		if (s_retry_num < WIFI_MAX_RETRY)
		{
			esp_wifi_connect();
			s_retry_num++;
			ESP_LOGI(TAG_WIFI, "retry to connect to the AP");
		}
		else
		{
			xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
		}
		ESP_LOGI(TAG_WIFI, "connect to the AP fail");
	}
	else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
	{
		ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
		ESP_LOGI(TAG_WIFI, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
		s_retry_num = 0;
		xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
	}
}

void init_WIFI(void)
{
	// Initialize NVS for saving the wifi configuration between boots
	esp_err_t ret = nvs_flash_init();

	s_wifi_event_group = xEventGroupCreate();

	ESP_ERROR_CHECK(esp_netif_init());

	ESP_ERROR_CHECK(esp_event_loop_create_default());
	esp_netif_create_default_wifi_sta();

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));

	esp_event_handler_instance_t instance_any_id;
	esp_event_handler_instance_t instance_got_ip;
	ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
														ESP_EVENT_ANY_ID,
														&event_handler,
														NULL,
														&instance_any_id));
	ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
														IP_EVENT_STA_GOT_IP,
														&event_handler,
														NULL,
														&instance_got_ip));

	wifi_config_t wifi_config = {
		.sta = {
			.ssid = WIFI_SSID,
			.password = WIFI_PASS,
			/* Setting a password implies station will connect to all security modes including WEP/WPA.
			 * However these modes are deprecated and not advisable to be used. Incase your Access point
			 * doesn't support WPA2, these mode can be enabled by commenting below line */
			.threshold.authmode = WIFI_AUTH_WPA2_PSK,
		},
	};

	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
	ESP_ERROR_CHECK(esp_wifi_start());

	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
	ESP_ERROR_CHECK(esp_wifi_start());

	/* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
	 * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
	EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
										   WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
										   pdFALSE,
										   pdFALSE,
										   portMAX_DELAY);

	/* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
	 * happened. */
	if (bits & WIFI_CONNECTED_BIT)
	{
		ESP_LOGI(TAG_WIFI, "connected to ap SSID:%s password:%s",
				 WIFI_SSID, WIFI_PASS);
	}
	else if (bits & WIFI_FAIL_BIT)
	{
		ESP_LOGI(TAG_WIFI, "Failed to connect to SSID:%s, password:%s",
				 WIFI_SSID, WIFI_PASS);
	}
	else
	{
		ESP_LOGE(TAG_WIFI, "UNEXPECTED EVENT");
	}

	/* The event will not be processed after unregister */
	ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
	ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
	vEventGroupDelete(s_wifi_event_group);

	xTaskCreate(tcp_server_task, "tcp_server", 4096, (void *)AF_INET, 5, NULL);
}

// Send debug info to be printed on the backend. str is memcpyed,
// so the callee can free the str immediately after calling this
// function.
void send_debug_backend(const char *str, u32 len)
{
	static tcp_task_data_t data = {0};
	// previous payload has not yet been delivered, don't send message.
	if (data.payload != NULL)
	{
		return;
	}

	data.payload = malloc(sizeof(char) * (len + 2));
	data.payload[0] = 'd';
	data.payload[1] = ' ';
	memcpy(&(data.payload[2]), str, len);
	data.len = len + 2;
	xTaskCreate(tcp_client_task, "tcp_client", 4096, &data, 5, NULL);
}

// TODO: consider null terminations

void send_live_update(rover_position_t *pos, uint8_t motor_speed_left, uint8_t motor_speed_right, float ultrasonic_distance, uint16_t radar)
{
	static tcp_task_data_t data = {0};

	// previous payload has not yet been delivered, don't send message.
	if (data.payload != NULL)
	{
		return;
	}

	data.payload = malloc(sizeof(char) * (LIVE_POS_STRING_MAX_LEN + 2));
	data.payload[0] = 'l';
	data.payload[1] = ' ';

	// TODO: send actual orientation
	int16_t orientation = 45;

	snprintf(&(data.payload[2]), LIVE_POS_STRING_MAX_LEN, "{\"position\": {\"x\": %d,\"y\": %d}} | {\"squal\": %u, \"motor_left\": %u, \"motor_right\": %u, \"orientation\": %d, \"ultrasonic\": %f, \"radar\": %d, \"battery\": %d}", pos->x, pos->y, pos->squal, motor_speed_left, motor_speed_right, orientation, ultrasonic_distance, radar, 0);
	// memcpy(&(data.payload[2]), str, len);
	data.len = strlen(data.payload);
	xTaskCreate(tcp_client_task, "tcp_client", 4096, &data, 5, NULL);
}

void send_alien_position(rover_position_t *pos)
{
	static tcp_task_data_t data = {0};
	// previous payload has not yet been delivered, don't send message.
	if (data.payload != NULL)
	{
		return;
	}
	data.payload = malloc(sizeof(char) * (LIVE_POS_STRING_MAX_LEN + 2));
	data.payload[0] = 'a';
	data.payload[1] = ' ';
	snprintf(&(data.payload[2]), LIVE_POS_STRING_MAX_LEN, "{\"position\": {\"x\": %d,\"y\": %d}} ", pos->x, pos->y);
	// memcpy(&(data.payload[2]), str, len);
	data.len = strlen(data.payload);
	xTaskCreate(tcp_client_task, "tcp_client", 4096, &data, 5, NULL);
}

void send_path(rover_position_t *start_pos, rover_position_t *end_pos)
{
	static tcp_task_data_t data = {0};
	// previous payload has not yet been delivered, don't send message.
	if (data.payload != NULL)
	{
		return;
	}
	data.payload = malloc(sizeof(char) * PATH_STRING_MAX_LEN);
	snprintf(data.payload, PATH_STRING_MAX_LEN, "{\"start\": {\"position\": {\"x\": %d,\"y\": %d}},\"end\": {\"position\": {\"x\": %d,\"y\": %d}}}", start_pos->x, start_pos->y, end_pos->x, end_pos->y);
	data.len = strlen(data.payload);
	xTaskCreate(tcp_client_task, "tcp_client", 4096, &data, 5, NULL);
}
