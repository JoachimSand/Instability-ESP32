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

void app_main(void)
{
	spi_device_handle_t spi_handle;
	alien_collection_t alien_collect;
	obstacle_collection_t obstacle_collect;
	init_fpga_connection(&spi_handle);
	while (1)
	{
		get_vision_data(&spi_handle, &alien_collect, &obstacle_collect);
	}
}
