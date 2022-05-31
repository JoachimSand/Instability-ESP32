#include <stdio.h>

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
#include "rom/ets_sys.h"

#include "platform.h"
#include "drivers/motor_driver.h"
#include "drivers/optical_flow_sensor.h"
#include "drivers/backend_connect.h"

void app_main(void)
{
	// init_motor_drivers();
	init_WIFI();
	send_debug(NULL, 0);

	spi_device_handle_t spi_handle;
	init_opt_flow_sensor(&spi_handle, GPIO_NUM_12, GPIO_NUM_13, GPIO_NUM_14, GPIO_NUM_25, GPIO_NUM_26);
	init_motor_drivers();

	optical_flow_data_t op_flow_data;

	// motor_move(DIR_BACKWARD, 255);
	// motor_rotate_in_place(DIR_RIGHT,  255);
	motor_stop();

	while (1)
	{
		optical_flow_sensor_read(&spi_handle, &op_flow_data);

		ESP_LOGI(TAG, "dx: %d  | dy: %d  | squal: %u", op_flow_data.delta_x, op_flow_data.delta_y, op_flow_data.squal);
		vTaskDelay(100 / portTICK_PERIOD_MS);

		// motor_move(DIR_FORWARD, 200);
		// vTaskDelay(5000 / portTICK_PERIOD_MS);
		// motor_stop();
	}
}
