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

#include "esp_log.h"

#include "platform.h"
#include "drivers/motor_driver.h"
#include "drivers/optical_flow_sensor.h"

void app_main(void)
{
    // init_motor_drivers();

    spi_device_handle_t spi_handle;
    init_opt_flow_sensor(&spi_handle, GPIO_NUM_12, GPIO_NUM_13, GPIO_NUM_14, GPIO_NUM_25, GPIO_NUM_26);

    uint8_t delta_x;
    uint8_t delta_y;
    uint8_t squal;

    while(1)
    {
        optical_flow_sensor_read_write_byte(&spi_handle, READ, DELTA_X, &delta_x);
        vTaskDelay(1 / portTICK_PERIOD_MS);
        optical_flow_sensor_read_write_byte(&spi_handle, READ, DELTA_Y, &delta_y);
        vTaskDelay(1 / portTICK_PERIOD_MS);
        optical_flow_sensor_read_write_byte(&spi_handle, READ, SQUAL, &squal);

        ESP_LOGI(TAG, "dx: %u  | dy: %u  | squal: %u" , delta_x, delta_y, squal);
        vTaskDelay(1000/ portTICK_PERIOD_MS);

        // motor_move(DIR_FORWARD, 200);
        // vTaskDelay(5000 / portTICK_PERIOD_MS);
        // motor_stop();
    }

}
