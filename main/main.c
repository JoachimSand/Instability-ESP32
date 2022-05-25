#include <stdio.h>

#include "driver/gpio.h"
#include "driver/ledc.h"

#include "freertos/portmacro.h"
#include "hal/gpio_types.h"
#include "hal/ledc_types.h"

// Include FreeRTOS for delay
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>


#include "platform.h"
#include "drivers/motor_driver.h"
#include "drivers/optical_flow_sensor.h"

void app_main(void)
{
    // init_motor_drivers();
    init_opt_flow_sensor(GPIO_NUM_12, GPIO_NUM_13, GPIO_NUM_14, GPIO_NUM_25);

    while(1)
    {
        // motor_move(DIR_FORWARD, 200);
        // vTaskDelay(5000 / portTICK_PERIOD_MS);
        // motor_stop();

        // motor_move(DIR_BACKWARD, 200);
        // vTaskDelay(5000 / portTICK_PERIOD_MS);
        // motor_stop();

        // motor_rotate_in_place(DIR_LEFT, 200);
        // vTaskDelay(5000 / portTICK_PERIOD_MS);
        // motor_stop();

        // motor_rotate_in_place(DIR_RIGHT, 200);
        // vTaskDelay(5000 / portTICK_PERIOD_MS);
        // motor_stop();

        // motor_stop();
        // vTaskDelay(5000 / portTICK_PERIOD_MS);
    }

}
