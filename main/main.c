#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/portmacro.h"
#include "hal/gpio_types.h"

// Include FreeRTOS for delay
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

void init_gpio(void)
{
    gpio_config_t config = {};
    config.pin_bit_mask = (1ULL << GPIO_NUM_4);
    config.mode = GPIO_MODE_OUTPUT; 
    config.pull_up_en = GPIO_PULLUP_DISABLE;
    config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    config.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&config);
}

void app_main(void)
{
    init_gpio();
    while(1)
    {
        gpio_set_level(GPIO_NUM_4, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(GPIO_NUM_4, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

}
