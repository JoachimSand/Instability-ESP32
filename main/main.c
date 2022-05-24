// #include <bits/stdint-uintn.h>
#include <stdio.h>
// #include <stdint.h>

#include "driver/gpio.h"
#include "driver/ledc.h"

#include "freertos/portmacro.h"
#include "hal/gpio_types.h"
#include "hal/ledc_types.h"

// Include FreeRTOS for delay
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define PWM_A   GPIO_NUM_32
#define AI1     GPIO_NUM_25
#define AI2     GPIO_NUM_33

#define PWM_B   GPIO_NUM_12
#define BI1     GPIO_NUM_27
#define BI2     GPIO_NUM_14

#define DIR_FORWARD 1
#define DIR_BACKWARD 0

#define DIR_LEFT 1
#define DIR_RIGHT 0

void init_gpio_motors(void)
{
    gpio_config_t config = {};
    config.pin_bit_mask = (1ULL << AI1) 
        | (1ULL << AI2) 
        | (1ULL << BI1) 
        | (1ULL << BI2);

    config.mode = GPIO_MODE_OUTPUT; 
    config.pull_up_en = GPIO_PULLUP_DISABLE;
    config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    config.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&config);
}

void init_pwm_motors(void)
{
    // Timer 0 config
    ledc_timer_config_t timer_config = {};
    
    timer_config.speed_mode = LEDC_LOW_SPEED_MODE;
    timer_config.duty_resolution = LEDC_TIMER_8_BIT;
    timer_config.timer_num = LEDC_TIMER_0;
    timer_config.freq_hz = 1000;
    timer_config.clk_cfg = LEDC_AUTO_CLK;

    ledc_timer_config(&timer_config);

    //Timer 1 config
    timer_config.timer_num = LEDC_TIMER_1;
    ledc_timer_config(&timer_config);

    // Channel A config
    ledc_channel_config_t channel_config = {};
    channel_config.gpio_num = PWM_A;
    channel_config.speed_mode = LEDC_LOW_SPEED_MODE;
    channel_config.channel = LEDC_CHANNEL_0;
    channel_config.intr_type = LEDC_INTR_DISABLE;
    channel_config.timer_sel = LEDC_TIMER_0;
    channel_config.duty = 100;

    ledc_channel_config(&channel_config);

    // Channel B config
    channel_config.gpio_num = PWM_B;
    channel_config.channel = LEDC_CHANNEL_1;
    channel_config.timer_sel = LEDC_TIMER_1;

    ledc_channel_config(&channel_config);
}

void init_motor_drivers(void)
{
    init_gpio_motors();
    init_pwm_motors();
}

void motor_move(uint8_t dir, uint8_t speed)
{
    gpio_set_level(AI1, dir);
    gpio_set_level(AI2, !dir);

    gpio_set_level(BI1, dir);
    gpio_set_level(BI2, !dir);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, speed);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, speed);
}

void motor_stop(void)
{
    gpio_set_level(AI1, 1);
    gpio_set_level(AI2, 1);

    gpio_set_level(BI1, 1);
    gpio_set_level(BI2, 1);

    vTaskDelay(50 / portTICK_PERIOD_MS);
}

void motor_rotate_in_place(uint8_t dir, uint8_t speed)
{
    gpio_set_level(AI1, dir);
    gpio_set_level(AI2, !dir);

    gpio_set_level(BI1, !dir);
    gpio_set_level(BI2, dir);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, speed);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, speed);
}

void app_main(void)
{
    init_motor_drivers();

    while(1)
    {
        motor_move(DIR_FORWARD, 200);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        motor_stop();

        motor_move(DIR_BACKWARD, 200);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        motor_stop();

        motor_rotate_in_place(DIR_LEFT, 200);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        motor_stop();

        motor_rotate_in_place(DIR_RIGHT, 200);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        motor_stop();

        motor_stop();
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }

}
