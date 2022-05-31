#include "motor_driver.h"
#include "driver/ledc.h"
#include "hal/ledc_types.h"

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

void motor_move(uint8_t dir, uint8_t speed, int16_t delta)
{
    gpio_set_level(AI1, dir);
    gpio_set_level(AI2, !dir);

    gpio_set_level(BI1, dir);
    gpio_set_level(BI2, !dir);

    uint8_t speed_channel_0 = saturate_to_uint8((int16_t)speed - (int16_t) delta);
    uint8_t speed_channel_1 = saturate_to_uint8((int16_t)speed + (int16_t) delta);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, speed_channel_0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, speed_channel_1);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
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
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, speed);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}


uint8_t saturate_to_uint8(int16_t val)
{
    if (val > 255)  return 255;
    if (val < 0)    return 0;
    return (uint8_t) val;
}

uint8_t saturate_uint8_to_val(int16_t input, uint8_t limit)
{
    if (input > limit)  return limit;
    if (input < 0)    return 0;
    return (uint8_t) input;
}
