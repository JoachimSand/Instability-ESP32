#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdio.h>

#include "driver/gpio.h"
#include "driver/ledc.h"

#include "freertos/portmacro.h"
#include "hal/gpio_types.h"
#include "hal/ledc_types.h"

// Include FreeRTOS for delay
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define PWM_A   GPIO_NUM_5
#define AI1     GPIO_NUM_19
#define AI2     GPIO_NUM_18

#define PWM_B   GPIO_NUM_23
#define BI1     GPIO_NUM_21
#define BI2     GPIO_NUM_22

#define DIR_FORWARD 1
#define DIR_BACKWARD 0

#define DIR_LEFT 0
#define DIR_RIGHT 1

void init_gpio_motors(void);

void init_pwm_motors(void);

void init_motor_drivers(void);

void motor_move(uint8_t dir, uint8_t speed);

void motor_stop(void);

void motor_rotate_in_place(uint8_t dir, uint8_t speed);

#endif
