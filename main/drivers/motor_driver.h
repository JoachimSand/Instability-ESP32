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

void init_gpio_motors(void);

void init_pwm_motors(void);

void init_motor_drivers(void);

void motor_move(uint8_t dir, uint8_t speed);

void motor_stop(void);

void motor_rotate_in_place(uint8_t dir, uint8_t speed);

#endif
