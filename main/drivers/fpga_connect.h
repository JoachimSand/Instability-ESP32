#ifndef FPGA_CONNECT_H
#define FPGA_CONNECT_H

#include "platform.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

// use the SPI2 controller for the optical flow sensor.
// note that there is only one other SPI
#define FPGA_HOST SPI3_HOST

static const char FPGA_TAG[] = "FpgaConnection";

#define PIN_MISO GPIO_NUM_19 // IO5
#define PIN_MOSI GPIO_NUM_23 // IO2
#define PIN_CLK GPIO_NUM_18	 // IO6
#define PIN_CS GPIO_NUM_5	 // IO7

#define PIN_NRST GPIO_NUM_15 // IO34 (not mapped)

void init_fpga_connection(spi_device_handle_t *spi_handle);

#endif