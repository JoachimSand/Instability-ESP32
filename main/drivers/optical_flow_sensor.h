#include "platform.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
// #include "portmacro.h"

// use the SPI2 controller for the optical flow sensor.
// note that there is only one other SPI
#define OPT_FLOW_HOST SPI2_HOST

#define READ    0x00
#define WRITE   0x80;

static const char TAG[] = "OptFlowSensor";

void transmit(spi_device_handle_t *spi_handle);

void init_opt_flow_sensor(u8 PIN_MISO, u8 PIN_MOSI, u8 PIN_CLK, u8 PIN_CS);
