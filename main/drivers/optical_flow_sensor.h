#include "platform.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
// #include "portmacro.h"

// use the SPI2 controller for the optical flow sensor.
// note that there is only one other SPI
#define OPT_FLOW_HOST SPI2_HOST

#define READ    0x00
#define WRITE   0x80

#define MOTION  0x02
#define DELTA_X 0x03
#define DELTA_Y 0x04
#define SQUAL   0x05

static const char TAG[] = "OptFlowSensor";

void transmit(spi_device_handle_t *spi_handle);

void init_opt_flow_sensor(spi_device_handle_t* spi_handle, u8 PIN_MISO, u8 PIN_MOSI, u8 PIN_CLK, u8 PIN_CS, u8 PIN_NRST);

void optical_flow_sensor_read_write_byte(spi_device_handle_t *spi_handle, u8 rw, u8 address, u8* read_to);
