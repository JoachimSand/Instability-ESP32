#include "optical_flow_sensor.h"
#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include <string.h>
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/portmacro.h"

// Include FreeRTOS for delay
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "rom/ets_sys.h"

// #define NOVERFLOW

void optical_flow_sensor_read(spi_device_handle_t *spi_handle, optical_flow_data_t *data)
{
	uint8_t delta_x_reg;
	uint8_t delta_y_reg;
	uint8_t squal_reg;

	optical_flow_sensor_read_write_byte(spi_handle, READ, MOTION, &delta_x_reg);
	ets_delay_us(100);

#ifndef NOVERFLOW
	if (delta_x_reg & (1 << 4))
	{
		ESP_LOGI(TAG, "OVERFLOW occured in optical flow sensor read...");
	}
#endif

	optical_flow_sensor_read_write_byte(spi_handle, READ, DELTA_X, &delta_x_reg);
	data->delta_x = *(int8_t *)&delta_x_reg;
	ets_delay_us(100);

	optical_flow_sensor_read_write_byte(spi_handle, READ, DELTA_Y, &delta_y_reg);
	data->delta_y = *(int8_t *)&delta_y_reg;
	ets_delay_us(100);

	optical_flow_sensor_read_write_byte(spi_handle, READ, SQUAL, &squal_reg);
	data->squal = squal_reg;
}

void optical_flow_sensor_read_write_byte(spi_device_handle_t *spi_handle, u8 rw, u8 address, u8 *read_to)
{
	u8 addr = rw | (address & ~0x80);

	esp_err_t err = spi_device_acquire_bus(*spi_handle, portMAX_DELAY);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "Failed to acquire SPI bus!!");
		return;
	}
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));
	t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_CS_KEEP_ACTIVE;
	t.length = 8;
	t.rxlength = 0;
	t.tx_data[0] = addr;
	spi_device_polling_transmit(*spi_handle, &t);

	// vTaskDelay(10);
	ets_delay_us(75);

	memset(&t, 0, sizeof(t));
	t.flags = SPI_TRANS_USE_RXDATA;
	t.length = 0;
	t.rxlength = 8;
	spi_device_polling_transmit(*spi_handle, &t);

	*read_to = *(u8 *)t.rx_data;

	spi_device_release_bus(*spi_handle);
}

void init_opt_flow_sensor(spi_device_handle_t *spi_handle, u8 PIN_MISO, u8 PIN_MOSI, u8 PIN_CLK, u8 PIN_CS, u8 PIN_NRST)
{
	gpio_config_t config = {};
	config.pin_bit_mask = (1ULL << PIN_NRST);

	config.mode = GPIO_MODE_OUTPUT;
	config.pull_up_en = GPIO_PULLUP_DISABLE;
	config.pull_down_en = GPIO_PULLDOWN_DISABLE;
	config.intr_type = GPIO_INTR_DISABLE;
	gpio_config(&config);

	// Software reset of opitcal flow sensor
	gpio_set_level(PIN_NRST, 1);
	vTaskDelay(1 / portTICK_PERIOD_MS);
	gpio_set_level(PIN_NRST, 0);

	// ref
	ESP_LOGI(TAG, "Initializing bus SPI%d...", OPT_FLOW_HOST + 1);

	spi_bus_config_t buscfg = {
		.miso_io_num = PIN_MISO,
		.mosi_io_num = PIN_MOSI,
		.sclk_io_num = PIN_CLK,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.max_transfer_sz = 32,
	};

	// Initialize the SPI bus
	// esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
	esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_DISABLED);
	ESP_ERROR_CHECK(ret);

	// Configuration for the SPI device on the other side of the bus
	spi_device_interface_config_t devcfg = {
		.flags = SPI_DEVICE_HALFDUPLEX,
		.command_bits = 0,
		.address_bits = 0,
		.dummy_bits = 0,
		.clock_speed_hz = 1000000,
		.duty_cycle_pos = 128, // 50% duty cycle
		.mode = 3,
		.spics_io_num = PIN_CS,
		.cs_ena_pretrans = 1,
		.cs_ena_posttrans = 1, // Keep the CS low 3 cycles after transaction, to stop slave from missing the last bit when CS has less propagation delay than CLK
		.queue_size = 2,
	};

	ret = spi_bus_add_device(OPT_FLOW_HOST, &devcfg, spi_handle);
	ESP_ERROR_CHECK(ret);

	// Check WHOAMI
	u8 read_byte = 0xFF;
	optical_flow_sensor_read_write_byte(spi_handle, READ, 0x00, &read_byte);
	if (read_byte == 0x17)
	{
		ESP_LOGI(TAG, "Opitcal flow sensor init SUCCESS...");
	}
	else
	{
		ESP_LOGI(TAG, "Opitcal flow sensor init FAILED... (Got %u instead of 0x17...)", read_byte);
	}
}
