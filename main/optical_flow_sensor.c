#include "optical_flow_sensor.h"

void transmit(spi_device_handle_t *spi_handle)
{
	esp_err_t err = spi_device_acquire_bus(spi_handle, portMAX_DELAY);
	spi_transaction_t t;
	memset(&t, sizeof(t), 0);
	t.length = 16;
	u8 data[] = {0, 0};
	t.tx_buffer = data;
	spi_device_polling_transmit(spi_handle, &t);
}

void init_opt_flow_sensor(u8 PIN_MISO, u8 PIN_MOSI, u8 PIN_CLK, u8 PIN_CS)
{
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
	esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
	ESP_ERROR_CHECK(ret);

	// Configuration for the SPI device on the other side of the bus
	spi_device_interface_config_t devcfg = {
		.flags = SPI_DEVICE_HALFDUPLEX,
		.command_bits = 0,
		.address_bits = 0,
		.dummy_bits = 0,
		.clock_speed_hz = 5000000,
		.duty_cycle_pos = 128, // 50% duty cycle
		.mode = 0,
		.spics_io_num = PIN_CS,
		.cs_ena_pretrans = 1,
		.cs_ena_posttrans = 1, // Keep the CS low 3 cycles after transaction, to stop slave from missing the last bit when CS has less propagation delay than CLK
		.queue_size = 3,
	};

	spi_device_handle_t spi_handle;
	ret = spi_bus_add_device(OPT_FLOW_HOST, &devcfg, &spi_handle);
	ESP_ERROR_CHECK(ret);
}