#include "fpga_connect.h"
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

void init_fpga_connection(spi_device_handle_t *spi_handle)
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
	ESP_LOGI(FPGA_TAG, "Initializing bus SPI%d...", FPGA_HOST + 1);

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
	esp_err_t ret = spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_DISABLED);
	ESP_ERROR_CHECK(ret);

	// Configuration for the SPI device on the other side of the bus
	spi_device_interface_config_t devcfg = {
		.flags = SPI_DEVICE_HALFDUPLEX,
		.command_bits = 0,
		.address_bits = 0,
		.dummy_bits = 0,
		.clock_speed_hz = 25000000,
		.duty_cycle_pos = 128, // 50% duty cycle
		.mode = 3,
		.spics_io_num = PIN_CS,
		.cs_ena_pretrans = 1,
		.cs_ena_posttrans = 1, // Keep the CS low 3 cycles after transaction, to stop slave from missing the last bit when CS has less propagation delay than CLK
		.queue_size = 2,
	};

	ret = spi_bus_add_device(FPGA_HOST, &devcfg, spi_handle);
	ESP_ERROR_CHECK(ret);

	esp_err_t err = spi_device_acquire_bus(*spi_handle, portMAX_DELAY);
	if (err != ESP_OK)
	{
		ESP_LOGE(FPGA_TAG, "Failed to acquire SPI bus!!");
		return;
	}

	u32 count = 0;

	while (1)
	{

		spi_transaction_t t;

		// ets_delay_us(75);

		memset(&t, 0, sizeof(t));
		t.flags = SPI_TRANS_USE_RXDATA;
		t.length = 0;
		t.rxlength = 32;
		spi_device_polling_transmit(*spi_handle, &t);

		u32 x = t.rx_data[0] << 8 | t.rx_data[1];
		u32 y = t.rx_data[2] << 8 | t.rx_data[3];
		// ESP_LOGI(FPGA_TAG, "SPI DATA: (%u, %u)", x, y);
		//  u8 *read_to;
		//  *read_to = *(u8 *)t.rx_data;
		count++;
		if (count >= 307200)
		{
			vTaskDelay(1);
			count = 0;
			ESP_LOGI(FPGA_TAG, "SPI DATA: (%u, %u)", x, y);
		}
	}
	spi_device_release_bus(*spi_handle);
}
