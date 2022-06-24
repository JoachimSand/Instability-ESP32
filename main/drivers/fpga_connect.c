#include "fpga_connect.h"
#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include <string.h>
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/portmacro.h"
#include "../platform.h"

// Include FreeRTOS for delay
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "rom/ets_sys.h"

// #define NOVERFLOW

void get_vision_data(spi_device_handle_t *spi_handle)
{
	esp_err_t err = spi_device_acquire_bus(*spi_handle, portMAX_DELAY);
	if (err != ESP_OK)
	{
		ESP_LOGE(FPGA_TAG, "Failed to acquire SPI bus!!");
		return;
	}

	u16 col_positions[32];
	u32 col_index = 0;
	u32 col_count = 0;

	u32 readings_count = 0;

	u8 ready_to_trans_count = 0;

	bb_collection_t bounding_box_collection = {0};

	while (ready_to_trans_count < 2)
	{
		spi_transaction_t t;
		memset(&t, 0, sizeof(t));
		t.flags = SPI_TRANS_USE_RXDATA;
		t.length = 0;
		t.rxlength = 32;

		spi_device_polling_transmit(*spi_handle, &t);

		// mask away state bits
		u32 left = (t.rx_data[0] << 8 | t.rx_data[1]) & 0x0FFF;
		u32 right = t.rx_data[2] << 8 | t.rx_data[3];
		u32 raw_data = (t.rx_data[0] << 24) | (t.rx_data[1] << 16) | (t.rx_data[2] << 8) | t.rx_data[3];

		spi_state_t state = (spi_state_t)(t.rx_data[0] >> 4);

		// ESP_LOGI(FPGA_TAG, "SPI STATE: %u \t%x", state, raw_data);

		switch (state)
		{
		case SPI_READY_TO_TRANS:
		{
			ready_to_trans_count++;
		}
		break;

		case SPI_TRANSMIT_COLS:
		{
			for (i32 i = 0; i < 28; i++)
			{
				if (raw_data & (0b1 << i))
				{
					col_positions[col_count] = col_index + i;
					col_count++;
				}
			}
			col_index += 28;
		}
		break;

		case SPI_RED_BB:
		{
			bounding_box_collection.red.left = left;
			bounding_box_collection.red.right = right;
		}
		break;

		case SPI_BLUE_BB:
		{
			bounding_box_collection.blue.left = left;
			bounding_box_collection.blue.right = right;
		}
		break;

		case SPI_PINK_BB:
		{
			bounding_box_collection.pink.left = left;
			bounding_box_collection.pink.right = right;
		}
		break;

		case SPI_YELLOW_BB:
		{
			bounding_box_collection.yellow.left = left;
			bounding_box_collection.yellow.right = right;
		}
		break;

		case SPI_TEAL_BB:
		{
			bounding_box_collection.teal.left = left;
			bounding_box_collection.teal.right = right;
		}
		break;

		case SPI_GREEN_BB:
		{
			bounding_box_collection.green.left = left;
			bounding_box_collection.green.right = right;
		}
		break;

		default:
		{
		}
		break;
		}

		// Intermittently perform vtaskdelay(1) to avoid blocking.
		readings_count++;
		if (readings_count % 1000)
		{

			vTaskDelay(1);
		}

		if (readings_count >= 10000)
		{
			ESP_LOGI(FPGA_TAG, "WARNING: Could not get vision data - FPGA not responding.");
			break;
		}
	}
	for (i32 i = 0; i < col_count; i++)
	{
		ESP_LOGI(FPGA_TAG, "Collumn at %u", col_positions[i]);
	}

	// ESP_LOGI(FPGA_TAG, "Red BB: (%u, %u)", bounding_box_collection.red.left, bounding_box_collection.red.right);
	// ESP_LOGI(FPGA_TAG, "blue BB: (%u, %u)", bounding_box_collection.blue.left, bounding_box_collection.blue.right);
	// ESP_LOGI(FPGA_TAG, "green BB: (%u, %u)", bounding_box_collection.green.left, bounding_box_collection.green.right);
	// ESP_LOGI(FPGA_TAG, "teal BB: (%u, %u)", bounding_box_collection.teal.left, bounding_box_collection.teal.right);
	// ESP_LOGI(FPGA_TAG, "pink BB: (%u, %u)", bounding_box_collection.pink.left, bounding_box_collection.pink.right);
	// ESP_LOGI(FPGA_TAG, "yellow BB: (%u, %u)", bounding_box_collection.yellow.left, bounding_box_collection.yellow.right);

	for (i32 i = 0; i < 6; i++)
	{
		i32 width = abs(bounding_box_collection.bb_list[i].left - bounding_box_collection.bb_list[i].right);

		if (width < 400 && width > 20)
		{
			ESP_LOGI(FPGA_TAG, "BB%i: (%u, %u),  W: %i", i, bounding_box_collection.bb_list[i].left, bounding_box_collection.bb_list[i].right, width);
		}
	}

	spi_device_release_bus(*spi_handle);
}

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

	spi_state_t state = SPI_IDLE;

	u16 col_positions[32];
	u32 col_index = 0;
	u32 col_count = 0;

	/*
	while (1)
	{

		spi_transaction_t t;

		// ets_delay_us(75);

		memset(&t, 0, sizeof(t));
		t.flags = SPI_TRANS_USE_RXDATA;
		t.length = 0;
		t.rxlength = 32;
		spi_device_polling_transmit(*spi_handle, &t);

		// mask away state bits
		u32 x = (t.rx_data[0] << 8 | t.rx_data[1]) & 0x0FFF;
		u32 y = t.rx_data[2] << 8 | t.rx_data[3];
		u32 raw_data = (t.rx_data[0] << 24) | (t.rx_data[1] << 16) | (t.rx_data[2] << 8) | t.rx_data[3];

		spi_state_t state_of_msg = (spi_state_t)(t.rx_data[0] >> 4);
		state = state_of_msg;

		ESP_LOGI(FPGA_TAG, "SPI STATE: %u \t%x", state_of_msg, raw_data);

		switch (state)
		{
		case SPI_READY_TO_TRANS:
		{
			// Reset columns
			col_count = 0;
			col_index = 0;
		}
		break;

		case SPI_TRANSMIT_COLS:
		{
			for (i32 i = 0; i < 28; i++)
			{
				if (raw_data & (0b1 << i))
				{
					col_positions[col_count] = col_index + i;
					col_count++;
				}
			}
			col_index += 28;

			if (col_index >= 640)
			{
				for (i32 i = 0; i < col_count; i++)
				{
					ESP_LOGI(FPGA_TAG, "Collumn at %u", col_positions[i]);
				}
				col_count = 0;
				col_index = 0;
			}
		}
		break;

		case SPI_RED_BB:
		{
			ESP_LOGI(FPGA_TAG, "Red BB: (%u, %u)", x, y);
		}
		break;

		case SPI_BLUE_BB:
		{
			ESP_LOGI(FPGA_TAG, "Blue BB: (%u, %u)", x, y);
		}
		break;

		case SPI_PINK_BB:
		{
			ESP_LOGI(FPGA_TAG, "Pink BB: (%u, %u)", x, y);
		}
		break;

		case SPI_YELLOW_BB:
		{
			ESP_LOGI(FPGA_TAG, "Pink BB: (%u, %u)", x, y);
		}
		break;

		case SPI_TEAL_BB:
		{
			ESP_LOGI(FPGA_TAG, "Teal BB: (%u, %u)", x, y);
		}
		break;

		case SPI_GREEN_BB:
		{
			ESP_LOGI(FPGA_TAG, "Green BB: (%u, %u)", x, y);
		}
		break;

		default:
		{
		}
		break;
		}

		//

		//   u8 *read_to;
		//   *read_to = *(u8 *)t.rx_data;
		count++;
		if (count >= 10000)
		{

			vTaskDelay(1);
			count = 0;
		}

		if (t.rx_data[0] != 0xCC)
		{
			// ESP_LOGI(FPGA_TAG, "NON IDLE: (%x)", raw_data);
		}
	}
	*/
	spi_device_release_bus(*spi_handle);
}
