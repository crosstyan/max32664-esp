//
// Created by Kurosu Chan on 2024/1/18.
//

#include <cstdint>
#include <driver/spi_master.h>
#include <esp_log.h>
#include "esp_err.h"
#include "hal_error.hpp"
#include "hal_gpio.hpp"
#include "hal_spi.hpp"
#include "utils/utils.hpp"
#include "utils/instant.hpp"
// https://docs.espressif.com/projects/esp-idf/en/v5.3.2/esp32h2/api-reference/peripherals/spi_master.html
// https://github.com/espressif/esp-idf/blob/v5.3.2/examples/peripherals/spi_master/lcd/main/spi_master_example_main.c
namespace hal::spi {
constexpr auto SPI_ADDR_WIDTH = 16;
constexpr auto TAG            = "spi";
namespace details {
	static bool is_initialized = false;
	static spi_device_handle_t spi_device;
}

void init() {
	if (details::is_initialized) {
		return;
	}

	// Configure SPI bus
	spi_bus_config_t bus_config = {
		.mosi_io_num     = MOSI_PIN,
		.miso_io_num     = MISO_PIN,
		.sclk_io_num     = SCLK_PIN,
		.quadwp_io_num   = -1,
		.quadhd_io_num   = -1,
		.data4_io_num    = -1,
		.data5_io_num    = -1,
		.data6_io_num    = -1,
		.data7_io_num    = -1,
		.max_transfer_sz = 0,
		.flags           = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_GPIO_PINS,
		.intr_flags      = 0,
	};

	// Configure SPI device
	// https://github.com/nopnop2002/esp-idf-sx126x/blob/main/components/ra01s/ra01s.c
	//
	// SPI_DEVICE_HALFDUPLEX flag will turn the device into half-duplex
	// See also the phase
	// https://docs.espressif.com/projects/esp-idf/en/v5.3.2/esp32/api-reference/peripherals/spi_master.html#spi-transactions
	spi_device_interface_config_t dev_config = {
		.command_bits   = 0,
		.address_bits   = 0,
		.dummy_bits     = 0,
		.mode           = 0,
		.duty_cycle_pos = 128,
		.clock_speed_hz = 10'000'000,
		.spics_io_num   = CS_PIN,
		.flags          = SPI_DEVICE_NO_DUMMY,
		.queue_size     = 1,
	};

	ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO));
	ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev_config, &details::spi_device));

	gpio_config_t io_conf = {
		.pin_bit_mask = (1ULL << BUSY_PIN),
		.mode         = GPIO_MODE_INPUT,
		.pull_up_en   = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type    = GPIO_INTR_DISABLE,
	};
	ESP_ERROR_CHECK(gpio_config(&io_conf));
	details::is_initialized = true;
}

/*!
	\brief SPI transfer method single byte.
	\note Unimplemented for ESP32, since the ESP_IDF has provided the `spi_transaction_t` and `spi_transaction_ext_t` types.
*/
static uint8_t hal_transfer(const uint8_t data) {
	std::unreachable();
}

static void cs_low() {
	hal::gpio::digital_write(CS_PIN, false);
}

static void cs_high() {
	hal::gpio::digital_write(CS_PIN, true);
}

/*!
  \brief SPI single transfer method for modules with stream-type SPI interface (SX126x, SX128x etc.).
  \param cmd SPI operation command.
  \param cmd_size SPI command length in bytes.
  \param write Set to true for write commands, false for read commands.
  \param out Data that will be transfered from master to slave.
  \param in Data that was transfered from slave to master.
  \param size Number of bytes to transfer.
  \param wait Whether to wait for some GPIO at the end of transfer (e.g. BUSY line on SX126x/SX128x).
  \param timeout_ms GPIO wait period timeout in milliseconds.
  \note Unimplemented, only useful for modules with register-type SPI interface (SX127x).
*/
void transfer(const uint8_t cmd, const uint16_t reg, const uint8_t *out, uint8_t *in, const uint8_t size) {
	std::unreachable();
};

// https://docs.espressif.com/projects/esp-idf/en/release-v5.3/esp32/api-reference/peripherals/spi_master.html
// https://github.com/espressif/arduino-esp32/blob/5ba4c21a990f46b61ae8c7913b81e15064b2a8ef/cores/esp32/esp32-hal-spi.c#L836-L843
// https://github.com/espressif/esp-idf/issues/5223

/*!
	\brief wait for busy pin to go down
	\return true if the pin goes down, return false if timeout
*/
bool wait_for_not_busy(const size_t timeout_ms) {
	if constexpr (BUSY_PIN == GPIO_NUM_NC) {
		utils::delay_ms(timeout_ms);
		return true;
	} else {
		const auto io_inst = utils::Instant<>{};
		while (hal::gpio::digital_read(BUSY_PIN) == hal::gpio::HIGH) {
			if (io_inst.elapsed_ms() > timeout_ms) {
				return false;
			}
		}
		return true;
	}
}

Result<uint8_t, error_t>
write_stream(const uint8_t *cmd, const uint8_t cmd_size, const uint8_t *out, const uint8_t size, const size_t timeout_ms) {
	if (not wait_for_not_busy(timeout_ms)) {
		return ue_t{error::TIMEOUT};
	}

	spi_transaction_ext_t transaction;
	uint8_t status = 0;
	// SPI_TRANS_CS_KEEP_ACTIVE
	if (cmd_size == 1) {
		transaction = spi_transaction_ext_t{
			.base = {
				.flags     = SPI_TRANS_VARIABLE_CMD,
				.cmd       = cmd[0],
				.length    = static_cast<size_t>(size * 8),
				.rxlength  = 1 * 8,
				.tx_buffer = out,
				.rx_buffer = &status,
			},
			.command_bits = static_cast<uint8_t>(1 * 8),
			.address_bits = 0,
			.dummy_bits   = 0,
		};
	} else if (cmd_size == 2) {
		transaction = spi_transaction_ext_t{
			.base = {
				.flags     = SPI_TRANS_VARIABLE_CMD,
				.cmd       = static_cast<uint16_t>(cmd[1] << 8 | cmd[0]),
				.length    = static_cast<size_t>(size * 8),
				.tx_buffer = out,
			},
			.command_bits = static_cast<uint8_t>(2 * 8),
			.address_bits = 0,
			.dummy_bits   = 0,
		};
	} else {
		std::unreachable();
	}
	esp_err_t err = spi_device_polling_transmit(details::spi_device, &transaction.base);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "failed to transfer %s (%d)", esp_err_to_name(err), err);
		return ue_t{error::FAILED};
	}
	return status;
}

// the first byte in status should be treated as a status
Result<Unit, error_t>
read_stream_with_status(const uint8_t *cmd, const uint8_t cmd_size, uint8_t *in, const uint8_t size, const size_t timeout_ms) {
	if (not wait_for_not_busy(timeout_ms)) {
		return ue_t{error::TIMEOUT};
	}

	spi_transaction_ext_t transaction;
	if (cmd_size == 1) {
		transaction = spi_transaction_ext_t{
			.base = {
				.flags     = SPI_TRANS_VARIABLE_CMD,
				.cmd       = cmd[0],
				.length    = static_cast<size_t>(size * 8),
				.rxlength  = static_cast<size_t>(size * 8),
				.rx_buffer = in,
			},
			.command_bits = static_cast<uint8_t>(1 * 8),
			.address_bits = 0,
			.dummy_bits   = 0,
		};
	} else if (cmd_size == 2) {
		transaction = spi_transaction_ext_t{
			.base = {
				.flags     = SPI_TRANS_VARIABLE_CMD,
				.cmd       = static_cast<uint16_t>(cmd[1] << 8 | cmd[0]),
				.length    = static_cast<size_t>(size * 8),
				.rx_buffer = in,
			},
			.command_bits = static_cast<uint8_t>(2 * 8),
			.address_bits = 0,
			.dummy_bits   = 0,
		};
	} else {
		std::unreachable();
	}
	esp_err_t err = spi_device_polling_transmit(details::spi_device, &transaction.base);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "failed to transfer %s (%d)", esp_err_to_name(err), err);
		return ue_t{error::FAILED};
	}
	return {};
}


Result<Unit, error_t> read_register_burst_with_status(uint16_t reg, uint8_t *buffer, uint8_t size) {
	spi_transaction_ext_t transaction;

	transaction = spi_transaction_ext_t{
		.base = {
			.flags     = SPI_TRANS_VARIABLE_CMD,
			.cmd       = SPI_READ_COMMAND,
			.addr      = reg,
			.length    = static_cast<size_t>(size * 8),
			.rx_buffer = buffer,
		},
		.command_bits = static_cast<uint8_t>(1 * 8),
		.address_bits = static_cast<uint8_t>(2 * 8),
		.dummy_bits   = 0,
	};
	esp_err_t err = spi_device_polling_transmit(details::spi_device, &transaction.base);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "failed to transfer %s (%d)", esp_err_to_name(err), err);
		return ue_t{error::FAILED};
	}
	return {};
};

// https://github.com/espressif/esp-idf/blob/master/components/hal/esp32s3/include/hal/spi_ll.h
Result<uint8_t, error_t> write_register_burst(uint16_t reg, const uint8_t *buffer, uint8_t size) {
	spi_transaction_ext_t transaction;
	uint8_t status = 0;

	// Configure the transaction with:
	// - Command: SPI_WRITE_COMMAND (1 byte)
	// - Address: reg (2 bytes)
	// - Data: buffer (size bytes)
	//
	transaction = spi_transaction_ext_t{
		.base = {
			.flags     = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR,
			.cmd       = SPI_WRITE_COMMAND,
			.addr      = reg,
			.length    = static_cast<size_t>(size * 8),
			.rxlength  = 8, // 1 byte for status
			.tx_buffer = buffer,
			.rx_buffer = &status,
		},
		.command_bits = 8,  // 1 byte command
		.address_bits = 16, // 2 byte address
		.dummy_bits   = 0,
	};

	esp_err_t err = spi_device_polling_transmit(details::spi_device, &transaction.base);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "failed to transfer %s (%d)", esp_err_to_name(err), err);
		return ue_t{error::FAILED};
	}
	return {};
}
}
