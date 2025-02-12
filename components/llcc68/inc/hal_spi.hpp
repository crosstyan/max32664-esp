//
// Created by Kurosu Chan on 2024/1/17.
//
#ifndef SPI_H
#define SPI_H
#include <cstdint>
#include <span>
#include "radio_definitions.hpp"
#include "hal_gpio.hpp"
#include "utils/result.hpp"
#include "hal_error.hpp"


namespace hal::spi {
template <typename T, typename E>
using Result = utils::Result<T, E>;
using Unit   = utils::Unit;

constexpr auto MOSI_PIN = GPIO_NUM_12;
constexpr auto MISO_PIN = GPIO_NUM_4;
constexpr auto SCLK_PIN = GPIO_NUM_13;
constexpr auto BUSY_PIN = GPIO_NUM_5;
constexpr auto CS_PIN   = GPIO_NUM_14;
using error_t           = error::t;
using ue_t              = utils::unexpected<error_t>;

/*!
  \brief Basic SPI read command. Defaults to 0x00.
*/
constexpr uint8_t SPI_READ_COMMAND = RADIOLIB_SX126X_CMD_READ_REGISTER;

/*!
  \brief Basic SPI write command. Defaults to 0x80.
*/
constexpr uint8_t SPI_WRITE_COMMAND = RADIOLIB_SX126X_CMD_WRITE_REGISTER;

/*!
  \brief Basic SPI no-operation command. Defaults to 0x00.
*/
constexpr uint8_t SPI_NOP_COMMAND = RADIOLIB_SX126X_CMD_NOP;

/*!
  \brief Whether the SPI interface is stream-type (e.g. SX126x) or register-type (e.g. SX127x).
*/
constexpr bool SPI_IS_STREAM_TYPE = true;

constexpr size_t DEFAULT_TIMEOUT_MS = 1000;

/*!
  \brief Initialize the SPI interface.
*/
void init();

/*!
  \brief SPI burst read method.
  \param reg Address of SPI register to read.
  \param buffer Pointer to array that will hold the read data.
  \param size Number of bytes that will be read.
*/
Result<Unit, error_t> read_register_burst_with_status(uint16_t reg, uint8_t *buffer, uint8_t size);

/*!
  \brief SPI burst write method.
  \param reg Address of SPI register to write.
  \param buffer Pointer to array that holds the data that will be written.
  \param size Number of bytes that will be written.
*/
Result<uint8_t, error_t> write_register_burst(uint16_t reg, const uint8_t *buffer, uint8_t size);

Result<Unit, error_t>
read_stream_with_status(const uint8_t *cmd, const uint8_t cmd_size, uint8_t *in, const uint8_t size, const size_t timeout_ms);

/*!
  \brief Method to perform a read transaction with SPI stream.
  \param cmd SPI operation command.
  \param cmd_size SPI command length in bytes.
  \param data Data that will be transferred from slave to master.
  \param size Number of bytes to transfer.
  \param wait Whether to wait for some GPIO at the end of transfer (e.g. BUSY line on SX126x/SX128x).
*/
inline Result<Unit, error_t>
read_stream(uint8_t *cmd, uint8_t cmd_size, uint8_t *data, uint8_t size);

/*!
  \brief Method to perform a read transaction with SPI stream.
  \param cmd SPI operation command.
  \param data Data that will be transferred from slave to master.
  \param size Number of bytes to transfer.
  \param wait Whether to wait for some GPIO at the end of transfer (e.g. BUSY line on SX126x/SX128x).
*/
inline Result<Unit, error_t>
read_stream(uint8_t cmd, uint8_t *data, uint8_t size) {
	return read_stream(&cmd, 1, data, size);
};

inline Result<Unit, error_t>
read_stream(uint8_t cmd, std::span<uint8_t> data) {
	return read_stream(cmd, data.data(), data.size());
};

/*!
  \brief Method to perform a write transaction with SPI stream.
  \param cmd SPI operation command.
  \param cmd_size SPI command length in bytes.
  \param data Data that will be transferred from master to slave.
  \param size Number of bytes to transfer.
  \param wait Whether to wait for some GPIO at the end of transfer (e.g. BUSY line on SX126x/SX128x).
*/
inline Result<uint8_t, error_t>
write_stream(const uint8_t *cmd, const uint8_t cmd_size, const uint8_t *data, const uint8_t size);

/*!
  \brief Method to perform a write transaction with SPI stream.
  \param cmd SPI operation command.
  \param data Data that will be transferred from master to slave.
  \param size Number of bytes to transfer.
  \param wait Whether to wait for some GPIO at the end of transfer (e.g. BUSY line on SX126x/SX128x).
*/
inline Result<uint8_t, error_t>
write_stream(const uint8_t cmd, const uint8_t *data, const uint8_t size) {
	return write_stream(&cmd, 1, data, size);
};

inline Result<uint8_t, error_t>
write_stream(const uint8_t cmd, std::span<const uint8_t> data) {
	return write_stream(cmd, data.data(), data.size());
};

namespace llcc68 {
	inline error_t parse_status(const uint8_t in) {
#ifndef SKIP_ERROR_CHECK
		constexpr auto MASK = 0b00001110;
		if (const auto IN_MASKED = in & MASK; IN_MASKED == RADIOLIB_SX126X_STATUS_CMD_TIMEOUT) {
			return error_t{error::SPI_TIMEOUT};
		} else if (IN_MASKED == RADIOLIB_SX126X_STATUS_CMD_INVALID) {
			return error_t{error::SPI_CMD_INVALID};
		} else if (IN_MASKED == RADIOLIB_SX126X_STATUS_CMD_FAILED) {
			return error_t{error::SPI_CMD_FAILED};
		} else if ((in == 0x00) || (in == 0xFF)) {
			return error_t{error::SPI_CMD_INVALID};
		}
#endif
		return error_t{error::OK};
	}

	inline error_t check_stream() {
		uint8_t status        = 0;
		constexpr uint8_t cmd = RADIOLIB_SX126X_CMD_GET_STATUS;
		const auto st         = spi::transfer_stream(&cmd, 1, false, nullptr, &status, 1);
		if (!st.has_value()) {
			return st.error();
		}
		return parse_status(status);
	}
}
}

#endif // SPI_H
