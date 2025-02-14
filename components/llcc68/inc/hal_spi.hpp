//
// Created by Kurosu Chan on 2024/1/17.
//
#ifndef SPI_H
#define SPI_H
#include <cstdint>
#include <span>
#include <utility>
#include "radio_definitions.hpp"
#include "llcc68_definitions.hpp"
#include "hal_gpio.hpp"
#include "utils/result.hpp"
#include "hal_error.hpp"


namespace hal::spi {
template <typename T, typename E>
using Result       = utils::Result<T, E>;
using Unit         = utils::Unit;
constexpr auto TAG = "spi";

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


inline error_t status_to_err(const uint8_t status) {
	const auto st = *reinterpret_cast<const llcc68::status_t *>(&status);
	switch (st.command_status) {
	case llcc68::COMMAND_STATUS::COMMAND_TIMEOUT:
		return error::SPI_TIMEOUT;
	case llcc68::COMMAND_STATUS::FAILURE_TO_EXECUTE_COMMAND:
		return error::SPI_CMD_FAILED;
	case llcc68::COMMAND_STATUS::INVALID:
		return error::SPI_CMD_INVALID;
	default:
		return error::OK;
	}
}

/*!
  See also LLCC68 chapter 10 Host Controller Interface
*/

/*!
  \brief perform SPI transaction
  \param cmd SPI operation command.
  \param cmd_size SPI command length in bytes.
  \param data Data that will be transferred from slave to master.
  \param size Number of bytes to transfer.
  \note the first byte in `in` is the status byte, it's expected to check it for user
*/
Result<Unit, error_t>
read_stream_raw(uint8_t cmd, uint8_t *in, const uint8_t size, const size_t timeout_ms);

Result<Unit, error_t>
read_stream_raw(uint8_t cmd, std::span<uint8_t> in, const size_t timeout_ms);

/*!
  \brief perform SPI transaction
  \param cmd SPI operation command.
  \param cmd_size SPI command length in bytes.
  \param data Data that will be transferred from slave to master.
  \param size Number of bytes to transfer.
  \tparam N the size of the internal data buffer (since one copy is needed for checking the status byte)
  \note the first byte in `in` is the status byte, it's expected to check it for user
*/
template <size_t N = 18>
inline Result<Unit, error_t>
read_stream(uint8_t cmd, uint8_t *data, uint8_t size, const size_t timeout_ms = DEFAULT_TIMEOUT_MS) {
	using ue_t = utils::unexpected<error_t>;
	uint8_t buffer_[N];
	// with 1 byte for status
	const auto buffer_size = size + 1;
	if (buffer_size > N) {
		return ue_t{error::INVALID_SIZE};
	}
	auto buffer = std::span{buffer_, buffer_size};
	auto res    = read_stream_raw(cmd, buffer, timeout_ms);
	if (not res) {
		return ue_t{res.error()};
	}
	const auto status = buffer[0];
	if (const auto err = status_to_err(status); err != error::OK) {
		return ue_t{err};
	}
	std::copy(std::span{buffer.data() + 1, size}, data);
	return {};
};

template <size_t N = 18>
Result<Unit, error_t>
read_stream(uint8_t cmd, std::span<uint8_t> data, const size_t timeout_ms = DEFAULT_TIMEOUT_MS) {
	return read_stream<N>(cmd, data.data(), data.size(), timeout_ms);
};


/*!
  \brief Method to perform a write transaction with SPI stream.
  \param cmd SPI operation command.
  \param cmd_size SPI command length in bytes.
  \param data Data that will be transferred from master to slave.
  \param size Number of bytes to transfer.
*/
Result<Unit, error_t>
write_stream_no_check(uint8_t cmd, const uint8_t *data, const uint8_t size, const size_t timeout_ms);

Result<Unit, error_t>
write_stream_no_check(uint8_t cmd, std::span<const uint8_t> data, const size_t timeout_ms);

Result<Unit, error_t>
write_stream_with_ext_status(uint8_t cmd, std::span<const uint8_t> data, std::span<uint8_t> status_out, const size_t timeout_ms);

template <size_t N = 18>
Result<Unit, error_t>
write_stream(uint8_t cmd, std::span<const uint8_t> data, const size_t timeout_ms) {
	uint8_t buffer_[N];
	if (data.size() > N) {
		return ue_t{error::INVALID_SIZE};
	}
	auto status_buf = std::span{buffer_, data.size() + 1};
	auto err        = write_stream_with_ext_status(cmd, data, status_buf, timeout_ms);
	if (not err) {
		return err;
	}
	uint8_t i = 0;
	for (const auto st : status_buf) {
		if (const auto err = status_to_err(st); err != error::OK) {
			ESP_LOGE(TAG, "failed to write with 0x%02x at byte %u", st, i);
			return ue_t{err};
		}
		i += 1;
	}
	return {};
}

template <size_t N = 18>
Result<Unit, error_t> read_register_burst_with_status(uint16_t reg, std::span<uint8_t> buffer, const size_t timeout_ms) {
	uint8_t buffer_[N];
	auto err = read_stream_raw(SPI_READ_COMMAND, buffer_, timeout_ms);
	if (not err) {
		return err;
	}
	return {};
};

Result<Unit, error_t> write_register_burst(uint16_t reg, std::span<const uint8_t> buffer, const size_t timeout_ms);
}

namespace llcc68 {
using error_t = hal::spi::error_t;
inline error_t check_stream() {
	std::unreachable();
}
}

#endif // SPI_H
