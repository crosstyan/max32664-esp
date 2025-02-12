//
// Created by Kurosu Chan on 2024/1/17.
//
#ifndef SPI_H
#define SPI_H
#include "radio_definitions.hpp"
#include "hal_gpio.hpp"
#include "utils/result.hpp"
#include "utils/utils.hpp"
#include "hal_error.hpp"
// https://stackoverflow.com/questions/35089273/why-errno-when-posix-function-indicate-error-condition-by-returning-1-or-null
// I was thinking about something like error stack.
// like call stack, but the error context could be pushed


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
  \brief SPI address width. Defaults to 8, currently only supports 8 and 16-bit addresses.
*/
constexpr uint8_t SPI_ADDR_WIDTH = 8;
static_assert(SPI_ADDR_WIDTH == 8 || SPI_ADDR_WIDTH == 16, "SPI address width must be 8 or 16");

/*!
  \brief Whether the SPI interface is stream-type (e.g. SX126x) or register-type (e.g. SX127x).
*/
constexpr bool SPI_IS_STREAM_TYPE = true;

constexpr size_t DEFAULT_TIMEOUT_MS = 1000;

inline void init() {
	// TODO
}

inline void cs_low() {
	hal::gpio::digital_write(CS_PIN, false);
}

inline void cs_high() {
	hal::gpio::digital_write(CS_PIN, true);
}

inline void before_transaction() {}
inline void after_transaction() {}

inline uint8_t hal_transfer(const uint8_t data) {
	// TODO
	std::unreachable();
}

inline error_t hal_transfer_buffer(const uint8_t *tx_data, uint8_t *rx_data, size_t size) {
	// TODO
	std::unreachable();
}

inline void delay_ms(const size_t ms) {
	utils::delay_ms(ms);
}

static uint32_t hal_millis() {
	return utils::millis();
}

/**
  \brief SPI read method that automatically masks unused bits. This method is the preferred SPI read mechanism.
  \param reg Address of SPI register to read.
  \param msb Most significant bit of the register variable. Bits above this one will be masked out.
  \param lsb Least significant bit of the register variable. Bits below this one will be masked out.
  \returns Masked register value or status code.
*/
Result<uint16_t, error_t>
register_value(uint16_t reg, uint8_t msb = 7, uint8_t lsb = 0);

/*!
  \brief Overwrite-safe SPI write method with verification. This method is the preferred SPI write mechanism.
  \param reg Address of SPI register to write.
  \param value Single byte value that will be written to the SPI register.
  \param msb Most significant bit of the register variable. Bits above this one will not be affected by the write operation.
  \param lsb Least significant bit of the register variable. Bits below this one will not be affected by the write operation.
  \returns \ref status_codes
*/
Result<Unit, error_t>
set_register_value(uint16_t reg, uint16_t value, uint8_t msb = 7, uint8_t lsb = 0);

/*!
  \brief SPI basic read method. Use of this method is reserved for special cases. Most of the time you should use `register_value`.
  \param reg Address of SPI register to read.
  \returns Value that was read from register.
  \sa register_value
*/
Result<uint8_t, error_t> read_register(uint16_t reg);

/*!
  \brief SPI burst read method.
  \param reg Address of SPI register to read.
  \param buffer Pointer to array that will hold the read data.
  \param size Number of bytes that will be read.
*/
Result<Unit, error_t> read_register_burst(uint16_t reg, uint8_t *buffer, uint8_t size);

/*!
  \brief SPI basic write method. Use of this method is reserved for special cases, set_register_value should be used instead.
  \param reg Address of SPI register to write.
  \param value Value that will be written to the register.
  \sa set_register_value
*/
Result<Unit, error_t> write_register(uint16_t reg, uint8_t value);

/*!
  \brief SPI burst write method.
  \param reg Address of SPI register to write.
  \param buffer Pointer to array that holds the data that will be written.
  \param size Number of bytes that will be written.
*/
Result<Unit, error_t> write_register_burst(uint16_t reg, const uint8_t *buffer, uint8_t size);

/*!
  \brief SPI single transfer method.
  \param cmd SPI access command (read/write/burst/...).
  \param reg Address of SPI register to transfer to/from.
  \param out Data that will be transfered from master to slave.
  \param in Data that was transfered from slave to master.
  \param size Number of bytes to transfer.
*/
void transfer(uint8_t cmd, uint16_t reg, const uint8_t *out, uint8_t *in, uint8_t size);

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
*/
Result<Unit, error_t>
transfer_stream(const uint8_t *cmd, uint8_t cmd_size, bool write, const uint8_t *out, uint8_t *in, uint8_t size, bool wait = true, size_t timeout_ms = DEFAULT_TIMEOUT_MS);

/*!
  \brief Method to perform a read transaction with SPI stream.
  \param cmd SPI operation command.
  \param cmd_size SPI command length in bytes.
  \param data Data that will be transferred from slave to master.
  \param size Number of bytes to transfer.
  \param wait Whether to wait for some GPIO at the end of transfer (e.g. BUSY line on SX126x/SX128x).
*/
inline Result<Unit, error_t>
read_stream(uint8_t *cmd, uint8_t cmd_size, uint8_t *data, uint8_t size, bool wait = true) {
	return transfer_stream(cmd, cmd_size, false, nullptr, data, size, wait);
};

/*!
  \brief Method to perform a read transaction with SPI stream.
  \param cmd SPI operation command.
  \param data Data that will be transferred from slave to master.
  \param size Number of bytes to transfer.
  \param wait Whether to wait for some GPIO at the end of transfer (e.g. BUSY line on SX126x/SX128x).
*/
inline Result<Unit, error_t>
read_stream(uint8_t cmd, uint8_t *data, uint8_t size, bool wait = true) {
	return read_stream(&cmd, 1, data, size, wait);
};

/*!
  \brief Method to perform a write transaction with SPI stream.
  \param cmd SPI operation command.
  \param cmd_size SPI command length in bytes.
  \param data Data that will be transferred from master to slave.
  \param size Number of bytes to transfer.
  \param wait Whether to wait for some GPIO at the end of transfer (e.g. BUSY line on SX126x/SX128x).
*/
inline Result<Unit, error_t>
write_stream(const uint8_t *cmd, const uint8_t cmd_size, const uint8_t *data, const uint8_t size, bool wait = true) {
	return transfer_stream(cmd, cmd_size, true, data, nullptr, size, wait);
};

/*!
  \brief Method to perform a write transaction with SPI stream.
  \param cmd SPI operation command.
  \param data Data that will be transferred from master to slave.
  \param size Number of bytes to transfer.
  \param wait Whether to wait for some GPIO at the end of transfer (e.g. BUSY line on SX126x/SX128x).
*/
inline Result<Unit, error_t>
write_stream(const uint8_t cmd, const uint8_t *data, const uint8_t size, const bool wait = true) {
	return write_stream(&cmd, 1, data, size, wait);
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
