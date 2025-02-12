//
// Created by Kurosu Chan on 2024/1/18.
//

#include "hal_gpio.hpp"
#include "hal_spi.hpp"
#include "utils/utils.hpp"

// #define SKIP_ERROR_CHECK
namespace hal::spi {
Result<uint16_t, error_t>
register_value(uint16_t reg, uint8_t msb, uint8_t lsb) {
	// TODO
	std::unreachable();
};

Result<Unit, error_t>
set_register_value(uint16_t reg, uint16_t value, uint8_t msb, uint8_t lsb) {
	// TODO
	std::unreachable();
};

Result<uint8_t, error_t> read_register(uint16_t reg) {
	uint8_t resp = 0;
	if constexpr (!SPI_IS_STREAM_TYPE) {
		transfer(SPI_READ_COMMAND, reg, nullptr, &resp, 1);
	} else {
		uint8_t cmd[]  = {SPI_READ_COMMAND, static_cast<uint8_t>((reg >> 8) & 0xff), static_cast<uint8_t>(reg & 0xff)};
		const auto res = transfer_stream(cmd, std::size(cmd), false, nullptr, &resp, 1, true, DEFAULT_TIMEOUT_MS);
		if (!res.has_value()) {
			return ue_t{res.error()};
		}
	}
	return resp;
};

Result<Unit, error_t> read_register_burst(uint16_t reg, uint8_t *buffer, uint8_t size) {
	if constexpr (!SPI_IS_STREAM_TYPE) {
		transfer(SPI_READ_COMMAND, reg, nullptr, buffer, size);
	} else {
		uint8_t cmd[]  = {SPI_READ_COMMAND, static_cast<uint8_t>((reg >> 8) & 0xff), static_cast<uint8_t>(reg & 0xff)};
		const auto res = transfer_stream(cmd, std::size(cmd), false, nullptr, buffer, size, true, DEFAULT_TIMEOUT_MS);
		if (!res.has_value()) {
			return ue_t{res.error()};
		}
	}
	return {};
};

Result<Unit, error_t> write_register(const uint16_t reg, const uint8_t value) {
	if constexpr (!SPI_IS_STREAM_TYPE) {
		transfer(SPI_WRITE_COMMAND, reg, &value, nullptr, 1);
	} else {
		uint8_t cmd[]  = {SPI_WRITE_COMMAND, static_cast<uint8_t>((reg >> 8) & 0xff), static_cast<uint8_t>(reg & 0xff)};
		const auto res = transfer_stream(cmd, std::size(cmd), true, &value, nullptr, 1, true, DEFAULT_TIMEOUT_MS);
		if (!res.has_value()) {
			return ue_t{res.error()};
		}
	}
	return {};
};

Result<Unit, error_t> write_register_burst(uint16_t reg, const uint8_t *buffer, uint8_t size) {
	if constexpr (!SPI_IS_STREAM_TYPE) {
		transfer(SPI_WRITE_COMMAND, reg, buffer, nullptr, size);
	} else {
		uint8_t cmd[]  = {SPI_WRITE_COMMAND, static_cast<uint8_t>((reg >> 8) & 0xff), static_cast<uint8_t>(reg & 0xff)};
		const auto res = transfer_stream(cmd, std::size(cmd), true, buffer, nullptr, size, true, DEFAULT_TIMEOUT_MS);
		if (!res.has_value()) {
			return ue_t{res.error()};
		}
	}
	return {};
};

void transfer(const uint8_t cmd, const uint16_t reg, const uint8_t *out, uint8_t *in, const uint8_t size) {
	before_transaction();
	cs_low();
	if constexpr (SPI_ADDR_WIDTH <= 8) {
		hal_transfer(reg | cmd);
	} else {
		hal_transfer((reg >> 8) | cmd);
		hal_transfer(reg & 0xff);
	}

	if (cmd == SPI_READ_COMMAND) {
		for (uint8_t i = 0; i < size; i++) {
			in[i] = hal_transfer(0x00);
		}
	} else if (cmd == SPI_WRITE_COMMAND) {
		for (uint8_t i = 0; i < size; i++) {
			hal_transfer(out[i]);
		}
	}
	cs_high();
	after_transaction();
};

Result<Unit, error_t>
transfer_stream(const uint8_t *cmd, const uint8_t cmd_size, const bool write, const uint8_t *out, uint8_t *in, const uint8_t size, bool wait, const size_t timeout_ms) {
	// TODO
	std::unreachable();
};
}
