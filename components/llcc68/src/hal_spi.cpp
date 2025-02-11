//
// Created by Kurosu Chan on 2024/1/18.
//

#include "hal_gpio.h"
#include "hal_spi.h"
#include "app_utils.h"

// #define SKIP_ERROR_CHECK
namespace hal::spi {
Result<uint16_t, error_t>
register_value(uint16_t reg, uint8_t msb, uint8_t lsb) {
  if ((msb > 7) || (lsb > 7) || (lsb > msb)) {
    return ue_t{error_t{Code::SPI_INVALID_BIT_RANGE}};
  }
  const uint8_t raw    = read_register(reg);
  const uint8_t masked = raw & ((0b11111111 << lsb) & (0b11111111 >> (7 - msb)));
  return masked;
};

Result<Unit, error_t>
set_register_value(uint16_t reg, uint16_t value, uint8_t msb, uint8_t lsb) {
  if ((msb > 7) || (lsb > 7) || (lsb > msb)) {
    return ue_t{error_t{Code::SPI_INVALID_BIT_RANGE}};
  }

  const uint8_t val     = read_register(reg);
  const uint8_t mask    = ~((0b11111111 << (msb + 1)) | (0b11111111 >> (8 - lsb)));
  const uint8_t new_val = (val & ~mask) | (val & mask);
  write_register(reg, new_val);
  return {};
};

Result<uint8_t, error_t> read_register(uint16_t reg) {
  uint8_t resp = 0;
  if constexpr (!SPIstreamType) {
    transfer(SPIreadCommand, reg, nullptr, &resp, 1);
  } else {
    uint8_t cmd[]  = {SPIreadCommand, static_cast<uint8_t>((reg >> 8) & 0xff), static_cast<uint8_t>(reg & 0xff)};
    const auto res = transfer_stream(cmd, std::size(cmd), false, nullptr, &resp, 1, true, DEFAULT_TIMEOUT_MS);
    if (!res.has_value()) {
      return ue_t{res.error()};
    }
  }
  return resp;
};

Result<Unit, error_t> read_register_burst(uint16_t reg, uint8_t *buffer, uint8_t size) {
  if constexpr (!SPIstreamType) {
    transfer(SPIreadCommand, reg, nullptr, buffer, size);
  } else {
    uint8_t cmd[]  = {SPIreadCommand, static_cast<uint8_t>((reg >> 8) & 0xff), static_cast<uint8_t>(reg & 0xff)};
    const auto res = transfer_stream(cmd, std::size(cmd), false, nullptr, buffer, size, true, DEFAULT_TIMEOUT_MS);
    if (!res.has_value()) {
      return ue_t{res.error()};
    }
  }
  return {};
};

Result<Unit, error_t> write_register(const uint16_t reg, const uint8_t value) {
  if constexpr (!SPIstreamType) {
    transfer(SPIwriteCommand, reg, &value, nullptr, 1);
  } else {
    uint8_t cmd[]  = {SPIwriteCommand, static_cast<uint8_t>((reg >> 8) & 0xff), static_cast<uint8_t>(reg & 0xff)};
    const auto res = transfer_stream(cmd, std::size(cmd), true, &value, nullptr, 1, true, DEFAULT_TIMEOUT_MS);
    if (!res.has_value()) {
      return ue_t{res.error()};
    }
  }
  return {};
};

Result<Unit, error_t> write_register_burst(uint16_t reg, const uint8_t *buffer, uint8_t size) {
  if constexpr (!SPIstreamType) {
    transfer(SPIwriteCommand, reg, buffer, nullptr, size);
  } else {
    uint8_t cmd[]  = {SPIwriteCommand, static_cast<uint8_t>((reg >> 8) & 0xff), static_cast<uint8_t>(reg & 0xff)};
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
  if constexpr (SPIaddrWidth <= 8) {
    hal_transfer(reg | cmd);
  } else {
    hal_transfer((reg >> 8) | cmd);
    hal_transfer(reg & 0xff);
  }

  if (cmd == SPIreadCommand) {
    for (uint8_t i = 0; i < size; i++) {
      in[i] = hal_transfer(0x00);
    }
  } else if (cmd == SPIwriteCommand) {
    for (uint8_t i = 0; i < size; i++) {
      hal_transfer(out[i]);
    }
  }
  cs_high();
  after_transaction();
};

Result<Unit, error_t>
transfer_stream(const uint8_t *cmd, const uint8_t cmd_size, const bool write, const uint8_t *out, uint8_t *in, const uint8_t size, bool wait, const size_t timeout_ms) {
  if constexpr (BUSY_PIN == NC) {
    delay_ms(1);
  } else {
    const auto start = millis();
    while (gpio::digital_read(BUSY_PIN)) {
      if (millis() - start > timeout_ms) {
        return ue_t{error_t{Code::SPI_CMD_TIMEOUT}};
      }
    }
  }
  before_transaction();
  cs_low();

  // send command
  for (uint8_t i = 0; i < cmd_size; i++) {
    hal_transfer(cmd[i]);
  }

  auto err = error_t{Code::OK};

  uint8_t st{};
  // Reserved for Future Use (RFU)
  // The command GetStatus() is not strictly necessary since device returns status
  // information also on command bytes.
  if (write) {
    for (uint8_t i = 0; i < size; i++) {
      st  = hal_transfer(out[i]);
      err = llcc68::parse_status(st);
    }
  } else {
    // skip the first byte for read-type commands (status-only)
    st  = hal_transfer(SPInopCommand);
    err = llcc68::parse_status(st);

    for (uint8_t i = 0; i < size; i++) {
      in[i] = hal_transfer(SPInopCommand);
    }
  }
  // LOGI("S", "0x%02x", st);

  cs_high();
  after_transaction();
  // we don't have to wait GPIO
  if (err.code != Code::OK) {
    return ue_t{err};
  }
  return {};
};
}
