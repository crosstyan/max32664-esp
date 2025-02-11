//
// Created by Kurosu Chan on 2024/1/17.
//
#ifndef SPI_H
#define SPI_H
#include "radio_definitions.h"
#include "result.h"
#include "hal_gpio.h"

namespace error {
/**
 * \brief SPI and Radio error codes
 */
struct spi {
  static constexpr uint8_t Category = 0x02;
  enum class Code : uint8_t {
    OK                    = 0,
    UNKNOWN               = 1,
    SPI_INVALID_BIT_RANGE = 2,
    // A transaction from host took too long to complete and triggered an internal watchdog.
    // The watchdog mechanism can be disabled by host; it is meant to ensure all outcomes are flagged to the host MCU.
    SPI_CMD_TIMEOUT = 3,
    // Processor was unable to process command either because of an invalid opcode or
    // because an incorrect number of parameters has been provided.
    SPI_CMD_INVALID = 4,
    // The command was successfully processed, however the chip could not execute the command;
    // for instance it was unable to enter the specified device mode or send the requested data
    SPI_CMD_FAILED           = 5,
    CHIP_NOT_FOUND           = 6,
    INVALID_TCXO_VOLTAGE     = 7,
    INVALID_CODING_RATE      = 8,
    INVALID_SPREADING_FACTOR = 9,
    INVALID_BANDWIDTH        = 10,
    INVALID_FREQUENCY        = 11,
    INVALID_OUTPUT_POWER     = 12,
    PACKET_TOO_LONG          = 13,
    INVALID_CAD_RESULT       = 14,
    WRONG_MODERN             = 15,
    RX_TIMEOUT               = 16,
    CRC_MISMATCH             = 17,
    BUSY_TX                  = 18,
    __LAST__
  };

  struct t {
    Code code;
    operator error::t() const { // NOLINT(*-explicit-constructor)
      return static_cast<error::t>(generic::Category << 8 | static_cast<uint8_t>(code));
    }
    bool operator==(t rhs) const {
      return code == rhs.code;
    }
    bool operator==(Code code) const {
      return this->code == code;
    }
  };

  static constexpr Code from_num(uint8_t num) {
    if (num < static_cast<uint8_t>(Code::__LAST__)) {
      return static_cast<Code>(num);
    }
    return Code::UNKNOWN;
  }

  static constexpr optional<t> as(const error::t e) {
    const uint8_t cat  = e >> 8;
    const uint8_t code = static_cast<uint8_t>(e & 0xff);
    if (cat == Category) {
      return t{from_num(code)};
    }
    return nullopt;
  }
};
}

namespace hal::spi {
constexpr auto MOSI_PIN = PA7;
constexpr auto MISO_PIN = PA6;
constexpr auto SCLK_PIN = PA5;
constexpr auto BUSY_PIN = PA2;
constexpr auto CS_PIN   = PA4;
using error_t           = error::spi::t;
using ue_t              = unexpected<error_t>;
using Code              = error::spi::Code;

/*!
  \brief Basic SPI read command. Defaults to 0x00.
*/
constexpr uint8_t SPIreadCommand = RADIOLIB_SX126X_CMD_READ_REGISTER;

/*!
  \brief Basic SPI write command. Defaults to 0x80.
*/
constexpr uint8_t SPIwriteCommand = RADIOLIB_SX126X_CMD_WRITE_REGISTER;

/*!
  \brief Basic SPI no-operation command. Defaults to 0x00.
*/
constexpr uint8_t SPInopCommand = RADIOLIB_SX126X_CMD_NOP;

/*!
  \brief SPI address width. Defaults to 8, currently only supports 8 and 16-bit addresses.
*/
constexpr uint8_t SPIaddrWidth = 8;
static_assert(SPIaddrWidth == 8 || SPIaddrWidth == 16, "SPI address width must be 8 or 16");

/*!
  \brief Whether the SPI interface is stream-type (e.g. SX126x) or register-type (e.g. SX127x).
*/
constexpr bool SPIstreamType = true;

constexpr size_t DEFAULT_TIMEOUT_MS = 1000;

namespace details {
  inline SPI_HandleTypeDef __spi__;
}

inline void init() {
  auto &spi = details::__spi__;
  // configure SPI Pins
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef gpio{};
  // A5 A6 A7
  gpio.Pin       = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
  gpio.Mode      = GPIO_MODE_AF_PP;
  gpio.Pull      = GPIO_NOPULL;
  gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF0_SPI1;
  HAL_GPIO_Init(GPIOA, &gpio);

  __HAL_RCC_SPI1_CLK_ENABLE();
  spi.Instance         = SPI1;
  spi.Init.Mode        = SPI_MODE_MASTER;
  spi.Init.Direction   = SPI_DIRECTION_2LINES;
  spi.Init.DataSize    = SPI_DATASIZE_8BIT;
  spi.Init.CLKPolarity = SPI_POLARITY_LOW;
  spi.Init.CLKPhase    = SPI_PHASE_1EDGE;
  spi.Init.NSS         = SPI_NSS_SOFT;
  // https://community.st.com/t5/missing-articles/how-is-my-spi-s-baudrate-calculated-using-stm32cubemx/td-p/49592
  // https://stackoverflow.com/questions/58125052/the-best-spi-baudrate-prescaler
  spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  spi.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  spi.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  // https://electronics.stackexchange.com/questions/28179/spi-ti-or-motorola-mode
  spi.Init.TIMode = SPI_TIMODE_DISABLE;

  if (HAL_SPI_Init(&spi) != HAL_OK) {
    while (true)
      ;
  }

  // https://github.com/stm32duino/Arduino_Core_STM32/wiki/HAL-configuration
  GPIO_InitTypeDef gpio_cs{};
  // PA4
  gpio_cs.Pin   = GPIO_PIN_4;
  gpio_cs.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio_cs.Pull  = GPIO_NOPULL;
  gpio_cs.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio_cs);

  GPIO_InitTypeDef gpio_busy{};
  // PA2
  gpio_busy.Pin   = GPIO_PIN_2;
  gpio_busy.Mode  = GPIO_MODE_INPUT;
  gpio_busy.Pull  = GPIO_NOPULL;
  gpio_busy.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio_busy);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

inline void cs_low() {
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}

inline void cs_high() {
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

inline void before_transaction() {}
inline void after_transaction() {}

inline uint8_t hal_transfer(const uint8_t data) {
  uint8_t recv           = 0;
  auto &spi              = details::__spi__;
  const volatile auto ok = HAL_SPI_TransmitReceive(&spi, const_cast<uint8_t *>(&data), &recv, 1, 10);
  static_cast<void>(ok);
  return recv;
}

inline Code hal_transfer_buffer(const uint8_t *tx_data, uint8_t *rx_data, size_t size) {
  auto &spi     = details::__spi__;
  const auto ok = HAL_SPI_TransmitReceive(&spi, const_cast<uint8_t *>(tx_data), rx_data, size, 100);
  if (ok != HAL_OK) {
    return Code::SPI_CMD_FAILED;
  }
  return Code::OK;
}

inline void delay_ms(const size_t ms) {
  ::delay(ms);
}

static uint32_t hal_millis() {
  return ::millis();
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
      return error_t{Code::SPI_CMD_TIMEOUT};
    } else if (IN_MASKED == RADIOLIB_SX126X_STATUS_CMD_INVALID) {
      return error_t{Code::SPI_CMD_INVALID};
    } else if (IN_MASKED == RADIOLIB_SX126X_STATUS_CMD_FAILED) {
      return error_t{Code::SPI_CMD_FAILED};
    } else if ((in == 0x00) || (in == 0xFF)) {
      return error_t{Code::SPI_CMD_INVALID};
    }
#endif

    return error_t{Code::OK};
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
