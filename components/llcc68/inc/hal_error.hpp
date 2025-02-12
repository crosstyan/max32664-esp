#ifndef D4D45A87_A012_494D_90D2_B4AE0DDE2487
#define D4D45A87_A012_494D_90D2_B4AE0DDE2487
#include <cstdint>
#include <array>
#include <tuple>

#define APP_ERR_TBL_IT(err) {err, #err}

// https://stackoverflow.com/questions/35089273/why-errno-when-posix-function-indicate-error-condition-by-returning-1-or-null
// I was thinking about something like error stack.
// like call stack, but the error context could be pushed
namespace error {
using t = int32_t;

// same definition as esp_err.h
// https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/error-codes.html
constexpr t OK     = 0;
constexpr t FAILED = -1;

constexpr t INVALID_ARG      = 0x102; /*!< Invalid argument */
constexpr t INVALID_STATE    = 0x103; /*!< Invalid state */
constexpr t INVALID_SIZE     = 0x104; /*!< Invalid size */
constexpr t NOT_FOUND        = 0x105; /*!< Requested resource not found */
constexpr t NOT_SUPPORTED    = 0x106; /*!< Operation or feature not supported */
constexpr t TIMEOUT          = 0x107; /*!< Operation timed out */
constexpr t INVALID_RESPONSE = 0x108; /*!< Received response was invalid */
constexpr t INVALID_CRC      = 0x109; /*!< CRC or checksum was invalid */
constexpr t INVALID_VERSION  = 0x10A; /*!< Version was invalid */
constexpr t INVALID_MAC      = 0x10B; /*!< MAC address was invalid */
constexpr t NOT_FINISHED     = 0x10C; /*!< Operation has not fully completed */
constexpr t NOT_ALLOWED      = 0x10D; /*!< Operation is not allowed */

// new defined generic error codes
constexpr t GENERIC_ERR_BASE = 0x120;
constexpr t AGAIN            = GENERIC_ERR_BASE + 1; /*!< Operation failed, retry */
constexpr t BUSY             = GENERIC_ERR_BASE + 2; /*!< Busy */

constexpr t SPI_ERR_BASE = 0x1'2000;
// A transaction from host took too long to complete and triggered an internal watchdog.
// The watchdog mechanism can be disabled by host; it is meant to ensure all outcomes are flagged to the host MCU.
constexpr t SPI_TIMEOUT = SPI_ERR_BASE + 1;
// Processor was unable to process command either because of an invalid opcode or
// because an incorrect number of parameters has been provided.
constexpr t SPI_CMD_INVALID = SPI_ERR_BASE + 2;
// The command was successfully processed, however the chip could not execute the command;
// for instance it was unable to enter the specified device mode or send the requested data
constexpr t SPI_CMD_FAILED = SPI_ERR_BASE + 3;

constexpr t RADIO_ERR_BASE                 = 0x1'3000;
constexpr t RADIO_CHIP_NOT_FOUND           = RADIO_ERR_BASE + 1;
constexpr t RADIO_INVALID_TCXO_VOLTAGE     = RADIO_ERR_BASE + 2;
constexpr t RADIO_INVALID_CODING_RATE      = RADIO_ERR_BASE + 3;
constexpr t RADIO_INVALID_SPREADING_FACTOR = RADIO_ERR_BASE + 4;
constexpr t RADIO_INVALID_BANDWIDTH        = RADIO_ERR_BASE + 5;
constexpr t RADIO_INVALID_FREQUENCY        = RADIO_ERR_BASE + 6;
constexpr t RADIO_INVALID_OUTPUT_POWER     = RADIO_ERR_BASE + 7;
constexpr t RADIO_INVALID_CAD_RESULT       = RADIO_ERR_BASE + 8;
constexpr t RADIO_WRONG_MODERN             = RADIO_ERR_BASE + 9;
constexpr t RADIO_RX_TIMEOUT               = RADIO_ERR_BASE + 10;
constexpr t RADIO_CRC_MISMATCH             = RADIO_ERR_BASE + 11;
constexpr t RADIO_BUSY_TX                  = RADIO_ERR_BASE + 12;

constexpr auto error_table = std::to_array<std::tuple<t, const char *>>(
	{
		APP_ERR_TBL_IT(OK),
		APP_ERR_TBL_IT(FAILED),
		APP_ERR_TBL_IT(INVALID_ARG),
		APP_ERR_TBL_IT(INVALID_STATE),
		APP_ERR_TBL_IT(INVALID_SIZE),
		APP_ERR_TBL_IT(NOT_FOUND),
		APP_ERR_TBL_IT(NOT_SUPPORTED),
		APP_ERR_TBL_IT(TIMEOUT),
		APP_ERR_TBL_IT(INVALID_RESPONSE),
		APP_ERR_TBL_IT(INVALID_CRC),
		APP_ERR_TBL_IT(INVALID_VERSION),

		APP_ERR_TBL_IT(AGAIN),
		APP_ERR_TBL_IT(BUSY),

		APP_ERR_TBL_IT(SPI_TIMEOUT),
		APP_ERR_TBL_IT(SPI_CMD_INVALID),
		APP_ERR_TBL_IT(SPI_CMD_FAILED),

		APP_ERR_TBL_IT(RADIO_CHIP_NOT_FOUND),
		APP_ERR_TBL_IT(RADIO_INVALID_TCXO_VOLTAGE),
		APP_ERR_TBL_IT(RADIO_INVALID_CODING_RATE),
		APP_ERR_TBL_IT(RADIO_INVALID_SPREADING_FACTOR),
		APP_ERR_TBL_IT(RADIO_INVALID_BANDWIDTH),
		APP_ERR_TBL_IT(RADIO_INVALID_FREQUENCY),
		APP_ERR_TBL_IT(RADIO_INVALID_OUTPUT_POWER),
		APP_ERR_TBL_IT(RADIO_INVALID_CAD_RESULT),
		APP_ERR_TBL_IT(RADIO_WRONG_MODERN),
		APP_ERR_TBL_IT(RADIO_RX_TIMEOUT),
		APP_ERR_TBL_IT(RADIO_CRC_MISMATCH),
		APP_ERR_TBL_IT(RADIO_BUSY_TX),
	});

inline const char *err_to_str(t err) {
	for (const auto &[code, name] : error_table) {
		if (code == err) {
			return name;
		}
	}
	return "UNKNOWN";
}
}

#endif /* D4D45A87_A012_494D_90D2_B4AE0DDE2487 */
