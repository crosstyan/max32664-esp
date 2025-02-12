//
// Created by Kurosu Chan on 2024/1/26.
//

#ifndef LLCC68_DEFINITIONS_H
#define LLCC68_DEFINITIONS_H
#include <cstddef>
#include <cstdint>
#include <tuple>
#include "radio_definitions.hpp"

namespace llcc68 {
using freq_t                            = float;
constexpr uint8_t DEFAULT_SYNC_WORD     = RADIOLIB_SX126X_SYNC_WORD_PRIVATE;
constexpr uint16_t DEFAULT_PREAMBLE_LEN = 8;
constexpr uint8_t DEFAULT_CRC_TYPE      = RADIOLIB_SX126X_LORA_CRC_OFF;
constexpr uint8_t DEFAULT_HEADER_TYPE   = RADIOLIB_SX126X_LORA_HEADER_EXPLICIT;
constexpr uint8_t DEFAULT_IQ_TYPE       = RADIOLIB_SX126X_LORA_IQ_STANDARD;
constexpr uint8_t PACKET_TYPE           = RADIOLIB_SX126X_PACKET_TYPE_LORA;

constexpr auto DEFAULT_BW           = RADIOLIB_SX126X_LORA_BW_125_0;
constexpr auto DEFAULT_SF           = 8;
constexpr auto DEFAULT_CR           = RADIOLIB_SX126X_LORA_CR_4_7;
constexpr auto DEFAULT_LDR_OPTIMIZE = RADIOLIB_SX126X_LORA_LOW_DATA_RATE_OPTIMIZE_ON;
constexpr auto DEFAULT_FREQUENCY    = freq_t{433.2};

enum class COMMAND_STATUS : uint8_t {
	RESERVED                   = 0x00,
	RFU                        = 0x01,
	DATA_AVAILABLE             = 0x02,
	COMMAND_TIMEOUT            = 0x03,
	COMMAND_PROCESSING_ERROR   = 0x04,
	FAILURE_TO_EXECUTE_COMMAND = 0x05,
	COMMAND_TX_DONE            = 0x06,
};

enum class CHIP_MODE : uint8_t {
	UNUSED    = 0x00,
	RFU       = 0x01,
	STBY_RC   = 0x02,
	STBY_XOSC = 0x03,
	FS        = 0x04,
	RX        = 0x05,
	TX        = 0x06,
};


struct __attribute__((packed)) status_t {
	uint8_t reserved_1 : 1;
	COMMAND_STATUS command_status : 3;
	CHIP_MODE chip_mode : 3;
	uint8_t reserved_2 : 1;
};
static_assert(sizeof(status_t) == 1);

struct parameters {
	uint8_t bw            = DEFAULT_BW;
	uint8_t sf            = DEFAULT_SF;
	uint8_t cr            = DEFAULT_CR;
	uint8_t ldr_optimize  = DEFAULT_LDR_OPTIMIZE;
	uint16_t preamble_len = DEFAULT_PREAMBLE_LEN;
	uint8_t sync_word     = DEFAULT_SYNC_WORD;
	freq_t frequency      = DEFAULT_FREQUENCY;

	constexpr static size_t size() {
		return sizeof(parameters);
	}
} __attribute__((packed));

struct calc_time_on_air_t {
	uint8_t bw;
	uint8_t sf;
	uint8_t cr;
	uint16_t preamble_length;
	static constexpr uint8_t crc_type    = DEFAULT_CRC_TYPE;
	static constexpr uint8_t header_type = DEFAULT_HEADER_TYPE;
	static constexpr uint8_t iq_type     = DEFAULT_IQ_TYPE;
	static calc_time_on_air_t from_parameters(const parameters &params) {
		return calc_time_on_air_t{
			.bw              = params.bw,
			.sf              = params.sf,
			.cr              = params.cr,
			.preamble_length = params.preamble_len};
	}
};

constexpr bool in_range(const auto v, const auto min, const auto max) {
	return v >= min && v <= max;
}

constexpr bool valid_cr(const uint8_t cr) {
	switch (cr) {
	case RADIOLIB_SX126X_LORA_CR_4_5:
	case RADIOLIB_SX126X_LORA_CR_4_6:
	case RADIOLIB_SX126X_LORA_CR_4_7:
	case RADIOLIB_SX126X_LORA_CR_4_8:
		return true;
	default:
		return false;
	}
}

constexpr std::tuple<uint8_t, uint8_t>
cr_to_ratio(const uint8_t cr) {
	switch (cr) {
	case RADIOLIB_SX126X_LORA_CR_4_5:
		return {4, 5};
	case RADIOLIB_SX126X_LORA_CR_4_6:
		return {4, 6};
	case RADIOLIB_SX126X_LORA_CR_4_7:
		return {4, 7};
	case RADIOLIB_SX126X_LORA_CR_4_8:
		return {4, 8};
	default:
		return {0, 1};
	}
}

constexpr bool valid_ldr_optimize(const uint8_t ldr_optimize) {
	if (ldr_optimize > RADIOLIB_SX126X_LORA_LOW_DATA_RATE_OPTIMIZE_ON) {
		return false;
	}
	return true;
}

constexpr bool valid_bw(const uint8_t bw) {
	switch (bw) {
	case RADIOLIB_SX126X_LORA_BW_7_8:
	case RADIOLIB_SX126X_LORA_BW_10_4:
	case RADIOLIB_SX126X_LORA_BW_15_6:
	case RADIOLIB_SX126X_LORA_BW_20_8:
	case RADIOLIB_SX126X_LORA_BW_31_25:
	case RADIOLIB_SX126X_LORA_BW_41_7:
	case RADIOLIB_SX126X_LORA_BW_62_5:
	case RADIOLIB_SX126X_LORA_BW_125_0:
	case RADIOLIB_SX126X_LORA_BW_250_0:
	case RADIOLIB_SX126X_LORA_BW_500_0:
		return true;
	default:
		return false;
	}
}

using bw_t = float;
constexpr bw_t bw_khz(const uint8_t bw) {
	switch (bw) {
	case RADIOLIB_SX126X_LORA_BW_7_8:
		return bw_t{7.8f};
	case RADIOLIB_SX126X_LORA_BW_10_4:
		return bw_t{10.4f};
	case RADIOLIB_SX126X_LORA_BW_15_6:
		return bw_t{15.6f};
	case RADIOLIB_SX126X_LORA_BW_20_8:
		return bw_t{20.8f};
	case RADIOLIB_SX126X_LORA_BW_31_25:
		return bw_t{31.25f};
	case RADIOLIB_SX126X_LORA_BW_41_7:
		return bw_t{41.7f};
	case RADIOLIB_SX126X_LORA_BW_62_5:
		return bw_t{62.5f};
	case RADIOLIB_SX126X_LORA_BW_125_0:
		return bw_t{125.0f};
	case RADIOLIB_SX126X_LORA_BW_250_0:
		return bw_t{250.0f};
	case RADIOLIB_SX126X_LORA_BW_500_0:
		return bw_t{500.0f};
	default:
		return bw_t{0};
	}
}

constexpr bool valid_sf(const uint8_t bw, const uint8_t sf) {
	if (const bool ok = valid_bw(bw); not ok) {
		return false;
	}
	switch (bw) {
	case RADIOLIB_SX126X_LORA_BW_125_0:
		return in_range(sf, 5, 9);
	case RADIOLIB_SX126X_LORA_BW_250_0:
		return in_range(sf, 5, 10);
	case RADIOLIB_SX126X_LORA_BW_500_0:
		return in_range(sf, 5, 11);
	default:
		return in_range(sf, 5, 12);
	}
}

constexpr bool valid_freq(const freq_t freq) {
	return in_range(freq, freq_t{150.0}, freq_t{960.0});
}

/**
 * \brief check if the parameters are valid. if not, set them to default.
 * \param params the parameters to check
 * \return if the parameters are valid, and the modified parameters
 */
constexpr std::tuple<bool, parameters>
check_fix_params(parameters params) {
	bool ok = true;

	if (not valid_bw(params.bw)) {
		params.bw = DEFAULT_BW;
		ok        = false;
	}
	if (not valid_sf(params.bw, params.sf)) {
		params.sf = DEFAULT_SF;
		ok        = false;
	}
	if (not valid_cr(params.cr)) {
		params.cr = DEFAULT_CR;
		ok        = false;
	}
	if (not valid_freq(params.frequency)) {
		params.frequency = DEFAULT_FREQUENCY;
		ok               = false;
	}
	if (not valid_ldr_optimize(params.ldr_optimize)) {
		params.ldr_optimize = DEFAULT_LDR_OPTIMIZE;
		ok                  = false;
	}
	return {ok, params};
}

/**
 * \brief only do the check
 */
constexpr bool check_params(const parameters &params) {
	if (not valid_bw(params.bw)) {
		return false;
	}
	if (not valid_sf(params.bw, params.sf)) {
		return false;
	}
	if (not valid_cr(params.cr)) {
		return false;
	}
	if (not valid_freq(params.frequency)) {
		return false;
	}
	if (not valid_ldr_optimize(params.ldr_optimize)) {
		return false;
	}
	return true;
}
}

#endif // LLCC68_DEFINITIONS_H
