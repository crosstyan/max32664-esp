//
// Created by Kurosu Chan on 2024/1/17.
//

#ifndef PWR_MGR_LLCC68_H
#define PWR_MGR_LLCC68_H
#include <cstddef>
#include <chrono>
#include <etl/vector.h>
#include "utils/result.hpp"
#include "utils/instant.hpp"
#include "llcc68_definitions.hpp"
#include "hal_spi.hpp"
#include "fixed_point.h"
#include "app_utils.h"

/**
 * \brief return if STATEVAR has value, otherwise return the error
 * \param STATEVAR the variable to check
 */
#define APP_RADIO_RETURN_ERR(STATEVAR)       \
	{                                        \
		if (!(STATEVAR.has_value())) {       \
			return (ue_t{STATEVAR.error()}); \
		}                                    \
	}

namespace llcc68 {
using namespace hal;
using Instant = utils::Instant;

template <typename T, typename E>
using Result = utils::Result<T, E>;
using Unit   = utils::Unit;

using millisecond       = std::chrono::duration<size_t, std::milli>;
constexpr auto RST_PIN  = PA1;
constexpr auto DIO1_PIN = PA3;

/**
 * \brief Whether to use only LDO regulator (true) or DC-DC regulator (false)
 */
constexpr bool USE_REGULATOR_LDO = false;
/*!
  \brief Whether the module has an XTAL (true) or TCXO (false)

 TCXO (Temperature Compensated Crystal Oscillator) will be disabled if this is set to true
*/
constexpr bool USE_XTAL              = true;
constexpr float DEFAULT_TCXO_VOLTAGE = 1.6f;

using error_t = spi::error_t;
using ue_t    = spi::ue_t;
using Code    = spi::Code;

/**
 * \brief maintaining the transmission state for async transmission
 */
struct transmit_state_t {
	bool is_transmitting = false;
	Instant start{};
	uint16_t expected_duration_ms{};
};

/**
 * @brief private namespace; Don't use anything in this namespace outside of this file
 */
namespace details {
	inline uint32_t __tcxo_delay__{};
	inline transmit_state_t __tx_state__{};
	inline bool __dio_flag__ = false;
}

/*
 * @brief Configure the EXTI for GPIO pin C3
 * @see https://github.com/cnlohr/ch32v003fun/blob/master/examples/exti_pin_change_isr/exti_pin_change_isr.c
 */
void config_exti();

/**
 * \brief check exti flag
 * \return true if exti flag is set, false otherwise
 */
[[nodiscard]]
inline bool flag() {
	return details::__dio_flag__;
}

inline void reset_flag() {
	details::__dio_flag__ = false;
	// for some reason the EXTI needs to be reattacted after interruption is triggered
	config_exti();
}

/**
 * \brief get the flag and reset it to false
 * \return flag before
 */
[[nodiscard]]
inline bool fetch_flag_reset() {
	if (!details::__dio_flag__) {
		return false;
	} else {
		reset_flag();
		return true;
	}
}

static constexpr auto delay_ms = [](const size_t ms) {
	::delay(ms);
};

/*!
  \brief Sets the module to standby mode.
*/
inline Result<Unit, error_t> standby() {
	// immediately attempt to wake up the module by pulling NSS low
	spi::cs_low();
	// Oscillator to be used in standby mode.
	// Can be set to RADIOLIB_SX126X_STANDBY_RC (13 MHz RC oscillator)
	// or RADIOLIB_SX126X_STANDBY_XOSC (32 MHz external crystal oscillator).
	constexpr uint8_t data[] = {RADIOLIB_SX126X_STANDBY_RC};
	return spi::write_stream(RADIOLIB_SX126X_CMD_SET_STANDBY, data, std::size(data));
}

inline Result<Unit, error_t>
reset() {
	gpio::set_mode(RST_PIN, OUTPUT_PP);
	gpio::digital_write(RST_PIN, LOW);
	delay_ms(5);
	gpio::digital_write(RST_PIN, HIGH);
	delay_ms(5);
	const auto instant      = Instant<uint16_t>{};
	constexpr auto INTERVAL = std::chrono::duration<uint16_t, std::milli>{spi::DEFAULT_TIMEOUT_MS};
	decltype(standby()) res = ue_t{error_t{Code::UNKNOWN}};
	while (!res && instant.elapsed() < INTERVAL) {
		res = standby();
		if (res.has_value()) {
			break;
		}
		// don't spam the module
		delay_ms(100);
	}
	if (!res) {
		return res;
	} else {
		return {};
	}
};

inline Result<Unit, error_t>
read_register(const uint16_t addr, uint8_t *data, const uint8_t size) {
	return spi::read_register_burst(addr, data, size);
}

inline Result<Unit, error_t>
write_register(const uint16_t addr, const uint8_t *data, const uint8_t size) {
	return spi::write_register_burst(addr, data, size);
}

Result<Unit, error_t> inline write_buffer(const uint8_t *data, const uint8_t size, const uint8_t offset = 0x00) {
	const uint8_t cmd[] = {RADIOLIB_SX126X_CMD_WRITE_BUFFER, offset};
	return spi::write_stream(cmd, std::size(cmd), data, size);
}

Result<Unit, error_t> inline read_buffer(uint8_t *data, const uint8_t size, const uint8_t offset = 0x00) {
	uint8_t cmd[] = {RADIOLIB_SX126X_CMD_READ_BUFFER, offset};
	return spi::read_stream(cmd, std::size(cmd), data, size);
}

inline Result<uint16_t, error_t>
irq_status() {
	uint8_t data[] = {0x00, 0x00};
	const auto res = spi::read_stream(RADIOLIB_SX126X_CMD_GET_IRQ_STATUS, data, std::size(data));
	APP_RADIO_RETURN_ERR(res);
	return static_cast<uint16_t>(data[0] << 8) | static_cast<uint16_t>(data[1]);
}

inline Result<Unit, error_t>
clear_irq_status(const uint16_t mask = RADIOLIB_SX126X_IRQ_ALL) {
	const uint8_t data[] = {static_cast<uint8_t>((mask >> 8) & 0xff), static_cast<uint8_t>(mask & 0xff)};
	return spi::write_stream(RADIOLIB_SX126X_CMD_CLEAR_IRQ_STATUS, data, std::size(data));
}

/**
 * \brief set RF frequency by writing the raw value to registers
 * \param frf the raw value to write to registers
 * \note don't use this function directly, use `set_frequency` instead
 * \sa set_frequency
 * \sa set_frequency_raw
 */
inline Result<Unit, error_t>
set_rf_frequency(const uint32_t frf) {
	const uint8_t data[] = {static_cast<uint8_t>((frf >> 24) & 0xFF), static_cast<uint8_t>((frf >> 16) & 0xFF), static_cast<uint8_t>((frf >> 8) & 0xFF), static_cast<uint8_t>(frf & 0xFF)};
	return spi::write_stream(RADIOLIB_SX126X_CMD_SET_RF_FREQUENCY, data, std::size(data));
}

inline Result<Unit, error_t>
set_buffer_base_address(const uint8_t txBaseAddress = 0x00, const uint8_t rxBaseAddress = 0x00) {
	const uint8_t data[] = {txBaseAddress, rxBaseAddress};
	return spi::write_stream(RADIOLIB_SX126X_CMD_SET_BUFFER_BASE_ADDRESS, data, std::size(data));
}

inline Result<Unit, error_t>
set_regulator_mode(const uint8_t mode) {
	const uint8_t data[] = {mode};
	return spi::write_stream(RADIOLIB_SX126X_CMD_SET_REGULATOR_MODE, data, std::size(data));
}

inline Result<uint8_t, error_t>
get_status() {
	uint8_t data   = 0;
	const auto res = spi::read_stream(RADIOLIB_SX126X_CMD_GET_STATUS, &data, 1);
	APP_RADIO_RETURN_ERR(res);
	return data;
}

inline Result<uint32_t, error_t>
get_packet_status() {
	uint8_t data[3] = {0, 0, 0};
	const auto res  = spi::read_stream(RADIOLIB_SX126X_CMD_GET_PACKET_STATUS, data, 3);
	APP_RADIO_RETURN_ERR(res);
	return ((static_cast<uint32_t>(data[0]) << 16) | (static_cast<uint32_t>(data[1]) << 8) | static_cast<uint32_t>(data[2]));
}

inline Result<uint16_t, error_t>
get_device_errors() {
	uint8_t data[] = {0x00, 0x00};
	const auto res = spi::read_stream(RADIOLIB_SX126X_CMD_GET_DEVICE_ERRORS, data, std::size(data));
	APP_RADIO_RETURN_ERR(res);
	uint16_t opError = ((static_cast<uint16_t>(data[0]) & 0xFF) << 8) | static_cast<uint16_t>(data[1]);
	return opError;
}

/**
 * \brief This commands clears all the errors recorded in the device. The errors can not be cleared independently.
 */
inline Result<Unit, error_t>
clear_device_errors() {
	constexpr uint8_t data[] = {RADIOLIB_SX126X_CMD_NOP, RADIOLIB_SX126X_CMD_NOP};
	return spi::write_stream(RADIOLIB_SX126X_CMD_CLEAR_DEVICE_ERRORS, data, std::size(data));
}

/**
 * \brief Set the RF frequency in MHz
 * \param freq the frequency to set
 * \sa set_frequency
 *
 * \note \text{value}=\frac{f_{\text{XTAL}}}{2^{25}}\cdot f_{\text{RF}}
 */
inline constexpr uint32_t frequency_raw(const freq_t freq) {
	// 32 MHz crystal oscillator as reference, but the frequency passed in is in MHz
	constexpr uint32_t DIV_EXPONENT          = 25;
	constexpr uint32_t CRYSTAL_FREQUENCY     = 32'000'000;
	constexpr uint32_t CRYSTAL_FREQUENCY_MHZ = CRYSTAL_FREQUENCY / 1'000'000;
	constexpr uint32_t PPL_DIV_FACTOR        = static_cast<uint32_t>(1) << DIV_EXPONENT;
	/**
	 * scale factor is necessary because the calculation will
	 * overflow the integer part if only using fixed point.
	 *
	 * the factor (1048576=0x00100000) is LARGER than uint16_t, so the
	 * fixed point calculation won't work here. Have to do the calculation manually.
	 */
	constexpr auto SCALE_FACTOR = 100;
	const auto fdf              = freq - cnl::floor(freq);
	constexpr auto factor       = static_cast<uint32_t>(1 * static_cast<double>(PPL_DIV_FACTOR) / static_cast<double>(CRYSTAL_FREQUENCY_MHZ));
	constexpr auto factor_s     = static_cast<uint32_t>(1 * static_cast<double>(PPL_DIV_FACTOR) / static_cast<double>(CRYSTAL_FREQUENCY_MHZ) / SCALE_FACTOR);
	const auto freq_integral    = static_cast<uint32_t>(cnl::floor(freq)) * factor;
	const auto freq_decimal     = static_cast<uint32_t>(fdf * SCALE_FACTOR) * factor_s;
	return freq_integral + freq_decimal;
}

/**
 * \brief Set the RF frequency in MHz, with necessary calibration and checks
 * \param freq the frequency to set, which must be in range [150, 960]
 * \param calibrate whether to calibrate the image
 * \sa set_frequency_raw
 */
inline Result<Unit, error_t>
set_frequency(freq_t freq, const bool calibrate = true) {
	using f_t = decltype(freq);
	if (!valid_freq(freq)) {
		return ue_t{error_t{Code::INVALID_FREQUENCY}};
	}
	if (calibrate) {
		uint8_t data[2];
		if (freq > f_t{900}) {
			data[0] = RADIOLIB_SX126X_CAL_IMG_902_MHZ_1;
			data[1] = RADIOLIB_SX126X_CAL_IMG_902_MHZ_2;
		} else if (freq > f_t{850.0}) {
			data[0] = RADIOLIB_SX126X_CAL_IMG_863_MHZ_1;
			data[1] = RADIOLIB_SX126X_CAL_IMG_863_MHZ_2;
		} else if (freq > f_t{770.0}) {
			data[0] = RADIOLIB_SX126X_CAL_IMG_779_MHZ_1;
			data[1] = RADIOLIB_SX126X_CAL_IMG_779_MHZ_2;
		} else if (freq > f_t{460.0}) {
			data[0] = RADIOLIB_SX126X_CAL_IMG_470_MHZ_1;
			data[1] = RADIOLIB_SX126X_CAL_IMG_470_MHZ_2;
		} else {
			data[0] = RADIOLIB_SX126X_CAL_IMG_430_MHZ_1;
			data[1] = RADIOLIB_SX126X_CAL_IMG_430_MHZ_2;
		}
		auto res = spi::write_stream(RADIOLIB_SX126X_CMD_CALIBRATE_IMAGE, data, std::size(data));
		APP_RADIO_RETURN_ERR(res);
		delay_ms(5);
	}
	const auto freq_raw = frequency_raw(freq);
	return set_rf_frequency(freq_raw);
}

inline Result<uint8_t, error_t>
get_packet_type() {
	uint8_t data   = 0;
	const auto res = spi::read_stream(RADIOLIB_SX126X_CMD_GET_PACKET_TYPE, &data, 1);
	APP_RADIO_RETURN_ERR(res);
	return data;
}

/**
 * \brief Some sensitivity degradation may be observed on any LoRa device, when receiving signals transmitted by the LLCC68 with
a LoRa BW of 500 kHz.
 * \note should be used before each packet transmission, to properly configure the chip
 * \param bw the bandwidth
 * \sa SX1262/SX1268 datasheet, chapter 15 Known Limitations, section 15.1 for details
 */
inline Result<Unit, error_t>
fix_sensitivity(const uint8_t bw) {
	uint8_t sensitivityConfig = 0;
	read_register(RADIOLIB_SX126X_REG_SENSITIVITY_CONFIG, &sensitivityConfig, 1);

	// bit 2
	constexpr auto HACK_MASK = 0x04;

	if (bw == RADIOLIB_SX126X_LORA_BW_500_0) {
		sensitivityConfig &= ~HACK_MASK;
	} else {
		sensitivityConfig |= HACK_MASK;
	}
	return write_register(RADIOLIB_SX126X_REG_SENSITIVITY_CONFIG, &sensitivityConfig, 1);
}

inline Result<Unit, error_t>
fix_pa_clamping(const bool enable = true) {
	// fixes overly eager PA clamping
	// see SX1262/SX1268 datasheet, chapter 15 Known Limitations, section 15.2 for details

	// read current clamping configuration

	// The LLCC68 platform embeds a Power Amplifier (PA) clamping mechanism, backing-off the power when over-voltage
	// conditions are detected internally. This method is put in place to protect the internal devices and ensure long-term
	// reliability of the chip. Considering a high-power operation of the LLCC68 (supporting +22dBm on-chip), these “clamping”
	// devices are overly protective, causing the chip to back-down its output power when even a reasonable mismatch is
	// detected at the PA output. The observation is typically 5 to 6 dB less output power than the expected.
	uint8_t clampConfig = 0;
	read_register(RADIOLIB_SX126X_REG_TX_CLAMP_CONFIG, &clampConfig, 1);

	// apply or undo workaround
	if (enable) {
		clampConfig |= 0x1E;
	} else {
		clampConfig = (clampConfig & ~0x1E) | 0x08;
	}

	return write_register(RADIOLIB_SX126X_REG_TX_CLAMP_CONFIG, &clampConfig, 1);
}

/**
 * \brief fixes inverted IQ on SX1262 rev. B
 * \param iq_config RADIOLIB_SX126X_LORA_IQ_STANDARD or RADIOLIB_SX126X_LORA_IQ_INVERTED
 * \see SX1262/SX1268 datasheet, chapter 15 Known Limitations, section 15.4 for details
 */
inline Result<Unit, error_t>
fix_inverted_iq(const uint8_t iq_config) {
	// read current IQ configuration
	uint8_t iqConfig = 0;
	const auto res   = read_register(RADIOLIB_SX126X_REG_IQ_CONFIG, &iqConfig, 1);
	APP_RADIO_RETURN_ERR(res);

	// Bit 2 at address 0x0736 must be set to
	// 0 when using inverted IQ
	// 1 when using standard IQ
	if (iqConfig == RADIOLIB_SX126X_LORA_IQ_INVERTED) {
		iqConfig &= ~0x04;
	} else {
		iqConfig |= 0x04;
	}

	return write_register(RADIOLIB_SX126X_REG_IQ_CONFIG, &iqConfig, 1);
}

struct irq_params_t {
	uint16_t irqMask  = RADIOLIB_SX126X_IRQ_NONE;
	uint16_t dio1Mask = RADIOLIB_SX126X_IRQ_NONE;
	uint16_t dio2Mask = RADIOLIB_SX126X_IRQ_NONE;
	uint16_t dio3Mask = RADIOLIB_SX126X_IRQ_NONE;
};

/**
 * @brief The IrqMask masks or unmasks the IRQ which can be triggered by the device.
 *
 * By default, all IRQ are masked (all ‘0’) and the user can enable them one by one (or several at a time) by setting the corresponding mask to ‘1’.
 */
inline Result<Unit, error_t>
set_dio_irq_params(const irq_params_t &irq_params) {
	const auto irqMask    = irq_params.irqMask;
	const auto dio1Mask   = irq_params.dio1Mask;
	const auto dio2Mask   = irq_params.dio2Mask;
	const auto dio3Mask   = irq_params.dio3Mask;
	const uint8_t data[8] = {static_cast<uint8_t>((irqMask >> 8) & 0xFF), static_cast<uint8_t>(irqMask & 0xFF),
							 static_cast<uint8_t>((dio1Mask >> 8) & 0xFF), static_cast<uint8_t>(dio1Mask & 0xFF),
							 static_cast<uint8_t>((dio2Mask >> 8) & 0xFF), static_cast<uint8_t>(dio2Mask & 0xFF),
							 static_cast<uint8_t>((dio3Mask >> 8) & 0xFF), static_cast<uint8_t>(dio3Mask & 0xFF)};
	return spi::write_stream(RADIOLIB_SX126X_CMD_SET_DIO_IRQ_PARAMS, data, std::size(data));
}

/*!
  \brief Set DIO2 to function as RF switch (default in Semtech example designs).
*/
inline Result<Unit, error_t>
set_dio2_as_rf_switch(const bool en) {
	const uint8_t data = en ? RADIOLIB_SX126X_DIO2_AS_RF_SWITCH : RADIOLIB_SX126X_DIO2_AS_IRQ;
	return spi::write_stream(RADIOLIB_SX126X_CMD_SET_DIO2_AS_RF_SWITCH_CTRL, &data, 1);
}

/**
 * \brief set packet type and do the calibration
 */
inline Result<Unit, error_t> config_packet_type(const parameters &params) {
	auto res = set_buffer_base_address();
	APP_RADIO_RETURN_ERR(res);
	constexpr auto mod = RADIOLIB_SX126X_PACKET_TYPE_LORA;
	uint8_t data[7];
	data[0] = mod;
	res     = spi::write_stream(RADIOLIB_SX126X_CMD_SET_PACKET_TYPE, data, 1);
	APP_RADIO_RETURN_ERR(res);
	data[0] = RADIOLIB_SX126X_RX_TX_FALLBACK_MODE_STDBY_RC;
	res     = spi::write_stream(RADIOLIB_SX126X_CMD_SET_RX_TX_FALLBACK_MODE, data, 1);
	APP_RADIO_RETURN_ERR(res);

	data[0] = RADIOLIB_SX126X_CAD_ON_8_SYMB;
	data[1] = params.sf + 13;
	data[2] = RADIOLIB_SX126X_CAD_PARAM_DET_MIN;
	data[3] = RADIOLIB_SX126X_CAD_GOTO_STDBY;
	data[4] = 0x00;
	data[5] = 0x00;
	data[6] = 0x00;
	res     = spi::write_stream(RADIOLIB_SX126X_CMD_SET_CAD_PARAMS, data, 7);
	APP_RADIO_RETURN_ERR(res);

	clear_irq_status();
	constexpr auto irq_params = irq_params_t{};
	res                       = set_dio_irq_params(irq_params);
	APP_RADIO_RETURN_ERR(res);

	// calibrate all
	data[0] = RADIOLIB_SX126X_CALIBRATE_ALL;
	res     = spi::write_stream(RADIOLIB_SX126X_CMD_CALIBRATE, data, 1);
	APP_RADIO_RETURN_ERR(res);

	// wait for calibration to complete
	delay_ms(5);
	while (gpio::digital_read(DIO1_PIN) == HIGH) {}
	const auto err = spi::llcc68::check_stream();
	if (err != Code::OK) {
		return ue_t{error_t{err}};
	}
	return {};
}

/*!
  \brief Get expected time-on-air for a given size of payload
  \param len Payload length in bytes.
  \returns Expected time-on-air in microseconds.
*/
constexpr uint32_t
calc_time_on_air(const size_t len, const calc_time_on_air_t params) {
	// everything is in microseconds to allow integer arithmetic
	// some constants have .25, these are multiplied by 4, and have _x4 postfix to indicate that fact
	const auto bw_                 = fixed_16_16{bw_khz(params.bw)};
	const auto ubw                 = static_cast<uint32_t>(bw_ * 10);
	const auto sf                  = params.sf;
	const uint32_t symbolLength_us = (static_cast<uint32_t>(1000 * 10) << sf) / ubw;
	uint8_t sfCoeff1_x4            = 17; // (4.25 * 4)
	uint8_t sfCoeff2               = 8;
	if (sf == 5 || sf == 6) {
		sfCoeff1_x4 = 25; // 6.25 * 4
		sfCoeff2    = 0;
	}
	uint8_t sfDivisor = 4 * sf;
	if (symbolLength_us >= 16000) {
		sfDivisor = 4 * (sf - 2);
	}
	constexpr int8_t bitsPerCrc      = 16;
	constexpr int8_t N_symbol_header = params.header_type == RADIOLIB_SX126X_LORA_HEADER_EXPLICIT ? 20 : 0;

	// numerator of equation in section 6.1.4 of SX1268 datasheet v1.1 (might not actually be bitcount, but it has len * 8)
	int16_t bitCount = static_cast<int16_t>(8) * len + params.crc_type * bitsPerCrc - 4 * sf + sfCoeff2 + N_symbol_header;
	if (bitCount < 0) {
		bitCount = 0;
	}
	// add (sfDivisor) - 1 to the numerator to give integer CEIL(...)
	const uint16_t nPreCodedSymbols = (bitCount + (sfDivisor - 1)) / (sfDivisor);

	const auto de = std::get<1>(cr_to_ratio(params.cr));
	// preamble can be 65k, therefore nSymbol_x4 needs to be 32 bit
	const uint32_t nSymbol_x4 = (params.preamble_length + 8) * 4 + sfCoeff1_x4 + nPreCodedSymbols * de * 4;

	return symbolLength_us * nSymbol_x4 / 4;
}

/**
 * \brief
 * \param preamble_length LoRa preamble length in symbols. Allowed values range from 1 to 65535
 * \param payload_length implicit header length; 0xff for explicit header
 * \param crc_type RADIOLIB_SX126X_LORA_CRC_ON or RADIOLIB_SX126X_LORA_CRC_OFF
 * \param hdr_type RADIOLIB_SX126X_LORA_HEADER_EXPLICIT or RADIOLIB_SX126X_LORA_HEADER_IMPLICIT
 * \note  This is for LoRA packet. setModulationParamsFSK (which is for GFSK) won't be ported.
 */
inline Result<Unit, error_t>
set_packet_params(const uint16_t preamble_length = 8,
				  const uint8_t payload_length   = 0xff,
				  const uint8_t crc_type         = RADIOLIB_SX126X_LORA_CRC_ON,
				  const uint8_t hdr_type         = RADIOLIB_SX126X_LORA_HEADER_EXPLICIT) {
	if constexpr (DEFAULT_IQ_TYPE == RADIOLIB_SX126X_LORA_IQ_INVERTED) {
		const auto res = fix_inverted_iq(RADIOLIB_SX126X_LORA_IQ_INVERTED);
		APP_RADIO_RETURN_ERR(res);
	}
	const uint8_t data[] = {static_cast<uint8_t>((preamble_length >> 8) & 0xff),
							static_cast<uint8_t>(preamble_length & 0xff),
							crc_type, payload_length,
							hdr_type, DEFAULT_IQ_TYPE};
	return spi::write_stream(RADIOLIB_SX126X_CMD_SET_PACKET_PARAMS, data, std::size(data));
}

/**
 * \brief frequency synthesizer calibration
 */
inline Result<Unit, error_t> fs() {
	return spi::write_stream(RADIOLIB_SX126X_CMD_SET_FS, nullptr, 0);
}

/**
 * \param timeout no idea why you need a timeout for TX
 */
inline Result<Unit, error_t> tx(const uint32_t timeout = 0) {
	const uint8_t data[] = {static_cast<uint8_t>((timeout >> 16) & 0xFF), static_cast<uint8_t>((timeout >> 8) & 0xFF), static_cast<uint8_t>(timeout & 0xFF)};
	return spi::write_stream(RADIOLIB_SX126X_CMD_SET_TX, data, std::size(data));
}

/**
  \brief Interrupt-driven receive method. DIO1 will be activated when full packet is received.
  \param timeout Receive mode type and/or raw timeout value, expressed as multiples of 15.625 us.

  \note When set to RADIOLIB_SX126X_RX_TIMEOUT_INF, the timeout will be infinite and the device will remain
  in Rx mode until explicitly commanded to stop (Rx continuous mode).
  When set to RADIOLIB_SX126X_RX_TIMEOUT_NONE, there will be no timeout and the device will return
  to standby when a packet is received (Rx single mode).
  For any other value, timeout will be applied and signal will be generated on DIO1 for conditions
  defined by irqFlags and irqMask.
 */
inline Result<Unit, error_t> rx(const uint32_t timeout = RADIOLIB_SX126X_RX_TIMEOUT_INF) {
	const uint8_t data[] = {static_cast<uint8_t>((timeout >> 16) & 0xFF), static_cast<uint8_t>((timeout >> 8) & 0xFF), static_cast<uint8_t>(timeout & 0xFF)};
	return spi::write_stream(RADIOLIB_SX126X_CMD_SET_RX, data, std::size(data));
}

/**
 * \brief Channel Activity Detection (CAD) method
 */
inline Result<Unit, error_t>
set_cad_mode(const uint8_t sf,
			 const uint8_t symbol_num = RADIOLIB_SX126X_CAD_PARAM_DEFAULT,
			 const uint8_t det_peak   = RADIOLIB_SX126X_CAD_PARAM_DEFAULT,
			 const uint8_t det_min    = RADIOLIB_SX126X_CAD_PARAM_DEFAULT) {
	const uint8_t detPeakValues[8]   = {22, 22, 22, 22, 23, 24, 25, 28};
	const uint8_t symbolNumValues[8] = {RADIOLIB_SX126X_CAD_ON_2_SYMB,
										RADIOLIB_SX126X_CAD_ON_2_SYMB,
										RADIOLIB_SX126X_CAD_ON_2_SYMB,
										RADIOLIB_SX126X_CAD_ON_2_SYMB,
										RADIOLIB_SX126X_CAD_ON_4_SYMB,
										RADIOLIB_SX126X_CAD_ON_4_SYMB,
										RADIOLIB_SX126X_CAD_ON_4_SYMB,
										RADIOLIB_SX126X_CAD_ON_4_SYMB};
	// build the packet
	uint8_t data[7];
	data[0] = symbolNumValues[sf - 5];
	data[1] = detPeakValues[sf - 5];
	data[2] = RADIOLIB_SX126X_CAD_PARAM_DET_MIN;
	data[3] = RADIOLIB_SX126X_CAD_GOTO_STDBY;
	data[4] = 0x00;
	data[5] = 0x00;
	data[6] = 0x00;
	if (symbol_num != RADIOLIB_SX126X_CAD_PARAM_DEFAULT) {
		data[0] = symbol_num;
	}
	if (det_peak != RADIOLIB_SX126X_CAD_PARAM_DEFAULT) {
		data[1] = det_peak;
	}
	if (det_min != RADIOLIB_SX126X_CAD_PARAM_DEFAULT) {
		data[2] = det_min;
	}
	auto res = spi::write_stream(RADIOLIB_SX126X_CMD_SET_CAD_PARAMS, data, std::size(data));
	APP_RADIO_RETURN_ERR(res);
	return spi::write_stream(RADIOLIB_SX126X_CMD_SET_CAD, nullptr, 0);
}

inline Result<Unit, error_t>
set_pa_config(const uint8_t pa_duty_cycle,
			  const uint8_t device_sel,
			  const uint8_t hp_max = RADIOLIB_SX126X_PA_CONFIG_HP_MAX,
			  const uint8_t pa_lut = RADIOLIB_SX126X_PA_CONFIG_PA_LUT) {
	const uint8_t data[] = {pa_duty_cycle, hp_max, device_sel, pa_lut};
	return spi::write_stream(RADIOLIB_SX126X_CMD_SET_PA_CONFIG, data, std::size(data));
}

/**
 * \brief get status from IRQ status register
 * \return true if channel is free, false if channel is busy, error otherwise
 */
inline Result<bool, error_t>
channel_scan_result() {
	const auto res = irq_status();
	APP_RADIO_RETURN_ERR(res);
	if (*res & RADIOLIB_SX126X_IRQ_CAD_DETECTED) {
		return false;
	} else if (*res & RADIOLIB_SX126X_IRQ_CAD_DONE) {
		return true;
	}
	return ue_t{error_t{Code::INVALID_CAD_RESULT}};
}

/**
 * \brief blocking CAD
 * \param sf spreading factor
 * \param timeout_ms timeout in milliseconds
 * \param symbol_num number of symbols to scan
 * \param det_peak peak detection threshold
 * \param det_min minimum detection threshold
 * \sa set_cad_mode
 */
inline Result<bool, error_t>
sync_scan_channel(
	const uint8_t sf,
	const uint32_t timeout_ms = 100,
	const uint8_t symbol_num  = RADIOLIB_SX126X_CAD_PARAM_DEFAULT,
	const uint8_t det_peak    = RADIOLIB_SX126X_CAD_PARAM_DEFAULT,
	const uint8_t det_min     = RADIOLIB_SX126X_CAD_PARAM_DEFAULT) {
	auto res = standby();
	APP_RADIO_RETURN_ERR(res);

	const auto instant = Instant<uint32_t>{};
	res                = set_cad_mode(sf, symbol_num, det_peak, det_min);
	APP_RADIO_RETURN_ERR(res);

	auto ok_ = channel_scan_result();
	while (!ok_ and !*ok_ and instant.has_elapsed_ms(timeout_ms)) {
		constexpr auto DELAY_INTERVAL_MS = 5;
		delay_ms(DELAY_INTERVAL_MS);
		ok_ = channel_scan_result();
	}

	if (!ok_) {
		return ue_t{error_t{ok_.error()}};
	}
	return *ok_;
}

template <float target = 0.0f>
bool __in_precision(const s_fixed_16_16 v) {
	using v_t                = decltype(v);
	constexpr auto precision = s_fixed_16_16{0.001};
	return v_t{abs(v - v_t{target})} <= precision;
}

/*!
  \brief Get one truly random byte from RSSI noise.
  \returns TRNG byte.
*/
inline uint8_t random_byte() {
	spi::set_register_value(RADIOLIB_SX126X_REG_ANA_LNA, RADIOLIB_SX126X_LNA_RNG_ENABLED, 0, 0);
	spi::set_register_value(RADIOLIB_SX126X_REG_ANA_MIXER, RADIOLIB_SX126X_MIXER_RNG_ENABLED, 0, 0);

	rx(RADIOLIB_SX126X_RX_TIMEOUT_INF);
	delay_ms(10);

	uint8_t rand = 0x00;
	read_register(RADIOLIB_SX126X_REG_RANDOM_NUMBER_0, &rand, 1);
	standby();

	spi::set_register_value(RADIOLIB_SX126X_REG_ANA_LNA, RADIOLIB_SX126X_LNA_RNG_DISABLED, 0, 0);
	spi::set_register_value(RADIOLIB_SX126X_REG_ANA_MIXER, RADIOLIB_SX126X_MIXER_RNG_DISABLED, 0, 0);
	return rand;
}

/*!
  \brief Sets TCXO (Temperature Compensated Crystal Oscillator) configuration.
  \param voltage TCXO reference voltage in volts. Allowed values are 1.6, 1.7, 1.8, 2.2. 2.4, 2.7, 3.0 and 3.3 V.
  Set to 0 to disable TCXO.
  NOTE: After setting this parameter to 0, the module will be reset (since there's no other way to disable TCXO).

  \param delay TCXO timeout in us. Defaults to 5000 us.
  \param XTAL Set to true to use XTAL instead of TCXO. Defaults to false.

  \note will return immediately if XTAL is true; DIO3_AS_TCXO_CTRL;
*/
inline Result<uint32_t, error_t>
set_TCXO(const s_fixed_16_16 voltage, const uint32_t delay = 5000, const bool XTAL = false) {
	if (XTAL) {
		return ue_t{error_t{Code::INVALID_TCXO_VOLTAGE}};
	}
	auto res = standby();
	APP_RADIO_RETURN_ERR(res);
	auto err_ = get_device_errors();
	APP_RADIO_RETURN_ERR(res);
	if (*err_ & RADIOLIB_SX126X_XOSC_START_ERR) {
		clear_device_errors();
	}

	if (__in_precision(voltage)) {
		reset();
		return {};
	}

	uint8_t data[4];
	if (__in_precision<1.6f>(voltage)) {
		data[0] = RADIOLIB_SX126X_DIO3_OUTPUT_1_6;
	} else if (__in_precision<1.7f>(voltage)) {
		data[0] = RADIOLIB_SX126X_DIO3_OUTPUT_1_7;
	} else if (__in_precision<1.8f>(voltage)) {
		data[0] = RADIOLIB_SX126X_DIO3_OUTPUT_1_8;
	} else if (__in_precision<2.2f>(voltage)) {
		data[0] = RADIOLIB_SX126X_DIO3_OUTPUT_2_2;
	} else if (__in_precision<2.4f>(voltage)) {
		data[0] = RADIOLIB_SX126X_DIO3_OUTPUT_2_4;
	} else if (__in_precision<2.7f>(voltage)) {
		data[0] = RADIOLIB_SX126X_DIO3_OUTPUT_2_7;
	} else if (__in_precision<3.0f>(voltage)) {
		data[0] = RADIOLIB_SX126X_DIO3_OUTPUT_3_0;
	} else if (__in_precision<3.3f>(voltage)) {
		data[0] = RADIOLIB_SX126X_DIO3_OUTPUT_3_3;
	} else {
		return ue_t{error_t{Code::INVALID_TCXO_VOLTAGE}};
	}

	uint32_t delay_val = static_cast<uint32_t>(fixed_16_16{delay} / fixed_16_16{15.625});
	data[1]            = static_cast<uint8_t>((delay_val >> 16) & 0xFF);
	data[2]            = static_cast<uint8_t>((delay_val >> 8) & 0xFF);
	data[3]            = static_cast<uint8_t>(delay_val & 0xFF);

	res = spi::write_stream(RADIOLIB_SX126X_CMD_SET_DIO3_AS_TCXO_CTRL, data, std::size(data));
	APP_RADIO_RETURN_ERR(res);

	return {delay_val};
}

/**
 * \brief if the symbol (length := 2^sf / bw_khz) > 16, then we're using LDR
 */
template <uint8_t bw, uint32_t sf>
bool static_is_use_ldr() {
	constexpr auto khz           = bw_khz(bw);
	constexpr auto symbol_length = static_cast<uint32_t>(std::pow(2, sf) / khz);
	return symbol_length > 16;
}

/**
 * \brief set LoRa modulation parameters
 * \param sf spread factor (5-12)
 * \param bw RAW bandwidth (RADIOLIB_SX126X_LORA_BW_*)
 * \param cr coding rate (RADIOLIB_SX126X_LORA_CR_*)
 * \param ldro RADIOLIB_SX126X_LORA_LOW_DATA_RATE_OPTIMIZE_ON or RADIOLIB_SX126X_LORA_LOW_DATA_RATE_OPTIMIZE_OFF
 */
inline Result<Unit, error_t>
set_modulation_params(const uint8_t sf,
					  const uint8_t bw   = RADIOLIB_SX126X_LORA_BW_500_0,
					  const uint8_t cr   = RADIOLIB_SX126X_LORA_CR_4_5,
					  const uint8_t ldro = RADIOLIB_SX126X_LORA_LOW_DATA_RATE_OPTIMIZE_OFF) {
	if (not valid_bw(bw)) {
		return ue_t{error_t{Code::INVALID_BANDWIDTH}};
	}
	if (not valid_sf(bw, sf)) {
		return ue_t{error_t{Code::INVALID_SPREADING_FACTOR}};
	}
	if (not valid_cr(cr)) {
		return ue_t{error_t{Code::INVALID_CODING_RATE}};
	}

	const uint8_t data[] = {sf, bw, cr, ldro};
	return spi::write_stream(RADIOLIB_SX126X_CMD_SET_MODULATION_PARAMS, data, std::size(data));
}

inline Result<Unit, error_t>
set_tx_params(const uint8_t pwr, const uint8_t ramp_time = RADIOLIB_SX126X_PA_RAMP_200U) {
	const uint8_t data[] = {pwr, ramp_time};
	return spi::write_stream(RADIOLIB_SX126X_CMD_SET_TX_PARAMS, data, std::size(data));
}

/*!
  \brief Sets LoRa sync word.
  \param sync_word LoRa sync word to be set.
  \param control_bits Undocumented control bits, required for compatibility purposes.
*/
inline Result<Unit, error_t> set_sync_word(const uint8_t sync_word = RADIOLIB_SX126X_SYNC_WORD_PRIVATE, const uint8_t control_bits = 0x44) {
	const auto pkt_type = get_packet_type();
	APP_RADIO_RETURN_ERR(pkt_type);
	if (*pkt_type != RADIOLIB_SX126X_PACKET_TYPE_LORA) {
		return ue_t{error_t{Code::WRONG_MODERN}};
	}
	const uint8_t data[2] = {static_cast<uint8_t>((sync_word & 0xF0) | ((control_bits & 0xF0) >> 4)),
							 static_cast<uint8_t>(((sync_word & 0x0F) << 4) | (control_bits & 0x0F))};
	return write_register(RADIOLIB_SX126X_REG_LORA_SYNC_WORD_MSB, data, std::size(data));
}

inline Result<Unit, error_t>
set_output_power(const int8_t power) {
	if (!in_range(power, -9, 22)) {
		return ue_t{error_t{Code::INVALID_OUTPUT_POWER}};
	}
	uint8_t ocp = 0;
	auto res    = read_register(RADIOLIB_SX126X_REG_OCP_CONFIGURATION, &ocp, 1);
	APP_RADIO_RETURN_ERR(res);

	res = set_pa_config(0x04, RADIOLIB_SX126X_PA_CONFIG_SX1262);
	APP_RADIO_RETURN_ERR(res);
	res = set_tx_params(power);
	APP_RADIO_RETURN_ERR(res);
	// restore OCP configuration
	return write_register(RADIOLIB_SX126X_REG_OCP_CONFIGURATION, &ocp, 1);
}

inline Result<Unit, error_t>
set_regulator_ldo() {
	return set_regulator_mode(RADIOLIB_SX126X_REGULATOR_LDO);
}

inline Result<Unit, error_t>
set_regulator_dc_dc() {
	return set_regulator_mode(RADIOLIB_SX126X_REGULATOR_DC_DC);
}

/**
 * \brief checks if BUSY pin is low
 * \return true if BUSY pin is low, false otherwise
 */
inline bool busy_low() {
	return gpio::digital_read(spi::BUSY_PIN) == LOW;
}

/**
 * \brief blocks until BUSY pin is low
 */
inline void until_busy_low() {
	while (!busy_low()) {}
}

template <float limit>
Result<Unit, error_t>
set_current_limit() {
	static_assert(in_range(limit, 0, 140), "limit must be in range [0, 140]");
	constexpr uint8_t raw = static_cast<uint8_t>(limit / 2.5);
	return write_register(RADIOLIB_SX126X_REG_OCP_CONFIGURATION, &raw, 1);
}

inline bool find_chip() {
	uint8_t i = 0;
	bool ok   = false;
	while ((i < 10) && !ok) {
		constexpr auto LLCC86_CHIP_TYPE = "LLCC68";
		constexpr auto SX1261_CHIP_TYPE = "SX1261";
		reset();
		char version[16]{};
		spi::read_register_burst(RADIOLIB_SX126X_REG_VERSION_STRING, reinterpret_cast<uint8_t *>(version), std::size(version));
		ok = strncmp(version, LLCC86_CHIP_TYPE, 6) == 0;
		if (!ok) {
			ok = strncmp(version, SX1261_CHIP_TYPE, 6) == 0;
		}
		delay_ms(10);
		i++;
	}
	return ok;
}

inline Result<uint8_t, error_t>
packet_length() {
	uint8_t rx_buf_status[] = {0, 0};
	auto res                = spi::read_stream(RADIOLIB_SX126X_CMD_GET_RX_BUFFER_STATUS, rx_buf_status, std::size(rx_buf_status));
	APP_RADIO_RETURN_ERR(res);
	return rx_buf_status[0];
}

/**
 * \brief read data from buffer
 * \param data the buffer to read into
 * \param len the container/buffer size
 * \return the length of data read
 */
inline Result<uint8_t, error_t>
read_data(uint8_t *data, size_t len) {
	const auto irq_ = irq_status();
	APP_RADIO_RETURN_ERR(irq_);
	const auto st  = spi::llcc68::check_stream();
	const auto irq = std::move(*irq_);
	if (irq & RADIOLIB_SX126X_IRQ_TIMEOUT && st == Code::SPI_CMD_TIMEOUT) {
		return ue_t{error_t{Code::RX_TIMEOUT}};
	}

	const auto sz_ = packet_length();
	APP_RADIO_RETURN_ERR(sz_);
	const auto sz = std::move(*sz_);
	if (sz > len) {
		return ue_t{error_t{Code::PACKET_TOO_LONG}};
	}
	auto res = read_buffer(data, sz);
	APP_RADIO_RETURN_ERR(res);

	res = set_buffer_base_address();
	APP_RADIO_RETURN_ERR(res);

	res = clear_irq_status();
	APP_RADIO_RETURN_ERR(res);

	if ((irq & RADIOLIB_SX126X_IRQ_CRC_ERR) || (irq & RADIOLIB_SX126X_IRQ_HEADER_ERR)) {
		return ue_t{error_t{Code::CRC_MISMATCH}};
	}

	return {sz};
}

/**
  Circuit Configuration for Basic Rx Operation

  After power up (battery insertion or hard reset) the chip run automatically a calibration procedure and goes to STDBY_RC
  mode. This is indicated by a low state on BUSY pin. From this state the steps are:
  1. If not in STDBY_RC mode, then set the circuit in this mode with the command SetStandby()
  2. Define the protocol (LoRa or FSK) with the command SetPacketType(...)
  3. Define the RF frequency with the command SetRfFrequency(...)
  4. Define where the data will be stored inside the data buffer in Rx with the command SetBufferBaseAddress(...)
  5. Define the modulation parameter according to the chosen protocol with the command SetModulationParams(...)1
  6. Define the frame format to be used with the command SetPacketParams(...)
  7. Configure DIO and IRQ: use the command SetDioIrqParams(...) to select the IRQ RxDone and map this IRQ to a DIO (DIO1 or DIO2 or DIO3), set IRQ Timeout as well.
  8. Define Sync Word value: use the command WriteReg(...) to write the value of the register via direct register access.
  9. Set the circuit in reception mode: use the command SetRx(). Set the parameter to enable timeout or continuous mode
  10. Wait for IRQ RxDone2 or Timeout: the chip will stay in Rx and look for a new packet if the continuous mode is selected
  otherwise it will goes to STDBY_RC mode.
  11. In case of the IRQ RxDone, check the status to ensure CRC is correct: use the command GetIrqStatus()
  12. Clear IRQ flag RxDone or Timeout: use the command ClearIrqStatus(). In case of a valid packet (CRC OK), get the packet
  length and address of the first byte of the received payload by using the command GetRxBufferStatus(...)
  13. In case of a valid packet (CRC OK), start reading the packet

  \note The IRQ RxDone means that a packet has been received but the CRC could be wrong: the user must check the CRC before validating the packet.
 */
inline void kick_inf_rx() {
	constexpr auto irq_params = irq_params_t{
		RADIOLIB_SX126X_IRQ_RX_DEFAULT,
		RADIOLIB_SX126X_IRQ_RX_DONE,
		RADIOLIB_SX126X_IRQ_RX_DONE,
	};
	set_dio_irq_params(irq_params);
	standby();
	rx(RADIOLIB_SX126X_RX_TIMEOUT_INF);
}

/**
 * \brief will be called after each `sync_transmit`
 * \note Application writer (i.e. myself) should manually enable the receiver after transmission
 */
inline void after_transmit() {
	clear_irq_status();
	kick_inf_rx();
}

/**
  After power up (battery insertion or hard reset) the chip runs automatically a calibration procedure and goes to STDBY_RC
  mode. This is indicated by a low state on BUSY pin. From this state the steps are:
  1. If not in STDBY_RC mode, then go to this mode with the command SetStandby(...)
  2. Define the protocol (LoRa or FSK) with the command SetPacketType(...)
  3. Define the RF frequency with the command SetRfFrequency(...)
  4. Define the Power Amplifier configuration with the command SetPaConfig(...)
  5. Define output power and ramping time with the command SetTxParams(...)
  6. Define where the data payload will be stored with the command SetBufferBaseAddress(...)
  7. Send the payload to the data buffer with the command WriteBuffer(...)
  8. Define the modulation parameter according to the chosen protocol with the command SetModulationParams(...)1
  9. Define the frame format to be used with the command SetPacketParams(...)2
  10. Configure DIO and IRQ: use the command SetDioIrqParams(...) to select TxDone IRQ and map this IRQ to a DIO (DIO1, DIO2 or DIO3)
  11. Define Sync Word value: use the command WriteReg(...) to write the value of the register via direct register access
  12. Set the circuit in transmitter mode to start transmission with the command SetTx(). Use the parameter to enable Timeout
  13. Wait for the IRQ TxDone or Timeout: once the packet has been sent the chip goes automatically to STDBY_RC mode
  14. Clear the IRQ TxDone flag
 */
inline Result<Unit, error_t>
sync_transmit(const uint8_t *data, const size_t len, const calc_time_on_air_t &params) {
	if (len > RADIOLIB_SX126X_MAX_PACKET_LENGTH) {
		return ue_t{error_t{Code::PACKET_TOO_LONG}};
	}

	decltype(standby()) res;
	res = standby();
	APP_RADIO_RETURN_ERR(res);

	res = set_packet_params(params.preamble_length, len, params.crc_type, params.header_type);
	APP_RADIO_RETURN_ERR(res);

	res = set_buffer_base_address();
	APP_RADIO_RETURN_ERR(res);

	res = write_buffer(data, len);
	APP_RADIO_RETURN_ERR(res);

	res = fix_sensitivity(params.bw);
	APP_RADIO_RETURN_ERR(res);

	constexpr auto irq_params = irq_params_t{
		RADIOLIB_SX126X_IRQ_TX_DONE | RADIOLIB_SX126X_IRQ_TIMEOUT,
		RADIOLIB_SX126X_IRQ_TX_DONE,
		RADIOLIB_SX126X_IRQ_TX_DONE,
	};
	res = set_dio_irq_params(irq_params);
	APP_RADIO_RETURN_ERR(res);
	clear_irq_status();
	APP_RADIO_RETURN_ERR(res);

	res = tx();
	APP_RADIO_RETURN_ERR(res);

	const auto instant    = Instant<uint16_t>{};
	const auto timeout_us = calc_time_on_air(len, params) * 11u / 10u;
	const auto timeout_ms = timeout_us / 1000;

	until_busy_low();

	// wait for IRQ or timeout
	while (not fetch_flag_reset() and instant.has_elapsed_ms(timeout_ms)) {}

	after_transmit();
	return {};
}

inline Result<Unit, error_t>
sync_transmit(const char *data, const calc_time_on_air_t &params) {
	const auto sz = std::strlen(data);
	return sync_transmit(reinterpret_cast<const uint8_t *>(data), sz, params);
}

/**
 * \brief async transmit
 * \param data the data to transmit
 * \param len the length of data
 * \param params the parameters for calculating time on air
 * \param tx_state current transmit state
 * \return new transmit state
 * \see poll_tx_state
 */
inline Result<transmit_state_t, error_t>
async_transmit(const uint8_t *data, const size_t len,
			   const calc_time_on_air_t &params,
			   const transmit_state_t &tx_state) {
	if (tx_state.is_transmitting) {
		return ue_t{error_t{Code::BUSY_TX}};
	}
	if (len > RADIOLIB_SX126X_MAX_PACKET_LENGTH) {
		return ue_t{error_t{Code::PACKET_TOO_LONG}};
	}

	decltype(standby()) res;
	res = standby();
	APP_RADIO_RETURN_ERR(res);

	res = set_packet_params(params.preamble_length, len, params.crc_type, params.header_type);
	APP_RADIO_RETURN_ERR(res);

	res = set_buffer_base_address();
	APP_RADIO_RETURN_ERR(res);

	res = write_buffer(data, len);
	APP_RADIO_RETURN_ERR(res);

	res = fix_sensitivity(params.bw);
	APP_RADIO_RETURN_ERR(res);

	constexpr auto irq_params = irq_params_t{
		RADIOLIB_SX126X_IRQ_TX_DONE | RADIOLIB_SX126X_IRQ_TIMEOUT,
		RADIOLIB_SX126X_IRQ_TX_DONE,
		RADIOLIB_SX126X_IRQ_TX_DONE,
	};
	res = set_dio_irq_params(irq_params);
	APP_RADIO_RETURN_ERR(res);
	clear_irq_status();
	APP_RADIO_RETURN_ERR(res);

	res = tx();
	APP_RADIO_RETURN_ERR(res);

	const auto timeout_us = calc_time_on_air(len, params) * 11u / 10u;
	const auto timeout_ms = timeout_us / 1000;
	auto instant          = Instant<uint16_t>{};
	return transmit_state_t{
		.is_transmitting      = true,
		.start                = std::move(instant),
		.expected_duration_ms = timeout_ms,
	};
}

/**
 * \brief polling transmit state
 * \param tx_state current transmit state
 * \return new transmit state
 * \see async_transmit
 */
inline transmit_state_t
poll_tx_state(const transmit_state_t tx_state) {
	if (not tx_state.is_transmitting) {
		return tx_state;
	}

	if (not busy_low()) {
		return tx_state;
	}

	if (tx_state.start.has_elapsed_ms(tx_state.expected_duration_ms)) {
		after_transmit();
		return transmit_state_t{false};
	}
	return tx_state;
}

static constexpr auto init_pins = [] {
	gpio::set_mode(RST_PIN, OUTPUT_PP);
	config_exti();
};

static constexpr auto begin = [](const parameters &params) -> Result<Unit, error_t> {
	/**
	 * Most of the commands can be sent in any order except for the radio configuration commands which will set the radio in
	 * the proper operating mode. Indeed, it is mandatory to set the radio protocol using the command SetPacketType(...) as a first
	 * step before issuing any other radio configuration commands. In a second step, the user should define the modulation
	 * parameter according to the chosen protocol with the command SetModulationParams(...). Finally, the user should then
	 * select the packet format with the command SetPacketParams(...).
	 *
	 * \note
	 * If this order is not respected, the behavior of the device could be unexpected.
	 **/
	decltype(standby()) res;
	res = reset();
	APP_RADIO_RETURN_ERR(res);

	res = standby();
	APP_RADIO_RETURN_ERR(res);

	if (!find_chip()) {
		return ue_t{error_t{Code::CHIP_NOT_FOUND}};
	}

	constexpr auto tcxo_voltage = DEFAULT_TCXO_VOLTAGE;
	if (!USE_XTAL && tcxo_voltage > 0.0f) {
		const auto voltage_     = set_TCXO(tcxo_voltage);
		details::__tcxo_delay__ = *voltage_;
		APP_RADIO_RETURN_ERR(res);
	}

	// SetPacketType
	res = config_packet_type(params);
	APP_RADIO_RETURN_ERR(res);

	// SetModulationParams
	res = set_modulation_params(params.sf,
								params.bw,
								params.cr,
								params.ldr_optimize);
	APP_RADIO_RETURN_ERR(res);

	res = set_sync_word(params.sync_word);
	APP_RADIO_RETURN_ERR(res);

	if constexpr (USE_REGULATOR_LDO) {
		res = set_regulator_ldo();
		APP_RADIO_RETURN_ERR(res);
	} else {
		res = set_regulator_dc_dc();
		APP_RADIO_RETURN_ERR(res);
	}

	res = set_current_limit<60.0f>();
	APP_RADIO_RETURN_ERR(res);

	res = set_dio2_as_rf_switch(false);
	APP_RADIO_RETURN_ERR(res);

	res = set_frequency(params.frequency);
	APP_RADIO_RETURN_ERR(res);

	// SetPacketParams
	res = set_packet_params(params.preamble_len,
							0x00,
							DEFAULT_CRC_TYPE,
							DEFAULT_HEADER_TYPE);
	APP_RADIO_RETURN_ERR(res);

	res = set_output_power(22);
	APP_RADIO_RETURN_ERR(res);

	res = fix_pa_clamping();
	APP_RADIO_RETURN_ERR(res);

	res = standby();
	APP_RADIO_RETURN_ERR(res);
	return {};
};
}

#undef APP_RADIO_RETURN_ERR
#endif // PWR_MGR_LLCC68_H
