#ifndef EE69DC8E_8AA5_4DAB_AB81_FC369B8A90CB
#define EE69DC8E_8AA5_4DAB_AB81_FC369B8A90CB

#include <cstdint>

namespace max {

// select the device operation mode
constexpr uint8_t FMY_DEV_MODE_W = 0x01;
constexpr uint8_t IDX_DEV_MODE_W = 0x00;
enum DevModeW : uint8_t {
	DEV_MODE_W_EXIT_BL  = 0x00,
	DEV_MODE_W_SHUTDOWN = 0x01,
	DEV_MODE_W_RST      = 0x02,
	DEV_MODE_W_ENTER_BL = 0x08,
};

// read the device operation mode
constexpr uint8_t FMY_DEV_MODE_R = 0x02;
constexpr uint8_t IDX_DEV_MODE_R = 0x00;
enum DevModeR : uint8_t {
	DEV_MODE_R_APP = 0x00,
	DEV_MODE_R_BL  = 0x08,
};

// https://github.com/analogdevicesinc/MAX32630FTHR_msbl_flasher/blob/52cafd7d589991550876703d88ccfc902a4badbd/api.py#L47-L60
enum STATUS_CODE : uint8_t {
	// The write transaction was successful.
	SUCCESS = 0x00,
	/// Illegal Family Byte and/or Index Byte was used.
	///
	/// - Verify that the Family Byte, Index Byte are valid for the host command
	/// sent.
	/// - Verify that the latest .msbl is flashed.
	ERR_UNAVAIL_CMD = 0x01,
	/// This function is not implemented. Verify that the Index Byte and Write
	/// Byte(s) are valid for the host command sent.
	ERR_UNAVAIL_FUNC = 0x02,
	/// Incorrect number of bytes sent for the requested Family Byte. Verify that
	/// the correct number of bytes are sent for the host command.
	ERR_DATA_FORMAT = 0x03,
	/// Illegal configuration value was attempted to be set.
	///
	/// - Verify that the Index Byte is correct for Family Byte 0x44.
	/// - Verify that the report period is not 0 for host command 0x10 0x02.
	/// - Verify that the Write byte for host command 0x10 0x03 is in the valid
	/// range specified.
	ERR_INPUT_VALUE = 0x04,
	/// Not used in application mode.
	ERR_INVALID_MODE = 0x05,
	/// General error while receiving/flashing a page during the bootloader sequence. Not used.
	ERR_BTLDR_GENERAL = 0x80,
	/// Bootloader checksum error while decrypting/checking page data. Verify
	/// that the keyed .msbl file is compatible with MAX32664A/B/C/D.
	ERR_BTLDR_CHECKSUM = 0x81,
	/// Bootloader authorization error. Verify that the keyed .msbl file is compatible with MAX32664A/B/C/D.
	ERR_BTLDR_AUTH = 0x82,
	/// Bootloader detected that the application is not valid.
	ERR_BTLDR_INVALID_APP = 0x83,
	/// Device is busy, try again. Increase the delay before the command and increase the CMD_DELAY.
	ERR_TRY_AGAIN = 0xFE,
	/// Unknown Error. Verify that the communications to the AFE/KX-122 are
	/// correct by reading the PART_ID/WHO_AM_I register. For MAX32664B/C, the
	/// MAX32664 is in deep sleep unless the host sets the MFIO pin low 250μs
	/// before and during the I2C communications.
	ERR_UNKNOWN = 0xFF,
	/// Device is busy. Insert delay and resend the host command
	ERR_BTLDR_TRY_AGAIN = 0x05,
};

enum class FIFO_OUTPUT_MODE : uint8_t {
	PAUSE           = 0x00, // No data output
	SENSOR_DATA     = 0x01, // Raw sensor data only
	ALGORITHM_DATA  = 0x02, // Processed algorithm data only
	SENSOR_AND_ALGO = 0x03, // Both sensor and algorithm data
	PAUSE_ALT       = 0x04, // Alternative pause mode (no data)
	COUNTER_SENSOR  = 0x05, // Sample counter byte + sensor data
	COUNTER_ALGO    = 0x06  // Sample counter byte + algorithm data
};

// belongs to the 0x50 0x07 command family
constexpr uint8_t PREFIX_ALGO_RUN_MODE = 0x0A;
enum class ALGO_OP_MODE : uint8_t {
	CONTINUOUS_HRM_CONTINUOUS_SPO2 = 0x00, // Continuous HRM, continuous SpO2
	CONTINUOUS_HRM_ONE_SHOT_SPO2   = 0x01, // Continuous HRM, one-shot SpO2
	CONTINUOUS_HRM                 = 0x02, // Continuous HRM
	SAMPLED_HRM                    = 0x03, // Sampled HRM
	SAMPLED_HRM_ONE_SHOT_SPO2      = 0x04, // Sampled HRM, one-shot SpO2
	ACTIVITY_TRACKING_ONLY         = 0x05, // Activity tracking only
	SPO2_CALIBRATION               = 0x06, // SpO2 calibration
};

enum class ACTIVATE_CLASS : uint8_t {
	REST,
	WALK,
	RUN,
	BIKE,
};

enum class SPO2_STATE : uint8_t {
	LED_ADJUSTMENT,
	COMPUTATION,
	SUCCESS,
	TIMEOUT,
};

// Skin contact detector (SCD)
enum class SCD_STATE : uint8_t {
	UNDETECTED,
	OFF_SKIN,
	ON_SOME_SUBJECT,
	ON_SKIN,
};

inline const char *activate_class_to_string(ACTIVATE_CLASS c) {
	switch (c) {
	case ACTIVATE_CLASS::REST:
		return "REST";
	case ACTIVATE_CLASS::WALK:
		return "WALK";
	case ACTIVATE_CLASS::RUN:
		return "RUN";
	case ACTIVATE_CLASS::BIKE:
		return "BIKE";
	default:
		return "UNKNOWN";
	}
}

inline const char *scd_state_to_string(SCD_STATE state) {
	switch (state) {
	case SCD_STATE::UNDETECTED:
		return "UNDETECTED";
	case SCD_STATE::OFF_SKIN:
		return "OFF_SKIN";
	case SCD_STATE::ON_SOME_SUBJECT:
		return "ON_SOME_SUBJECT";
	case SCD_STATE::ON_SKIN:
		return "ON_SKIN";
	default:
		return "UNKNOWN";
	};
}

inline const char *spo2_state_to_string(SPO2_STATE state) {
	switch (state) {
	case SPO2_STATE::LED_ADJUSTMENT:
		return "LED_ADJUSTMENT";
	case SPO2_STATE::COMPUTATION:
		return "COMPUTATION";
	case SPO2_STATE::SUCCESS:
		return "SUCCESS";
	case SPO2_STATE::TIMEOUT:
		return "TIMEOUT";
	default:
		return "UNKNOWN";
	}
}

constexpr auto FMY_FIFO_OUTPUT_READ               = 0x12;
constexpr auto IDX_FIFO_OUTPUT_READ_NUM_OF_SAMPLE = 0x0;
constexpr auto IDX_FIFO_OUTPUT_READ_DATA          = 0x01;


constexpr auto FMY_ALGO_CFG               = 0x50;
constexpr auto IDX_ALGO_CFG_WEARABLE_SUIT = 0x07;

constexpr auto FMY_ALGO_EN               = 0x52;
constexpr auto FMY_ALGO_EN_WEARABLE_SUIT = 0x07;
} // namespace max

#endif /* EE69DC8E_8AA5_4DAB_AB81_FC369B8A90CB */
