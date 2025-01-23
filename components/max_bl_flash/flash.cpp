#include "inc/flash.hpp"
#include <driver/i2c_master.h>

extern "C" {
extern const uint8_t app_msbl_start[] asm("_binary_app_msbl_start");
extern const uint8_t app_msbl_end[] asm("_binary_app_msbl_end");
}

namespace flash {
const std::span<const uint8_t> msbl() {
	return {app_msbl_start, app_msbl_end};
}

// [0x34, 0x43]
const std::span<const uint8_t> auth_bytes() {
	// note that subspan is (offset, count)
	return msbl().subspan(0x34, 0x43 - 0x34);
}

// [0x28, 0x32]
const std::span<const uint8_t> init_vector_bytes() {
	return msbl().subspan(0x28, 0x32 - 0x28);
}
}
