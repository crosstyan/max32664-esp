#include "inc/flash.hpp"
#include "driver/i2c_types.h"
#include "esp_err.h"
#include "soc/gpio_num.h"
#include <cstdint>
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
	return msbl().subspan(0x34, AES_AUTH_SIZE);
}

// nonce for AES, expecting a 16-byte array
//
// [0x28, 0x32]
const std::span<const uint8_t> init_vector_bytes() {
	return msbl().subspan(0x28, AES_NONCE_SIZE);
}
uint8_t number_of_pages() {
	return msbl()[0x44];
}
}
