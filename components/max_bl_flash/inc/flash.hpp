#ifndef E1F84456_BFA5_4757_A3AF_13F4CF09E1D9
#define E1F84456_BFA5_4757_A3AF_13F4CF09E1D9
#include <cstdint>
#include <span>

namespace flash {
constexpr auto AES_AUTH_SIZE   = 16;
constexpr auto AES_NONCE_SIZE  = 11;
constexpr auto MAX_PAGE_SIZE   = 8192;
constexpr auto CHECKBYTES_SIZE = 16;

constexpr auto PER_READ_SIZE = MAX_PAGE_SIZE + CHECKBYTES_SIZE;
constexpr auto CONTENT_START = 0x4c;

// write to the bootloader
constexpr uint8_t FMY_BL_W           = 0x80;
constexpr uint8_t IDX_BL_W_NONCE     = 0x00;
constexpr uint8_t IDX_BL_W_AUTH      = 0x01;
constexpr uint8_t IDX_BL_W_PAGE_NUM  = 0x02;
constexpr uint8_t IDX_BL_W_ERASE_APP = 0x03;
constexpr uint8_t IDX_BL_W_SEND_PAGE = 0x04;

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

const std::span<const uint8_t>
msbl();
const std::span<const uint8_t> auth_bytes();
const std::span<const uint8_t> init_vector_bytes();
uint8_t number_of_pages();
}

#endif /* E1F84456_BFA5_4757_A3AF_13F4CF09E1D9 */
