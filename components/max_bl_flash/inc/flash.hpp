#ifndef E1F84456_BFA5_4757_A3AF_13F4CF09E1D9
#define E1F84456_BFA5_4757_A3AF_13F4CF09E1D9
#include <cstdint>
#include <span>
#include "max.hpp"

namespace max::flash {
constexpr auto AES_AUTH_SIZE   = 16;
constexpr auto AES_NONCE_SIZE  = 11;
constexpr auto MAX_PAGE_SIZE   = 8192;
constexpr auto CHECKBYTES_SIZE = 16;

// layout: PAGE_NUM * (PAGE_SIZE + CHECKBYTES_SIZE) from APP_START_OFFSET
constexpr auto APP_START_OFFSET = 0x4c;

// write to the bootloader
constexpr uint8_t FMY_BL_W           = 0x80;
constexpr uint8_t IDX_BL_W_NONCE     = 0x00;
constexpr uint8_t IDX_BL_W_AUTH      = 0x01;
constexpr uint8_t IDX_BL_W_PAGE_NUM  = 0x02;
constexpr uint8_t IDX_BL_W_ERASE_APP = 0x03;
constexpr uint8_t IDX_BL_W_SEND_PAGE = 0x04;

const std::span<const uint8_t> msbl();
const std::span<const uint8_t> auth_bytes();
const std::span<const uint8_t> init_vector_bytes();
uint8_t number_of_pages();
}

#endif /* E1F84456_BFA5_4757_A3AF_13F4CF09E1D9 */
