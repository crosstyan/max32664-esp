#ifndef E1F84456_BFA5_4757_A3AF_13F4CF09E1D9
#define E1F84456_BFA5_4757_A3AF_13F4CF09E1D9
#include <cstdint>
#include <span>

namespace flash {
const std::span<const uint8_t> msbl();
const std::span<const uint8_t> auth_bytes();
const std::span<const uint8_t> init_vector_bytes();
}

#endif /* E1F84456_BFA5_4757_A3AF_13F4CF09E1D9 */
