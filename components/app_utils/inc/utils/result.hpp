#ifndef B4FE4219_FA85_454A_91ED_9B150094CB1E
#define B4FE4219_FA85_454A_91ED_9B150094CB1E
#include <tl/expected.hpp>

namespace utils {
template <typename T, typename E>
using Result = tl::expected<T, E>;
using Unit   = tl::monostate;

template <typename E>
using unexpected = tl::unexpected<E>;
}

#endif /* B4FE4219_FA85_454A_91ED_9B150094CB1E */
