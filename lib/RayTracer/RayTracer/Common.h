#include <stlab/copy_on_write.hpp>

namespace VdbFields {
template <class T>
inline constexpr std::enable_if_t<std::is_floating_point_v<T>, T> epsilon_mm;

template <>
inline constexpr float epsilon_mm<float> = 0.0001f;

template <typename T>
using cow = stlab::copy_on_write<T>;
}  // namespace VdbFields