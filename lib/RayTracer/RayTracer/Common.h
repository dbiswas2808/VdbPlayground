#include <stlab/copy_on_write.hpp>

namespace VdbFields::RayTracer {
template<typename T>
using cow = stlab::copy_on_write<T>;
}