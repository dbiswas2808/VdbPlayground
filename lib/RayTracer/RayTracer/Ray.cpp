#include <RayTracer/Ray.h>

namespace VdbFields::RayTracer {
Ray Ray::transform(const Eigen::Affine3f& tx) const {
    return Ray{.origin = (tx * origin),
               .direction = (tx.linear() * direction),
               .m_minMaxT = m_minMaxT};
}
}  // namespace VdbFields::RayTracer