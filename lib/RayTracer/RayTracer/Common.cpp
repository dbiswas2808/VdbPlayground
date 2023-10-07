#include <RayTracer/Common.h>

namespace VdbFields {
// AABB implementation
void AABB::extend(const Eigen::Vector3f& point) {
    aabbMin = aabbMin.cwiseMin(point);
    aabbMax = aabbMax.cwiseMax(point);
}

void AABB::extend(const AABB& other) {
    aabbMin = aabbMin.cwiseMin(other.aabbMin);
    aabbMax = aabbMax.cwiseMax(other.aabbMax);
}

[[nodiscard]] bool AABB::isEmpty() const {
    return aabbMin.x() > aabbMax.x() || aabbMin.y() > aabbMax.y() || aabbMin.z() > aabbMax.z();
}

[[nodiscard]] float AABB::area() const {
    Eigen::Vector3f diff = aabbMax - aabbMin;
    return isEmpty() ? 0 : 2 * (diff.x() * diff.y() + diff.x() * diff.z() + diff.y() * diff.z());
}

[[nodiscard]] bool AABB::contains(const Eigen::Vector3f& pt) const {
    return pt.x() >= aabbMin.x() && pt.x() <= aabbMax.x() && pt.y() >= aabbMin.y() &&
           pt.y() <= aabbMax.y() && pt.z() >= aabbMin.z() && pt.z() <= aabbMax.z();
}

AABB operator*(const Eigen::Affine3f& transform, const AABB& aabb) {
    AABB result;
    for (int ii = 0; ii < 8; ++ii) {
        auto corner = Eigen::Vector3f(ii & 0b001 ? aabb.aabbMin[0] : aabb.aabbMax[0],
                                      ii & 0b010 ? aabb.aabbMin[1] : aabb.aabbMax[1],
                                      ii & 0b100 ? aabb.aabbMin[2] : aabb.aabbMax[2]);
        result.extend(transform * corner);
    }

    return result;
}
}  // namespace VdbFields

float VdbFields::intersectAABB(const Eigen::Vector3f& origin, const Eigen::Vector3f& invDir,
                               float t, const Eigen::Vector3f& aabbMin,
                               const Eigen::Vector3f& aabbMax) {
    float tx1 = (aabbMin.x() - origin.x()) * invDir.x();
    float tx2 = (aabbMax.x() - origin.x()) * invDir.x();
    float tmin = std::min(tx1, tx2);
    float tmax = std::max(tx1, tx2);

    float ty1 = (aabbMin.y() - origin.y()) * invDir.y();
    float ty2 = (aabbMax.y() - origin.y()) * invDir.y();
    tmin = std::max(tmin, std::min(ty1, ty2));
    tmax = std::min(tmax, std::max(ty1, ty2));

    float tz1 = (aabbMin.z() - origin.z()) * invDir.z();
    float tz2 = (aabbMax.z() - origin.z()) * invDir.z();

    tmin = std::max(tmin, std::min(tz1, tz2));
    tmax = std::min(tmax, std::max(tz1, tz2));

    return (tmax >= tmin && tmin < t && tmax > 0) ? tmin : std::numeric_limits<float>::infinity();
}