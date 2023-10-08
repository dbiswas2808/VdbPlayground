#pragma once
#include <stlab/copy_on_write.hpp>
#include <Eigen/Geometry>

namespace VdbFields {
template <class T>
inline constexpr std::enable_if_t<std::is_floating_point_v<T>, T> epsilon_mm;

template <>
inline constexpr float epsilon_mm<float> = 0.0001f;

template <typename T>
using cow = stlab::copy_on_write<T>;

struct TriIntersectData {
    float t;
    Eigen::Vector2f uv;
};

struct AABB {
    Eigen::Vector3f aabbMin = Eigen::Vector3f::Constant(std::numeric_limits<float>::infinity());
    Eigen::Vector3f aabbMax = Eigen::Vector3f::Constant(-std::numeric_limits<float>::infinity());

    void extend(const Eigen::Vector3f& point);
    void extend(const AABB& other);
    [[nodiscard]] bool isEmpty() const;
    [[nodiscard]] float area() const;
    [[nodiscard]] bool contains(const Eigen::Vector3f& pt) const;

    [[nodiscard]] friend AABB operator*(const Eigen::Affine3f& transform, const AABB& aabb);
};

[[nodiscard]] float intersectAABB(const Eigen::Vector3f& origin, const Eigen::Vector3f& invDir,
                                  float t, const Eigen::Vector3f& aabbMin,
                                  const Eigen::Vector3f& aabbMax);

[[nodiscard]] inline Eigen::Matrix3f txInvTranspose(const Eigen::Affine3f& transform) {
    return transform.rotation().transpose().inverse();
}

[[nodiscard]] TriIntersectData intersectTriangle(const Eigen::Vector3f& origin,
                                                 const Eigen::Vector3f& direction,
                                                 const Eigen::Vector3f& v0,
                                                 const Eigen::Vector3f& v1,
                                                 const Eigen::Vector3f& v2);

[[nodiscard]] inline Eigen::Vector3f normal(const Eigen::Vector3f& v0, const Eigen::Vector3f& v1,
                                            const Eigen::Vector3f& v2) {
    return (v2 - v1).cross(v0 - v1).normalized();
}

}  // namespace VdbFields