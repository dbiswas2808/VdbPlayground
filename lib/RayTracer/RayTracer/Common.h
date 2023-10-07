#include <stlab/copy_on_write.hpp>
#include <Eigen/Geometry>

namespace VdbFields {
template <class T>
inline constexpr std::enable_if_t<std::is_floating_point_v<T>, T> epsilon_mm;

template <>
inline constexpr float epsilon_mm<float> = 0.0001f;

template <typename T>
using cow = stlab::copy_on_write<T>;

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


[[nodiscard]] Eigen::Matrix3f txInvTranspose(const Eigen::Affine3f& transform) {
    return transform.rotation().transpose().inverse();
}
}  // namespace VdbFields