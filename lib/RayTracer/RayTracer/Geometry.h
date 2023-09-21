#pragma once
#include <vector>
#include <Eigen/Geometry>

namespace VdbFields {
template <class T>
inline constexpr std::enable_if_t<std::is_floating_point_v<T>, T> epsilon_mm;

template <>
inline constexpr float epsilon_mm<float> = 0.0001f;
}

namespace VdbFields::RayTracer {


struct TriMesh {
    Eigen::Matrix<float, 3, Eigen::Dynamic> points;
    std::vector<size_t> tris;
    [[nodiscard]] std::array<Eigen::Vector3f, 4> getTriangle(size_t triIdx) const {
        return {points.col(tris[3 * triIdx + 0]), points.col(tris[3 * triIdx + 1]),
                points.col(tris[3 * triIdx + 2])};
    }
};

struct Sphere {
      Eigen::Vector3f center;
      float radius_mm;
};
}