#pragma once
#include <Eigen/Geometry>
#include <RayTracer/Material.h>

namespace VdbFields::RayTracer { 
struct Ray {
    Eigen::Vector3f origin;
    Eigen::Vector3f direction;
    Eigen::Vector2f m_minMaxT = Eigen::Vector2f(0, std::numeric_limits<float>::infinity());

    [[nodiscard]] Ray transform(const Eigen::Affine3f& tx) const;
};

struct RayIntersect {
    Eigen::Vector3f point_world;
    float hitT_mm;
    Eigen::Vector3f normal_world;
    BRDF brdf;
};
}  // namespace VdbFields::RayTracer