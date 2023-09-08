#include <Eigen/Geometry>

namespace VdbFields::RayTracer { 
struct Ray {
    Eigen::Vector3f origin;
    Eigen::Vector3f direction;
    Eigen::Vector2f m_minMaxT;

    [[nodiscard]] Ray transform(const Eigen::Affine3f& tx) const;
};

struct RayIntersect {
    Eigen::Vector3f point_world;
    float hitT_mm;
    Eigen::Vector3f normal_world;
};
}