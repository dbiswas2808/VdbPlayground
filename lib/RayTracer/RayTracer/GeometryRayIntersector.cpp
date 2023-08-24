#include <RayTracer/GeometryRayIntersector.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace VdbFields::RayTracer {
std::optional<RayIntersect> SphereIntersector::intersect(const Ray& ray) const {
    Eigen::Vector3f center_world = (m_worldFromGeom * m_sphere.center.homogeneous()).hnormalized();
    const auto radiusSq_world = (m_sphere.radius_mm * m_sphere.radius_mm);

    assert(ray.direction_world.stableNorm() - 1.0 < 1e-6);
    const auto rayOriginToSphereCenter_world = (center_world - ray.origin_world).eval();
    const auto proj = rayOriginToSphereCenter_world.dot(ray.direction_world);

    const auto b = -2 * proj;
    const auto c = rayOriginToSphereCenter_world.squaredNorm() - radiusSq_world;
    const auto discriminant = b * b - 4 * c;
    if (discriminant < 0) {
        return std::nullopt;
    }

    auto hitT = -0.5f * (b + std::sqrt(discriminant));
    const auto intersectionPt_world = (ray.origin_world + hitT * ray.direction_world).eval();
    const auto normal_world = (intersectionPt_world - center_world).stableNormalized();
    return RayIntersect{intersectionPt_world, hitT, normal_world.eval()};
}
}