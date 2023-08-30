#include <RayTracer/GeometryRayIntersector.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace VdbFields::RayTracer {
Ray Ray::transform(Eigen::Matrix4f tx) const {
    return Ray{.origin = (tx * origin.homogeneous()).hnormalized(),
               .direction = (tx.block<3, 3>(0, 0) * direction),
               .m_minMaxT = m_minMaxT};
}

std::optional<RayIntersect> SphereIntersector::intersect(const Ray& ray_camera) const {
    Ray ray_world = ray_camera.transform(m_worldFromCamera);

    Eigen::Vector3f center_world = (m_worldFromGeom * m_sphere.center.homogeneous()).hnormalized();
    const auto radiusSq_world = (m_sphere.radius_mm * m_sphere.radius_mm);

    assert(ray.direction_world.stableNorm() - 1.0 < 1e-6);
    const auto rayOriginToSphereCenter_world = (center_world - ray_world.origin).eval();
    const auto proj = rayOriginToSphereCenter_world.dot(ray_world.direction);

    const auto b = -2 * proj;
    const auto c = rayOriginToSphereCenter_world.squaredNorm() - radiusSq_world;
    const auto discriminant = b * b - 4 * c;
    if (discriminant < 0) {
        return std::nullopt;
    }

    auto hitT = -0.5f * (b + std::sqrt(discriminant));
    const auto intersectionPt_world = (ray_world.origin + hitT * ray_world.direction).eval();
    const auto normal_world = (intersectionPt_world - center_world).stableNormalized();
    return RayIntersect{intersectionPt_world, hitT, normal_world.eval()};
}
}