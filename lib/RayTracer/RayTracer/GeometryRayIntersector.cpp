#include <RayTracer/GeometryRayIntersector.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#pragma GCC optimize ("O0")

namespace VdbFields::RayTracer {
bool SphereIntersector::hasIntersection(const Ray& ray_world) const {
    Eigen::Vector3f center_world = (m_worldFromGeom * m_sphere.center);
    const auto radiusSq_world = (m_sphere.radius_mm * m_sphere.radius_mm);

    assert(ray.direction_world.stableNorm() - 1.0 < 1e-6);
    const auto rayOriginToSphereCenter_world = (center_world - ray_world.origin).eval();
    const auto proj_world = rayOriginToSphereCenter_world.dot(ray_world.direction);

    const auto b = -2 * proj_world;
    const auto c = rayOriginToSphereCenter_world.squaredNorm() - radiusSq_world;
    const auto discriminant = b * b - 4 * c;
    if (discriminant < 0) {
        return false;
    }

    auto hitT = -0.5f * (b + std::sqrt(discriminant));
    if (std::signbit(hitT)) {
        false;
    }

    return true;
}

std::optional<RayIntersect> SphereIntersector::intersect(const Ray& ray_world) const {
    Eigen::Vector3f center_world = (m_worldFromGeom * m_sphere.center);
    const auto radiusSq_world = (m_sphere.radius_mm * m_sphere.radius_mm);

    assert(ray.direction_world.stableNorm() - 1.0 < epsilon_mm<float>);
    const auto rayOriginToSphereCenter_world = (center_world - ray_world.origin);
    const auto proj_world = rayOriginToSphereCenter_world.dot(ray_world.direction);

    const auto b = -2 * proj_world;
    const auto c = rayOriginToSphereCenter_world.squaredNorm() - radiusSq_world;
    const auto discriminant = b * b - 4 * c;
    if (std::signbit(discriminant)) {
        return std::nullopt;
    }

    auto hitT = -0.5f * (b + std::sqrt(discriminant));
    if (std::signbit(hitT)) {
        return std::nullopt;
    }

    const auto intersectionPt_world = (ray_world.origin + hitT * ray_world.direction);
    const auto normal_world = (intersectionPt_world - center_world).normalized();
    return RayIntersect{.point_world = intersectionPt_world,
                        .hitT_mm = hitT,
                        .normal_world = normal_world,
                        .brdf = m_material.getBRDF(intersectionPt_world)};
}

std::optional<RayIntersect> TriMeshIntersector::intersect(const Ray& ray) const {
    auto ray_geom = ray.transform(m_geomFromWorld);

    BVHRay bvhRay_geom = {.origin = ray_geom.origin,
                          .direction = (m_geomFromWorld.rotation() * ray.direction).eval(),
                          .invDirection = ray.direction.cwiseInverse()};
    bvh.intersect(bvhRay_geom);
    if (bvhRay_geom.hasIntersection()) {
        const auto intersectionPt_world =
            m_worldFromGeom * (bvhRay_geom.origin + bvhRay_geom.t * bvhRay_geom.direction);
        const auto normal_world =
            m_worldFromGeom.rotation().transpose().inverse() * bvhRay_geom.normal;
        return RayIntersect{.point_world = intersectionPt_world,
                            .hitT_mm = bvhRay_geom.t,
                            .normal_world = normal_world,
                            .brdf = m_material.getBRDF(intersectionPt_world)};
    }
    return std::nullopt;
}

std::optional<RayIntersect> AggregratePrimitiveIntersector::intersect(const Ray& ray_world) const {
    std::optional<RayIntersect> closestIntersect;

    for (const auto& shapeIntersector : m_shapeIntersectors) {
        if (auto intersect = shapeIntersector.intersect(ray_world)) {
            if (!closestIntersect || intersect->hitT_mm < closestIntersect->hitT_mm) {
                closestIntersect = intersect;
            }
        }
    }
    return closestIntersect;
}
}  // namespace VdbFields::RayTracer