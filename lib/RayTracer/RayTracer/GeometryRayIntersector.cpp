#include <RayTracer/GeometryRayIntersector.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#pragma GCC optimize ("O0")

namespace VdbFields::RayTracer {
namespace {
[[nodiscard]] BVHRay bvhRayFromRay(const Ray& ray) {
    return {.origin = ray.origin,
            .direction = ray.direction,
            .invDirection = ray.direction.cwiseInverse()};
}
}  // namespace

/*static*/ 
AABB SphereIntersector::makeAABB_world(const Sphere& sphere, const Eigen::Affine3f& worldFromGeom) {
    AABB result_world;
    result_world.extend(worldFromGeom * (sphere.center + Eigen::Vector3f(sphere.radius_mm, 0, 0)));
    result_world.extend(worldFromGeom * (sphere.center + Eigen::Vector3f(0, sphere.radius_mm, 0)));
    result_world.extend(worldFromGeom * (sphere.center + Eigen::Vector3f(0, 0, sphere.radius_mm)));
    result_world.extend(worldFromGeom * (sphere.center + Eigen::Vector3f(-sphere.radius_mm, 0, 0)));
    result_world.extend(worldFromGeom * (sphere.center + Eigen::Vector3f(0, -sphere.radius_mm, 0)));
    result_world.extend(worldFromGeom * (sphere.center + Eigen::Vector3f(0, 0, -sphere.radius_mm)));

    return result_world;
}

bool SphereIntersector::hasIntersection(const Ray& ray_world) const {
    if (not std::isfinite(VdbFields::intersectAABB(
            ray_world.origin, ray_world.direction.cwiseInverse(),
            std::numeric_limits<float>::infinity(), m_aabb_world.aabbMin, m_aabb_world.aabbMax))) {
        return false;
    }

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
    if (not std::isfinite(VdbFields::intersectAABB(
            ray_world.origin, ray_world.direction.cwiseInverse(),
            std::numeric_limits<float>::infinity(), m_aabb_world.aabbMin, m_aabb_world.aabbMax))) {
        return std::nullopt;
    }

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

std::optional<RayIntersect> TriMeshIntersector::intersect(const Ray& ray_world) const {
    BVHRay bvhRay_world = bvhRayFromRay(ray_world);
    m_bvh.intersect(bvhRay_world);
    if (bvhRay_world.hasIntersection()) {
        const auto intersectionPt_world = bvhRay_world.getIntersect();
        const auto normal_world = bvhRay_world.normal.normalized();
        return RayIntersect{.point_world = intersectionPt_world,
                            .hitT_mm = bvhRay_world.t,
                            .normal_world = normal_world,
                            .brdf = m_material.getBRDF(intersectionPt_world)};
    }
    return std::nullopt;
}

std::optional<RayIntersect> AggregateMeshIntersector::intersect(const Ray& ray_world) const {
    auto bvhRay_world = bvhRayFromRay(ray_world);
    m_tlas->intersect(bvhRay_world);

    BRDF material = {
        .diffuse = Eigen::Vector3f(6.8, 10, 5.8),   // Diffuse reflectance (kd)
        .specular = Eigen::Vector3f(0.1, 0.5, 0.5),  // Specular reflectance (ks)
        .ambient = Eigen::Vector3f(0.1, 0.2, 0.2),   // Ambient reflectance (ka)
        .emission = Eigen::Vector3f(0.0, 0.0, 0.0),  // Emission (glow)
        .shininess = 30                              // Shininess (n)
    };

    if (bvhRay_world.hasIntersection()) {
        const auto intersectionPt_world = bvhRay_world.getIntersect();
        const auto normal_world = bvhRay_world.normal.normalized();
        return RayIntersect{.point_world = intersectionPt_world,
                            .hitT_mm = bvhRay_world.t,
                            .normal_world = normal_world,
                            .brdf = m_materials.at(bvhRay_world.bvhIdx).getBRDF(intersectionPt_world)};
    }

    return std::nullopt;
 }
bool AggregateMeshIntersector::hasIntersection(const Ray& ray_world) const {
    return intersect(ray_world).has_value();
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