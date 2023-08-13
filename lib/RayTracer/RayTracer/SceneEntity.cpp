#include <RayTracer/SceneEntity.h>
#include <iostream>
#include <png.h>

namespace VdbFields::RayTracer {

Eigen::Matrix3f detail::ndcFromPixel(Eigen::Vector2i shape_px, float fov_deg) {
    auto aspectRatio = shape_px[0] / static_cast<float>(shape_px[1]);
    auto scale = std::tan(std::numbers::pi_v<float> / 180 * (fov_deg * 0.5));
    Eigen::Matrix3f screenFromPixel =
        Eigen::DiagonalMatrix<float, 3>(2.f / shape_px[0], -2.f / shape_px[1], 1.f);
    screenFromPixel.block<2, 1>(0, 2) = Eigen::Vector2f(-1, 1);
    return Eigen::DiagonalMatrix<float, 3>(scale * aspectRatio, scale, 1.f) * screenFromPixel;
}

Camera::Camera(Eigen::Vector3f origin_camera, Eigen::Vector2i shape_px, float fov_deg,
               Eigen::Vector2f minMaxT_mm, Eigen::Matrix4f worldFromCamera)
    : m_origin_camera(origin_camera),
      m_ndcFromPixel(detail::ndcFromPixel(shape_px, fov_deg)),
      m_minMaxT_mm(minMaxT_mm),
      m_worldFromCamera(worldFromCamera) {}

Ray Camera::getRay(Eigen::Vector2f sample_px) const {
    auto sample_screen =
        (m_ndcFromPixel.block<2, 2>(0, 0) * sample_px + m_ndcFromPixel.block<2, 1>(0, 2)).eval();
    Eigen::Vector3f sample_camera;
    sample_camera << sample_screen.x(), sample_screen.y(), -1;

    return Ray{.origin_world = m_worldFromCamera * m_origin_camera.homogeneous(),
               .direction_world = m_worldFromCamera.block<3, 3>(0, 0) *
                                  (sample_camera - m_origin_camera).normalized(),
               .m_minMaxT_mm = m_minMaxT_mm};
}

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
    return RayIntersect{intersectionPt_world, hitT};
}
}  // namespace VdbFields::RayTracer
