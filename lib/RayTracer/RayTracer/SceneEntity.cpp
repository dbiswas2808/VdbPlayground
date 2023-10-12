#include <RayTracer/SceneEntity.h>

#include <iostream>
#include <png.h>

#include <RayTracer/GeometryRayIntersector.h>

namespace VdbFields {
namespace RayTracer {
Eigen::Affine2f detail::ndcFromPixel(Eigen::Vector2i shape_px, float fov_deg) {
    auto aspectRatio = shape_px[0] / static_cast<float>(shape_px[1]);
    auto scale = std::tan(std::numbers::pi_v<float> / 180 * (fov_deg * 0.5));
    return Eigen::Scaling<float>(scale * aspectRatio, scale) * Eigen::Translation2f(-1, 1) *
           Eigen::Affine2f(Eigen::Scaling(2.f / shape_px[0], -2.f / shape_px[1]));
}

Camera::Camera(Eigen::Vector3f origin_camera, Eigen::Vector2i shape_px, float fov_deg,
               Eigen::Vector2f minMaxT)
    : m_origin_camera(origin_camera),
      m_ndcFromPixel(detail::ndcFromPixel(shape_px, fov_deg)),
      m_minMaxT(minMaxT) {}

Ray Camera::getRay_camera(Eigen::Vector2f sample_px) const {
    auto sample_screen =
        (m_ndcFromPixel * sample_px).eval();
    Eigen::Vector3f sample_camera;
    sample_camera << sample_screen.x(), sample_screen.y(), -1;

    return Ray{.origin = m_origin_camera,
               .direction = (sample_camera - m_origin_camera).normalized(),
               .m_minMaxT = m_minMaxT};
}
}  // namespace VdbFields::RayTracer

[[nodiscard]] Eigen::Affine3f RayTracer::lookAt_cameraFromWorld(Eigen::Vector3f eye_world,
                                                                Eigen::Vector3f target_world,
                                                                Eigen::Vector3f up_world) {
    auto dir_world = (eye_world - target_world).normalized();
    auto right_world = up_world.cross(dir_world).normalized();
    auto newUp_world = dir_world.cross(right_world).normalized();

    Eigen::Affine3f cameraFromWorld = Eigen::Affine3f::Identity();
    cameraFromWorld.linear().row(0) = right_world;
    cameraFromWorld.linear().row(1) = newUp_world;
    cameraFromWorld.linear().row(2) = dir_world;
    cameraFromWorld *= Eigen::Translation3f(-eye_world);

    return cameraFromWorld;
}
}  // namespace VdbFields