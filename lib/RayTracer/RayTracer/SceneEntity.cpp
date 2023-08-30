#include <RayTracer/SceneEntity.h>

#include <iostream>
#include <png.h>

#include <RayTracer/GeometryRayIntersector.h>



namespace VdbFields {
namespace RayTracer {
Eigen::Matrix3f detail::ndcFromPixel(Eigen::Vector2i shape_px, float fov_deg) {
    auto aspectRatio = shape_px[0] / static_cast<float>(shape_px[1]);
    auto scale = std::tan(std::numbers::pi_v<float> / 180 * (fov_deg * 0.5));
    Eigen::Matrix3f screenFromPixel =
        Eigen::DiagonalMatrix<float, 3>(2.f / shape_px[0], -2.f / shape_px[1], 1.f);
    screenFromPixel.block<2, 1>(0, 2) = Eigen::Vector2f(-1, 1);
    return Eigen::DiagonalMatrix<float, 3>(scale * aspectRatio, scale, 1.f) * screenFromPixel;
}

Camera::Camera(Eigen::Vector3f origin_camera, Eigen::Vector2i shape_px, float fov_deg,
               Eigen::Vector2f minMaxT)
    : m_origin_camera(origin_camera),
      m_ndcFromPixel(detail::ndcFromPixel(shape_px, fov_deg)),
      m_minMaxT(minMaxT) {}

Ray Camera::getRay_camera(Eigen::Vector2f sample_px) const {
    auto sample_screen =
        (m_ndcFromPixel.block<2, 2>(0, 0) * sample_px + m_ndcFromPixel.block<2, 1>(0, 2)).eval();
    Eigen::Vector3f sample_camera;
    sample_camera << sample_screen.x(), sample_screen.y(), -1;

    return Ray{.origin = m_origin_camera,
               .direction = (sample_camera - m_origin_camera).normalized(),
               .m_minMaxT = m_minMaxT};
}
}  // namespace VdbFields::RayTracer

[[nodiscard]] Eigen::Matrix4f RayTracer::lookAt_cameraFromWorld(Eigen::Vector3f eye_world,
                                                                Eigen::Vector3f target_world,
                                                                Eigen::Vector3f up_world) {
    auto dir_world = (eye_world - target_world).stableNormalized();
    auto right_world = up_world.cross(dir_world).stableNormalized();
    auto newUp_world = dir_world.cross(right_world).stableNormalized();

    Eigen::Matrix4f cameraFromWorld = Eigen::Matrix4f::Identity();
    cameraFromWorld.block<1, 3>(0, 0) = right_world;
    cameraFromWorld.block<1, 3>(1, 0) = newUp_world;
    cameraFromWorld.block<1, 3>(2, 0) = dir_world;
    cameraFromWorld.block<3, 1>(0, 3) = -eye_world;

    return cameraFromWorld;
}
}  // namespace VdbFields