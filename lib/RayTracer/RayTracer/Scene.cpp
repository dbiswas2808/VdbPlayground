#include <RayTracer/Scene.h>

namespace VdbFields::RayTracer {
Scene::Scene(Eigen::Vector2i shape, Sampler<> sampler, Camera camera,
             std::vector<ShapeIntersector>&& shapeIntersectors, std::vector<Light>&& lights)
    : m_sampler(sampler),
      m_camera(camera),
      m_rayTracer(std::move(shapeIntersectors), std::move(lights)),
      m_film(shape) {}

[[nodiscard]] BRDF::Color Scene::rayTraceAtPx(const Eigen::Vector2i& px) {
    Eigen::Vector3f color = Eigen::Vector3f(0, 0, 0);
    m_sampler.setPixel(px);

    for (int kk = 0; kk < 1; ++kk) {
        Eigen::Vector2f sample_px = m_sampler.getSample_px();
        Ray ray = m_camera.getRay_camera(sample_px);
        color += m_rayTracer.rayTrace(ray);
    }

    return color / 1;
}

void Scene::rayTrace() {
    for (int ii = 0; ii < m_film.getShape_px().prod(); ++ii) {
        const auto px = Eigen::Vector2i(ii % m_film.getShape_px()[0], ii / m_film.getShape_px()[1]);
        m_film.addSample(px, rayTraceAtPx(px));
    }
}
}  // namespace VdbFields::RayTracer
