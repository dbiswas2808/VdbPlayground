#pragma once
#include <span>
#include <RayTracer/Film.h>
#include <RayTracer/GeometryRayIntersector.h>
#include <RayTracer/Light.h>
#include <RayTracer/Material.h>
#include <RayTracer/RayTracer.h>
#include <RayTracer/SceneEntity.h>

#pragma GCC optimize("O0")
namespace VdbFields::RayTracer {

template<typename SamplerT>
class Scene {
   public:
    Scene(Eigen::Vector2i shape, SamplerT sampler, Camera camera,
          std::vector<ShapeIntersector>&& shapeIntersectors, std::vector<Light>&& lights)
        : m_sampler(sampler),
          m_camera(camera),
          m_rayTracer(std::move(shapeIntersectors), std::move(lights)),
          m_film(shape) {}

    void rayTrace(std::function<void(int, int)> progress = nullptr, size_t numSamples = 1) {
        for (int ii = 0; ii < m_film.getShape_px().prod(); ++ii) {
            if (progress) {
                progress(ii, m_film.getShape_px().prod());
            }

            const auto px =
                Eigen::Vector2i(ii % m_film.getShape_px()[0], ii / m_film.getShape_px()[0]);
            m_film.addSample(px, this->rayTraceAtPx(px, numSamples));
        }
    }

    void writRayTracedImageToFile(std::string filename) const { m_film.imageToFile(filename); }

   private:
    [[nodiscard]] BRDF::Color rayTraceAtPx(const Eigen::Vector2i& px, size_t numSamples) {
        Eigen::Vector3f color = Eigen::Vector3f(0, 0, 0);
        m_sampler.setPixel(px);

        for (size_t kk = 0; kk < numSamples; ++kk) {
            Eigen::Vector2f sample_px = m_sampler.getSample_px();
            Ray ray = m_camera.getRay_camera(sample_px);
            color += m_rayTracer.rayTrace(ray);
        }

        return color / numSamples;
    }

    SamplerT m_sampler;
    Camera m_camera;
    RayTracerImpl m_rayTracer;
    Film m_film;
};
}  // namespace VdbFields::RayTracer