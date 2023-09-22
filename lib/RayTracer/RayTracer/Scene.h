#pragma once
#include <span>
#include <tbb/tbb.h>
#include <RayTracer/Film.h>
#include <RayTracer/GeometryRayIntersector.h>
#include <RayTracer/Light.h>
#include <RayTracer/Material.h>
#include <RayTracer/RayTracer.h>
#include <RayTracer/SceneEntity.h>

#pragma GCC optimize("O0")
namespace VdbFields::RayTracer {

template <typename SamplerT>
class Scene {
   public:
    Scene(Eigen::Vector2i shape, Camera camera, std::vector<ShapeIntersector>&& shapeIntersectors,
          std::vector<Light>&& lights,
          std::optional<Eigen::Vector2i> tiles = Eigen::Vector2i(1000, 1000))
        : m_camera(camera),
          m_rayTracer(std::move(shapeIntersectors), std::move(lights)),
          m_film(shape),
          m_tiles(tiles) {}

    void rayTrace(std::function<void(int, int)> progress = nullptr, size_t numSamples = 1) {
        if (m_tiles) {
            auto nX_px = m_film.getShape_px()[0];
            auto nY_px = m_film.getShape_px()[1];

            tbb::parallel_for(
                tbb::blocked_range2d(0, nX_px, (*m_tiles)[0], 0, nY_px, (*m_tiles)[1]),
                [&](const tbb::blocked_range2d<int>& r) {
                    SamplerT sampler;
                    for (int x_px : std::ranges::iota_view(r.rows().begin(), r.rows().end())) {
                        for (int y_px : std::ranges::iota_view(r.cols().begin(), r.cols().end())) {
                            const auto px = Eigen::Vector2i(x_px, y_px);
                            sampler.setPixel(px);
                            m_film.addSample(px, this->rayTraceAtPx(sampler, numSamples));
                            if (progress) {
                                progress(x_px * y_px, m_film.getShape_px().prod());
                            }
                        }
                    }
                });
        } else {
            SamplerT sampler;
            for (int ii = 0; ii < m_film.getShape_px().prod(); ++ii) {
                if (progress) {
                    progress(ii, m_film.getShape_px().prod());
                }
                const auto px =
                    Eigen::Vector2i(ii % m_film.getShape_px()[0], ii / m_film.getShape_px()[0]);
                sampler.setPixel(px);

                m_film.addSample(px, this->rayTraceAtPx(sampler, numSamples));
            }
        }
    }

    void writRayTracedImageToFile(std::string filename) const { m_film.imageToFile(filename); }

   private:
    [[nodiscard]] BRDF::Color rayTraceAtPx(SamplerT& sampler, size_t numSamples) {
        Eigen::Vector3f color = Eigen::Vector3f(0, 0, 0);
        for (size_t kk = 0; kk < numSamples; ++kk) {
            Eigen::Vector2f sample_px = sampler.getSample_px();
            Ray ray = m_camera.getRay_camera(sample_px);
            color += m_rayTracer.rayTrace(ray);
        }

        return color / numSamples;
    }

    Camera m_camera;
    RayTracerImpl m_rayTracer;
    Film m_film;
    std::optional<Eigen::Vector2i> m_tiles;
};
}  // namespace VdbFields::RayTracer