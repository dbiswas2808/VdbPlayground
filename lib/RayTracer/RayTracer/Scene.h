#pragma once
#include <span>
#include <RayTracer/Film.h>
#include <RayTracer/GeometryRayIntersector.h>
#include <RayTracer/Light.h>
#include <RayTracer/Material.h>
#include <RayTracer/RayTracer.h>
#include <RayTracer/SceneEntity.h>

namespace VdbFields::RayTracer {
class Scene {
     public:
      Scene(Eigen::Vector2i shape, Sampler<> sampler, Camera camera,
            std::vector<ShapeIntersector>&& shapeIntersectors, std::vector<Light>&& lights);

      void rayTrace();

     private:
      [[nodiscard]] BRDF::Color rayTraceAtPx(const Eigen::Vector2i& px);

      Sampler<> m_sampler;
      Camera m_camera;
      RayTracerImpl m_rayTracer;
      Film m_film;
};
}  // namespace VdbFields::RayTracer