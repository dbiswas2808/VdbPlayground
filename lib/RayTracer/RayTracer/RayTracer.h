#pragma once
#include <span>

#include <RayTracer/GeometryRayIntersector.h>
#include <RayTracer/Light.h>
#include <RayTracer/Ray.h>

namespace VdbFields::RayTracer {
[[nodiscard]] BRDF::Color calculateVisibleLightsIllumination(
    Ray rayIn, std::span<const Light> lights,
    const AggregratePrimitiveIntersector& aggregatePrimitiveIntersector, int depth = 5);

class RayTracerImpl {
   public:
    RayTracerImpl(AggregratePrimitiveIntersector&& aggregatePrimitiveIntersector,
                  std::vector<Light>&& lights)
        : m_aggregatePrimitiveIntersector(std::move(aggregatePrimitiveIntersector)),
          m_lights(std::move(lights)){};

    [[nodiscard]] BRDF::Color rayTrace(Ray inRay);

    AggregratePrimitiveIntersector m_aggregatePrimitiveIntersector;
    std::vector<Light> m_lights;
};
}  // namespace VdbFields::RayTracer