#include <RayTracer/RayTracer.h>
#include <ranges>
#include <RayTracer/SceneEntity.h>

namespace VdbFields {
namespace RayTracer {
namespace {
[[nodiscard]] Eigen::Vector3f snellsReflection(const Eigen::Vector3f& v, const Eigen::Vector3f& n) {
    return v - 2 * v.dot(n) * n;
}

[[nodiscard]] RayTracer::Ray snellsReflection(const RayTracer::Ray& rayIn,
                                              const RayTracer::RayIntersect& intersect) {
    auto refDir = snellsReflection(rayIn.direction, intersect.normal_world);
    return Ray{intersect.point_world + epsilon_mm<float> * intersect.normal_world, refDir};
}

[[nodiscard]] RayTracer::BRDF::Color getVisibleLightIntensity(const RayIntersect& intersect,
                                                              const Eigen::Vector3f& out,
                                                              const Eigen::Vector3f& light) {
    auto h = (out + light).normalized();
    const auto& d = intersect.brdf.diffuse;
    const auto& s = intersect.brdf.specular;
    const auto& n = intersect.normal_world;
    const auto& sh = intersect.brdf.shininess;

    return d * std::max(n.dot(light), 0.f) + s * std::pow(std::max(n.dot(h), 0.f), sh);
}
}  // namespace
}  // namespace RayTracer

[[nodiscard]] RayTracer::BRDF::Color RayTracer::calculateVisibleLightsIllumination(
    Ray rayIn, std::span<const Light> lights,
    const AggregratePrimitiveIntersector& aggregatePrimitiveIntersector, int depth) {
    if (depth <= 0) {
        return BRDF::Color::Zero();
    }

    BRDF::Color sum = BRDF::Color::Zero();
    if (auto rayIntersect = aggregatePrimitiveIntersector.intersect(rayIn)) {
        sum = rayIntersect->brdf.emission + rayIntersect->brdf.ambient;
        for (auto& l : lights) {
            auto li = l.getIntensity(rayIntersect->point_world);

            auto lightRay =
                Ray{rayIntersect->point_world, l.getDirection(rayIntersect->point_world)};
            auto isVisible = not aggregatePrimitiveIntersector.hasIntersection(lightRay);

            if (isVisible) {
                sum += li * getVisibleLightIntensity(rayIntersect.value(), -rayIn.direction,
                                                     lightRay.direction);
            }
        }

        sum += rayIntersect->brdf.reflectivity *
               calculateVisibleLightsIllumination(snellsReflection(rayIn, *rayIntersect), lights,
                                                  aggregatePrimitiveIntersector, depth - 1);
    }

    return sum;
}

namespace RayTracer {
BRDF::Color RayTracerImpl::rayTrace(Ray inRay) {
    // Add diffuse and specular component
    BRDF::Color result =
        calculateVisibleLightsIllumination(inRay, m_lights, m_aggregatePrimitiveIntersector, 8);

    return result;
}
}  // namespace RayTracer
}  // namespace VdbFields