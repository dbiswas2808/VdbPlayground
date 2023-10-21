#include <RayTracer/Material.h>

namespace VdbFields {
namespace {
Eigen::Vector3f reflect(const Eigen::Vector3f& v, const Eigen::Vector3f& n) {
    return v - 2 * v.dot(n) * n;
}
}

// Implementation of the Phong reflection model
// https://en.wikipedia.org/wiki/Phong_reflection_model
[[nodiscard]] RayTracer::BRDF::Color RayTracer::phong_shading(const Eigen::Vector3f& lightDir,
                                                              const Eigen::Vector3f& viewDir,
                                                              const Eigen::Vector3f& surfaceNormal,
                                                              BRDF brdf) {
    const auto& diffuse = brdf.diffuse;
    const auto& specular = brdf.specular;
    const auto& ambient = brdf.ambient;
    const auto& emission = brdf.emission;
    const auto& shininess = brdf.shininess;

    const auto lightDir_normalized = lightDir.normalized();
    const auto viewDir_normalized = viewDir.normalized();
    const auto surfaceNormal_normalized = surfaceNormal.normalized();

    const auto reflectionDir = reflect(-lightDir_normalized, surfaceNormal_normalized);

    const auto diffuseTerm = std::max(0.f, lightDir_normalized.dot(surfaceNormal_normalized));
    const auto specularTerm = std::pow(std::max(0.f, reflectionDir.dot(viewDir_normalized)), shininess);

    return diffuse * diffuseTerm + specular * specularTerm + ambient + emission;
}
}  // namespace VdbFields