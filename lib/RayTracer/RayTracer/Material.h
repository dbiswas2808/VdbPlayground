#pragma once
#include <Eigen/Geometry>

namespace VdbFields::RayTracer {
struct BRDF {
    using Color = Eigen::Vector3f;
    Color diffuse;
    Color specular;
    Color ambient;
    Color emission;
    int shininess;
};

struct Material {
    BRDF constantBRDF;

    [[nodiscard]] BRDF getBRDF(Eigen::Vector3f) const {
        return constantBRDF;
    }
};

[[nodiscard]] BRDF::Color phong_shading(const Eigen::Vector3f& lightDir,
                                        const Eigen::Vector3f& viewDir,
                                        const Eigen::Vector3f& surfaceNormal, BRDF brdf);
}