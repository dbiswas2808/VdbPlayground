#include <Eigen/Geometry>
#include <RayTracer/Ray.h>

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

    [[nodiscard]] BRDF getBRDF(RayIntersect) const {
        return constantBRDF;
    }
};

[[nodiscard]] BRDF::Color phong_shading(const Eigen::Vector3f& lightDir,
                                        const Eigen::Vector3f& viewDir,
                                        const Eigen::Vector3f& surfaceNormal, BRDF brdf);
}