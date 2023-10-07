#include <RayTracer/SceneEntity.h>
#include <catch2/catch_test_macros.hpp>
#include <RayTracer/Film.h>
#include <RayTracer/GeometryRayIntersector.h>

namespace VdbFields {
namespace {
class InfiniteXYPlaneIntersector {
   public:
    explicit InfiniteXYPlaneIntersector(float zoffset_mm, const Eigen::Affine3f& worldFromCamera,
                                        const Eigen::Affine3f& worldFromGeom)
        : m_zoffset_mm(zoffset_mm),
          m_worldFromGeom(worldFromGeom) {}

    [[nodiscard]] virtual std::optional<RayTracer::RayIntersect> intersect(
        const RayTracer::Ray& ray_world) const final {
        assert(ray.direction_world.z() > epsilon_mm<float>);
        auto hitT = (m_zoffset_mm - ray_world.origin.z()) / ray_world.direction.z();
        const auto intersectionPt_world = (ray_world.origin + hitT * ray_world.direction).eval();
        return RayTracer::RayIntersect{intersectionPt_world, hitT};
    }
    [[nodiscard]] virtual bool hasIntersection(const RayTracer::Ray& ray) const {
        return false;
    }
    [[nodiscard]] virtual RayTracer::BRDF getBRDF(
        const RayTracer::RayIntersect& intersect) const {
        return {};
    }

   private:
    Eigen::Affine3f m_worldFromGeom;
    float m_zoffset_mm;
};
}  // namespace

TEST_CASE("SceneEntity: Ray sampling") {
    RayTracer::Sampler sampler;
    sampler.setPixel(Eigen::Vector2i(0, 0));
    CHECK(sampler.getSample_px() == Eigen::Vector2f(0.5f, 0.5f));

    sampler = RayTracer::Sampler();
    sampler.setPixel({100, 20});

    CHECK(sampler.getSample_px() == Eigen::Vector2f({100.5f, 20.5f}));

    sampler = RayTracer::Sampler();
    sampler.setPixel(Eigen::Vector2i(50, 60));
    CHECK(sampler.getSample_px() == Eigen::Vector2f(50.5f, 60.5f));
}

TEST_CASE("Test lookAt") {
    using namespace RayTracer;

    Eigen::Affine3f worldFromCamera =
        lookAt_cameraFromWorld(Eigen::Vector3f(-8.f, 0.f, 0.f), Eigen::Vector3f(0.f, 0.f, -8.f),
                               Eigen::Vector3f(0.f, 1.f, 0.f))
            .inverse();

    auto vec_camera = Eigen::Vector3f(0.f, 0.f, -1.f);

    Eigen::Vector3f vec_world = worldFromCamera.rotation() * vec_camera;
    CHECK((vec_world - Eigen::Vector3f(1.f, 0.f, -1.f).normalized()).norm() < epsilon_mm<float>);

    auto pt_camera = Eigen::Vector3f(0.f, 0.f, 0.f);
    auto pt_world = worldFromCamera * pt_camera;
    CHECK((pt_world - Eigen::Vector3f(-8.f, 0.f, 0.f)).norm() < epsilon_mm<float>);
}

TEST_CASE("Test camera ray") {
    using namespace RayTracer;
    // auto worldFromCamera = Eigen::Matrix4f::Identity();
    Eigen::Affine3f worldFromCamera =
        lookAt_cameraFromWorld(Eigen::Vector3f(0.f, 0.f, 0.f), Eigen::Vector3f(0.f, 0.f, -8.f),
                               Eigen::Vector3f(0.f, 1.f, 0.f))
            .inverse();

    auto screenShape = Eigen::Vector2i(100, 100);
    auto camera = Camera(Eigen::Vector3f(0.f, 0.f, 0.f), screenShape, 90.f,
                         Eigen::Vector2f(0, std::numeric_limits<float>::infinity()));

    auto sampler = Sampler();
    ShapeIntersector infinitePlaneIntersector =
        ShapeIntersector::fromImpl<InfiniteXYPlaneIntersector>(-4.f, worldFromCamera,
                                                               Eigen::Affine3f::Identity());

    auto aspectRatio = static_cast<float>(screenShape[0]) / screenShape[1];
    auto rayIncrements =
        Eigen::Vector2f(aspectRatio * 8.f / screenShape.x(), 8.f / screenShape.y());

    // Create BRDF with appropriate values for testing
    BRDF material = {
        .diffuse = Eigen::Vector3f(0.1, 0.8, 0.8),   // Diffuse reflectance (kd)
        .specular = Eigen::Vector3f(0.5, 0.1, 0.5),  // Specular reflectance (ks)
        .ambient = Eigen::Vector3f(0.2, 0.2, 0.1),   // Ambient reflectance (ka)
        .emission = Eigen::Vector3f(0.0, 0.0, 0.0),  // Emission (glow)
        .reflectivity = 0.8f,
        .shininess = 30                              // Shininess (n)
    };
    Eigen::Vector3f sphereCenter = {0.f, 0.f, -8.f};
    auto sphereIntersector = ShapeIntersector::fromImpl<SphereIntersector>(
        Sphere{sphereCenter, 4.f * std::sqrt(2.f)}, Eigen::Affine3f::Identity(),
        Material{material});

    auto aggregatePrimitiveIntersector =
        AggregratePrimitiveIntersector(std::vector{sphereIntersector});

    // Generate rays and test intersections
    for (int i = 0; i < screenShape.prod(); ++i) {
        auto px = Eigen::Vector2i{i % screenShape[0], i / screenShape[0]};
        sampler.setPixel(px);

        // Default sampler samples at the center of the pixel
        Eigen::Vector2f expectedSample_px = px.cast<float>() + Eigen::Vector2f({0.5f, 0.5f});
        CHECK(sampler.getSample_px() == expectedSample_px);

        auto ray_world = camera.getRay_camera(sampler.getSample_px()).transform(worldFromCamera);

        auto intersectPt = infinitePlaneIntersector.intersect(ray_world);

        CHECKED_IF(intersectPt.has_value()) {
            CHECK((intersectPt->point_world -
                   Eigen::Vector3f(rayIncrements[0] * expectedSample_px[0] - aspectRatio * 4.f,
                                   4.f - rayIncrements[1] * expectedSample_px[1], -4.f))
                      .stableNorm() < epsilon_mm<float>);
        }

        intersectPt = aggregatePrimitiveIntersector.intersect(ray_world);

        CHECKED_IF(intersectPt.has_value()) {
            Eigen::Vector3f proj = intersectPt->point_world;
            proj.z() = -4.f;

            auto alpha = intersectPt->normal_world.z();
            alpha = std::pow(alpha, 4);

            CHECK(
                std::abs((intersectPt->point_world - Eigen::Vector3f(0.f, 0.f, -8.f)).stableNorm() -
                         4.f * std::sqrt(2.f)) < epsilon_mm<float>);
            CHECK((proj - Eigen::Vector3f(0.f, 0.f, -4.f)).stableNorm() < 4.f);
        }
    }
}
}  // namespace VdbFields