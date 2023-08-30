#include <RayTracer/SceneEntity.h>
#include <catch2/catch_test_macros.hpp>
#include <RayTracer/Film.h>
#include <RayTracer/GeometryRayIntersector.h>

namespace VdbFields {
namespace {
class InfiniteXYPlaneIntersector {
   public:
    explicit InfiniteXYPlaneIntersector(float zoffset_mm, Eigen::Matrix4f worldFromCamera,
                                        Eigen::Matrix4f worldFromGeom)
        : m_zoffset_mm(zoffset_mm),
          m_worldFromCamera(worldFromCamera),
          m_worldFromGeom(worldFromGeom) {}

    [[nodiscard]] virtual std::optional<RayTracer::RayIntersect> intersect(
        const RayTracer::Ray& ray_camera) const final {
        auto ray_world = ray_camera.transform(m_worldFromCamera);
        assert(ray.direction_world.z() > epsilon_mm<float>);
        auto hitT = (m_zoffset_mm - ray_world.origin.z()) / ray_world.direction.z();
        const auto intersectionPt_world = (ray_world.origin + hitT * ray_world.direction).eval();
        return RayTracer::RayIntersect{intersectionPt_world, hitT};
    }
    [[nodiscard]] virtual bool hasIntersection(const RayTracer::Ray& ray) const final {
        return false;
    }
    [[nodiscard]] virtual RayTracer::BRDF getBRDF(
        const RayTracer::RayIntersect& intersect) const final {
        return {};
    }

   private:
    Eigen::Matrix4f m_worldFromGeom;
    Eigen::Matrix4f m_worldFromCamera;
    float m_zoffset_mm;
};
}  // namespace

template <class T>
inline constexpr std::enable_if_t<std::is_floating_point_v<T>, T> epsilon_mm;

template <>
inline constexpr float epsilon_mm<float> = 0.0001f;

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

TEST_CASE("Test camera ray") {
    using namespace RayTracer;
    // auto worldFromCamera = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f worldFromCamera =
        lookAt_cameraFromWorld(Eigen::Vector3f(0.f, 0.f, 0.f), Eigen::Vector3f(0.f, 0.f, -1.f),
                               Eigen::Vector3f(0.f, 1.f, 0.f))
            .inverse();

    auto screenShape = Eigen::Vector2i(250, 125);
    auto camera = Camera(Eigen::Vector3f(0.f, 0.f, 0.f), screenShape, 90.f,
                         Eigen::Vector2f(0, std::numeric_limits<float>::infinity()));

    auto sampler = Sampler();
    ShapeIntersector infinitePlaneIntersector =
        ShapeIntersector::fromImpl<InfiniteXYPlaneIntersector>(-4.f, worldFromCamera, Eigen::Matrix4f::Identity());

    auto aspectRatio = static_cast<float>(screenShape[0]) / screenShape[1];
    auto rayIncrements =
        Eigen::Vector2f(aspectRatio * 8.f / screenShape.x(), 8.f / screenShape.y());

    // Sphere center
    Eigen::Vector3f sphereCenter = {0, 0, -8.f};
    auto sphereIntersector = ShapeIntersector::fromImpl<SphereIntersector>(
        Sphere{sphereCenter, 4.f * std::sqrt(2.f)}, worldFromCamera, Eigen::Matrix4f::Identity());

    Film film{screenShape};
    for (int i = 0; i < screenShape.prod(); ++i) {
        auto px = Eigen::Vector2i{i % screenShape[0], i / screenShape[0]};
        sampler.setPixel(px);

        // Default sampler samples at the center of the pixel
        Eigen::Vector2f expectedSample_px = px.cast<float>() + Eigen::Vector2f({0.5f, 0.5f});
        CHECK(sampler.getSample_px() == expectedSample_px);

        auto ray_camera = camera.getRay_camera(sampler.getSample_px());

        auto intersectPt = infinitePlaneIntersector.intersect(ray_camera);
        CHECKED_IF(intersectPt.has_value()) {
            CHECK((intersectPt->point_world -
                   Eigen::Vector3f(rayIncrements[0] * expectedSample_px[0] - aspectRatio * 4.f,
                                   4.f - rayIncrements[1] * expectedSample_px[1], -4.f))
                      .stableNorm() < epsilon_mm<float>);
        }

        intersectPt = sphereIntersector.intersect(ray_camera);
        CHECKED_IF(intersectPt.has_value()) {
            Eigen::Vector3f proj = intersectPt->point_world;
            proj.z() = -4.f;

            auto alpha = intersectPt->normal_world.z();
            alpha = std::pow(alpha, 4);

            CHECK(
                std::abs((intersectPt->point_world - Eigen::Vector3f(0.f, 0.f, -8.f)).stableNorm() -
                         4.f * std::sqrt(2.f)) < epsilon_mm<float>);
            CHECK((proj - Eigen::Vector3f(0.f, 0.f, -4.f)).stableNorm() < 4.f);
            film.addSample(px, alpha  * Eigen::Vector3f(0.f, 1.f, 0.f) +  (1 - alpha)  * Eigen::Vector3f(1.f, 0.f, 0.f));
        }
    }
    film.imageToFile("/home/dbiswas2808/Documents/Projects/VdbPlayground/rt_test_images/test.png");
}
}  // namespace VdbFields