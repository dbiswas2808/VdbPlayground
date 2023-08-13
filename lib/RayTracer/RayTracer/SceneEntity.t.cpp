#include <RayTracer/SceneEntity.h>
#include <catch2/catch_test_macros.hpp>
#include <RayTracer/Film.h>

namespace VdbFields {
namespace {
class InfiniteXYPlaneIntersector : public RayTracer::ShapeIntersector {
   public:
    explicit InfiniteXYPlaneIntersector(float zoffset_mm, Eigen::Matrix4f worldFromGeom)
        : m_zoffset_mm(zoffset_mm), m_worldFromGeom(worldFromGeom) {}

    [[nodiscard]] virtual std::optional<RayTracer::RayIntersect> intersect(const RayTracer::Ray& ray) const final{
        assert(ray.direction_world.z() > epsilon_mm<float>);
        auto hitT = (m_zoffset_mm - ray.origin_world.z()) / ray.direction_world.z();
        const auto intersectionPt_world = (ray.origin_world + hitT * ray.direction_world).eval();
        return RayTracer::RayIntersect{intersectionPt_world, hitT};
    }
    [[nodiscard]] virtual bool hasIntersection(const RayTracer::Ray& ray) const final { return false; }
    [[nodiscard]] virtual RayTracer::BRDF getBRDF(const RayTracer::RayIntersect& intersect) const final { return {}; }

   private:
    Eigen::Matrix4f m_worldFromGeom;
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
    auto worldFromCamera = Eigen::Matrix4f::Identity();

    auto screenShape = Eigen::Vector2i(250, 250);

    auto camera = RayTracer::Camera(Eigen::Vector3f(0.f, 0.f, 0.f), screenShape, 90.f,
                                    Eigen::Vector2f(0, std::numeric_limits<float>::infinity()),
                                    worldFromCamera);

    auto sampler = RayTracer::Sampler();

    InfiniteXYPlaneIntersector infinitePlaneIntersector(-4.f, Eigen::Matrix4f::Identity());
    auto rayIncrements = Eigen::Vector2f(8.f / screenShape.x(), 8.f / screenShape.y());

    //Sphere center
    Eigen::Vector3f sphereCenter = {0, 0, -8.f};

    RayTracer::SphereIntersector sphereIntersector(
        {{}, sphereCenter, 4.f * std::sqrt(2.f)}, Eigen::Matrix4f::Identity());

    RayTracer::Film film{screenShape};
    for (int i = 0; i < screenShape.prod(); ++i) {
        auto px = Eigen::Vector2i{i % screenShape[0], i / screenShape[0]};
        sampler.setPixel(px);

        // Default sampler samples at the center of the pixel
        Eigen::Vector2f expectedSample_px = px.cast<float>() + Eigen::Vector2f({0.5f, 0.5f});
        CHECK(sampler.getSample_px() == expectedSample_px);

        auto ray = camera.getRay(sampler.getSample_px());

        auto intersectPt =  infinitePlaneIntersector.intersect(ray);
        CHECKED_IF(intersectPt.has_value()) {
            CHECK((intersectPt->point_world - Eigen::Vector3f(rayIncrements[0] * expectedSample_px[0] - 4.f,
                                                              4.f - rayIncrements[1] * expectedSample_px[1],
                                                              -4.f))
                      .stableNorm() < epsilon_mm<float>);
        }

        intersectPt = sphereIntersector.intersect(ray);
        CHECKED_IF(intersectPt.has_value()) {
            Eigen::Vector3f proj = intersectPt->point_world;
            proj.z() = -4.f;

            CHECK(
                std::abs((intersectPt->point_world - Eigen::Vector3f(0.f, 0.f, -8.f)).stableNorm() -
                         4.f * std::sqrt(2.f)) < epsilon_mm<float>);
            CHECK((proj - Eigen::Vector3f(0.f, 0.f, -4.f)).stableNorm() < 4.f);
            film.addSample(px, Eigen::Vector3f(0.5f, 0.5f, 0));
        }
    }
    film.imageToFile("/home/dbiswas2808/Documents/Projects/VdbPlayground/rt_test_images/test.png");
}
}  // namespace VdbFields