#include <RayTracer/Scene.h>
#include <iostream>
#include <random>
#include <catch2/catch_test_macros.hpp>

#pragma GCC optimize("O0")
namespace VdbFields {
namespace {
struct UniformRandomSampler {
    UniformRandomSampler() : gen(std::random_device()()), dis(0.5, 0.5 / 6) {}

    [[nodiscard]] Eigen::Vector2f operator()() const {
        return m_pixel.cast<float>() +
               Eigen::Vector2f(std::clamp(dis(gen), 0.1f, 0.9f), std::clamp(dis(gen), 0.1f, 0.9f));
    }
    void setPixel(Eigen::Vector2i pixel) { m_pixel = pixel; }
    Eigen::Vector2i m_pixel;
    mutable std::mt19937 gen;
    mutable std::normal_distribution<float> dis; 

};
}

TEST_CASE("Test full ray tracing") {
    using namespace RayTracer;
    // auto worldFromCamera = Eigen::Matrix4f::Identity();
    Eigen::Affine3f worldFromCamera =
        lookAt_cameraFromWorld(Eigen::Vector3f(0.f, 0.f, 0.f), Eigen::Vector3f(0.f, 0.f, -8.f),
                               Eigen::Vector3f(0.f, 1.f, 0.f))
            .inverse();

    auto screenShape = Eigen::Vector2i(10'000, 10'000);
    auto camera = Camera(Eigen::Vector3f(0.f, 0.f, 0.f), screenShape, 90.f,
                         Eigen::Vector2f(0, std::numeric_limits<float>::infinity()));
    auto sampler = Sampler();

    // Create BRDF with appropriate values for testing
    BRDF material1 = {
        .diffuse = Eigen::Vector3f(10, 3.8, 8.8),   // Diffuse reflectance (kd)
        .specular = Eigen::Vector3f(0.5, 0.1, 0.5),  // Specular reflectance (ks)
        .ambient = Eigen::Vector3f(0.2, 0.2, 0.1),   // Ambient reflectance (ka)
        .emission = Eigen::Vector3f(0.0, 0.0, 0.0),  // Emission (glow)
        .shininess = 30                              // Shininess (n)
    };
    BRDF material2 = {
        .diffuse = Eigen::Vector3f(6.8, 10, 5.8),   // Diffuse reflectance (kd)
        .specular = Eigen::Vector3f(0.1, 0.5, 0.5),  // Specular reflectance (ks)
        .ambient = Eigen::Vector3f(0.1, 0.2, 0.2),   // Ambient reflectance (ka)
        .emission = Eigen::Vector3f(0.0, 0.0, 0.0),  // Emission (glow)
        .shininess = 30                              // Shininess (n)
    };
    BRDF material3 = {
        .diffuse = Eigen::Vector3f(6.8, 10.8, 10),   // Diffuse reflectance (kd)
        .specular = Eigen::Vector3f(0.5, 0.1, 0.2),  // Specular reflectance (ks)
        .ambient = Eigen::Vector3f(0.8, 0.2, 0.2),   // Ambient reflectance (ka)
        .emission = Eigen::Vector3f(0.0, 0.0, 0.0),  // Emission (glow)
        .shininess = 30                              // Shininess (n)
    };
    BRDF material4 = {
        .diffuse = Eigen::Vector3f(10, 10.6, 6.9),   // Diffuse reflectance (kd)
        .specular = Eigen::Vector3f(0.5, 0.6, 0.9),  // Specular reflectance (ks)
        .ambient = Eigen::Vector3f(0.8, 0.5, 0.6),   // Ambient reflectance (ka)
        .emission = Eigen::Vector3f(0.0, 0.8, 0.9),  // Emission (glow)
        .shininess = 60                              // Shininess (n)
    };

    Eigen::Vector3f sphereCenter1 = {0.f, -8.f, -16.f};
    Eigen::Vector3f sphereCenter2 = {0.f, 8.f, -16.f};
    Eigen::Vector3f sphereCenter3 = {-16.f, 0.f, -40.f};
    Eigen::Vector3f sphereCenter4 = {50.f, 2.f, -100.f};

    auto sphereIntersector1 = ShapeIntersector::fromImpl<SphereIntersector>(
        Sphere{sphereCenter1, 4.f * std::sqrt(2.f)}, Eigen::Affine3f::Identity(),
        Material{material1});
    auto sphereIntersector2 = ShapeIntersector::fromImpl<SphereIntersector>(
        Sphere{sphereCenter2, 4.f * std::sqrt(2.f)}, Eigen::Affine3f::Identity(),
        Material{material2});
    auto sphereIntersector3 = ShapeIntersector::fromImpl<SphereIntersector>(
        Sphere{sphereCenter3, 8.f}, Eigen::Affine3f::Identity(), Material{material3});
    auto sphereIntersector4 = ShapeIntersector::fromImpl<SphereIntersector>(
        Sphere{sphereCenter4, 30.f}, Eigen::Affine3f::Identity(), Material{material4});

    auto aggregatePrimitiveIntersector = AggregratePrimitiveIntersector(std::vector{
        sphereIntersector1, sphereIntersector2, sphereIntersector3, sphereIntersector4});

    auto lightDir1 =
        Light::fromImpl<DirectionalLight>(Eigen::Vector3f(-1.f, 1.f, 1.f).normalized(), 10.f);
    auto lightDir2 =
        Light::fromImpl<DirectionalLight>(Eigen::Vector3f(1.f, -1.f, 1.f).normalized(), 10.f);
    auto lightPoint1 =
        Light::fromImpl<PointLight>(Eigen::Vector3f(-16.f, 0.f, 16.f).normalized(), 10.f);

    std::vector<Light> lights = {lightDir1, lightDir2, lightPoint1};
    
    auto scene = Scene(screenShape, sampler, camera, std::vector{
        sphereIntersector1, sphereIntersector2, sphereIntersector3, sphereIntersector4}, std::move(lights));
    scene.rayTrace([](int num, int den) {
        if (num % 10'000 == 0) {
            std::cout << "progress: " << static_cast<float>(num) / den << std::endl;
        }
    });
    scene.writRayTracedImageToFile("/home/dbiswas2808/Documents/Projects/VdbPlayground/rt_test_images/test.png");
}

TEST_CASE("Full ray tracing on tri mesh") {

    using namespace RayTracer;
    // auto worldFromCamera = Eigen::Matrix4f::Identity();
    Eigen::Affine3f worldFromCamera =
        lookAt_cameraFromWorld(Eigen::Vector3f(0.f, 0.f, 0.f), Eigen::Vector3f(0.f, 0.f, -8.f),
                               Eigen::Vector3f(0.f, 1.f, 0.f))
            .inverse();

    auto screenShape = Eigen::Vector2i(10'000, 10'000);
    auto camera = Camera(Eigen::Vector3f(0.f, 0.f, 0.f), screenShape, 90.f,
                         Eigen::Vector2f(0, std::numeric_limits<float>::infinity()));
    auto sampler = Sampler<UniformRandomSampler>();

    // Create BRDF with appropriate values for testing
    BRDF material1 = {
        .diffuse = Eigen::Vector3f(10, 3.8, 8.8),   // Diffuse reflectance (kd)
        .specular = Eigen::Vector3f(0.5, 0.1, 0.5),  // Specular reflectance (ks)
        .ambient = Eigen::Vector3f(0.2, 0.2, 0.1),   // Ambient reflectance (ka)
        .emission = Eigen::Vector3f(0.0, 0.0, 0.0),  // Emission (glow)
        .shininess = 30                              // Shininess (n)
    };

    BRDF material2 = {
        .diffuse = Eigen::Vector3f(6.8, 10, 5.8),   // Diffuse reflectance (kd)
        .specular = Eigen::Vector3f(0.1, 0.5, 0.5),  // Specular reflectance (ks)
        .ambient = Eigen::Vector3f(0.1, 0.2, 0.2),   // Ambient reflectance (ka)
        .emission = Eigen::Vector3f(0.0, 0.0, 0.0),  // Emission (glow)
        .shininess = 30                              // Shininess (n)
    };

    BRDF material4 = {
        .diffuse = Eigen::Vector3f(10, 10.6, 6.9),   // Diffuse reflectance (kd)
        .specular = Eigen::Vector3f(0.5, 0.6, 0.9),  // Specular reflectance (ks)
        .ambient = Eigen::Vector3f(0.8, 0.5, 0.6),   // Ambient reflectance (ka)
        .emission = Eigen::Vector3f(0.0, 0.8, 0.9),  // Emission (glow)
        .shininess = 60                              // Shininess (n)
    };

    // Sample 5 vertices of pyramid mesh test case
    std::vector<Eigen::Vector3f> tris = {
        Eigen::Vector3f::Zero(),   Eigen::Vector3f(1, -1, -1),  Eigen::Vector3f(1, 1, -1),
        Eigen::Vector3f::Zero(),   Eigen::Vector3f(1, 1, -1),   Eigen::Vector3f(-1, 1, -1),
        Eigen::Vector3f::Zero(),   Eigen::Vector3f(-1, 1, -1),  Eigen::Vector3f(-1, -1, -1),
        Eigen::Vector3f::Zero(),   Eigen::Vector3f(-1, -1, -1), Eigen::Vector3f(1, -1, -1),
        Eigen::Vector3f(1, 1, -1), Eigen::Vector3f(-1, -1, -1), Eigen::Vector3f(-1, 1, -1),
        Eigen::Vector3f(1, 1, -1), Eigen::Vector3f(1, -1, -1), Eigen::Vector3f(-1, -1, -1)};

    auto pyramid1 = ShapeIntersector::fromImpl<TriMeshIntersector>(
        tris, Eigen::Affine3f(Eigen::Translation3f(0.f, 0.f, -8.f)), Material{material1});

    auto pyramid2 = ShapeIntersector::fromImpl<TriMeshIntersector>(
        tris, Eigen::Affine3f(Eigen::Translation3f(-1.f, 0.f, -5.f)), Material{material2});

    Eigen::Vector3f sphereCenter4 = {20.f, 2.f, -100.f};
    auto sphereIntersector4 = ShapeIntersector::fromImpl<SphereIntersector>(
        Sphere{sphereCenter4, 30.f}, Eigen::Affine3f::Identity(), Material{material4});

    auto lightDir1 =
        Light::fromImpl<DirectionalLight>(Eigen::Vector3f(-1.f, 1.f, 1.f).normalized(), 10.f);
    auto lightDir2 =
        Light::fromImpl<DirectionalLight>(Eigen::Vector3f(1.f, -1.f, 1.f).normalized(), 10.f);
    auto lightPoint1 =
        Light::fromImpl<PointLight>(Eigen::Vector3f(-16.f, 0.f, 16.f).normalized(), 10.f);

    std::vector<Light> lights = {lightDir1, lightDir2, lightPoint1};

    auto scene = Scene(screenShape, sampler, camera,
                       std::vector{pyramid1, pyramid2, sphereIntersector4}, std::move(lights));
    scene.rayTrace(
        [](int num, int den) {
            if (num % 10'000 == 0) {
                std::cout << "progress: " << static_cast<float>(num) / den << std::endl;
            }
        },
        10);

    scene.writRayTracedImageToFile("/home/dbiswas2808/Documents/Projects/VdbPlayground/rt_test_images/pyramid_test.png");
}
}  // namespace VdbFields