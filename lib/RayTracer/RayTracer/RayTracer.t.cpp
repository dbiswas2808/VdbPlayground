#include <RayTracer/RayTracer.h>

#include <iostream>

#include <catch2/catch_test_macros.hpp>
#include <RayTracer/Film.h>
#include <RayTracer/GeometryRayIntersector.h>
#include <RayTracer/SceneEntity.h>

namespace VdbFields {
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

    auto aspectRatio = static_cast<float>(screenShape[0]) / screenShape[1];

    // Create BRDF with appropriate values for testing
    BRDF material1 = {
        .diffuse = Eigen::Vector3f(0.1, 0.8, 0.8),   // Diffuse reflectance (kd)
        .specular = Eigen::Vector3f(0.5, 0.1, 0.5),  // Specular reflectance (ks)
        .ambient = Eigen::Vector3f(0.2, 0.2, 0.1),   // Ambient reflectance (ka)
        .emission = Eigen::Vector3f(0.0, 0.0, 0.0),  // Emission (glow)
        .shininess = 30                              // Shininess (n)
    };
    BRDF material2 = {
        .diffuse = Eigen::Vector3f(0.8, 0.1, 0.8),   // Diffuse reflectance (kd)
        .specular = Eigen::Vector3f(0.1, 0.5, 0.5),  // Specular reflectance (ks)
        .ambient = Eigen::Vector3f(0.1, 0.2, 0.2),   // Ambient reflectance (ka)
        .emission = Eigen::Vector3f(0.0, 0.0, 0.0),  // Emission (glow)
        .shininess = 30                              // Shininess (n)
    };
    BRDF material3 = {
        .diffuse = Eigen::Vector3f(0.8, 0.8, 0.1),   // Diffuse reflectance (kd)
        .specular = Eigen::Vector3f(0.5, 0.1, 0.2),  // Specular reflectance (ks)
        .ambient = Eigen::Vector3f(0.8, 0.2, 0.2),   // Ambient reflectance (ka)
        .emission = Eigen::Vector3f(0.0, 0.0, 0.0),  // Emission (glow)
        .shininess = 30                              // Shininess (n)
    };
    BRDF material4 = {
        .diffuse = Eigen::Vector3f(0.5, 0.6, 0.9),   // Diffuse reflectance (kd)
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

    // Generate rays and test intersections
    Film film{screenShape};
    auto rayTracer = RayTracerImpl(AggregratePrimitiveIntersector(aggregatePrimitiveIntersector),
                                   std::move(lights));
    for (int i = 0; i < screenShape.prod(); ++i) {
        auto px = Eigen::Vector2i{i % screenShape[0], i / screenShape[0]};
        sampler.setPixel(px);

        if (i % 10'000 == 0) {
            std::cout << "progress: " << float(i) / screenShape.prod() * 100.f << std::endl;
        }

        auto ray_world = camera.getRay_camera(sampler.getSample_px()).transform(worldFromCamera);

        // Generate a test image
        if (auto intersectPt = aggregatePrimitiveIntersector.intersect(ray_world)) {
            auto color = rayTracer.rayTrace(ray_world);
            film.addSample(px, color);
        }
    }
    film.imageToFile("/home/dbiswas2808/Documents/Projects/VdbPlayground/rt_test_images/test.png");
}
}