#include <RayTracer/Scene.h>
#include <iostream>
#include <random>
#include <mutex>
#include <catch2/catch_test_macros.hpp>

#include <MeshFileReaders/StlFileReader.h>

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

[[nodiscard]] std::vector<Eigen::Vector3f> makeVertexTestNormals(std::span<const Eigen::Vector3f> vertices) {
    std::vector<Eigen::Vector3f> normals;
    for (size_t ii = 0; ii < vertices.size(); ii+=3) {
        auto n = normal(vertices[ii], vertices[ii + 1], vertices[ii + 2]);
        normals.push_back(n);
        normals.push_back(n);
        normals.push_back(n);
    }

    return normals;
}

[[nodiscard]] std::vector<Eigen::Vector2f> makeTestTexCoords(
    std::span<const Eigen::Vector3f> vertices) {
    return std::vector<Eigen::Vector2f>(vertices.size(), Eigen::Vector2f::Zero());
}
}  // namespace

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

    // Create BRDF with appropriate values for testing
    BRDF material1 = {
        .diffuse = Eigen::Vector3f(10, 3.8, 8.8),   // Diffuse reflectance (kd)
        .specular = Eigen::Vector3f(0.5, 0.1, 0.5),  // Specular reflectance (ks)
        .ambient = Eigen::Vector3f(0.2, 0.2, 0.1),   // Ambient reflectance (ka)
        .emission = Eigen::Vector3f(0.0, 0.0, 0.0),  // Emission (glow)
        .reflectivity = 0.8f,
        .shininess = 30                              // Shininess (n)
    };
    BRDF material2 = {
        .diffuse = Eigen::Vector3f(6.8, 10, 5.8),   // Diffuse reflectance (kd)
        .specular = Eigen::Vector3f(0.1, 0.5, 0.5),  // Specular reflectance (ks)
        .ambient = Eigen::Vector3f(0.1, 0.2, 0.2),   // Ambient reflectance (ka)
        .emission = Eigen::Vector3f(0.0, 0.0, 0.0),  // Emission (glow)
        .reflectivity = 0.8f,
        .shininess = 30                              // Shininess (n)
    };
    BRDF material3 = {
        .diffuse = Eigen::Vector3f(6.8, 10.8, 10),   // Diffuse reflectance (kd)
        .specular = Eigen::Vector3f(0.5, 0.1, 0.2),  // Specular reflectance (ks)
        .ambient = Eigen::Vector3f(0.8, 0.2, 0.2),   // Ambient reflectance (ka)
        .emission = Eigen::Vector3f(0.0, 0.0, 0.0),  // Emission (glow)
        .reflectivity = 0.8f,
        .shininess = 30                              // Shininess (n)
    };
    BRDF material4 = {
        .diffuse = Eigen::Vector3f(10, 10.6, 6.9),   // Diffuse reflectance (kd)
        .specular = Eigen::Vector3f(0.5, 0.6, 0.9),  // Specular reflectance (ks)
        .ambient = Eigen::Vector3f(0.8, 0.5, 0.6),   // Ambient reflectance (ka)
        .emission = Eigen::Vector3f(0.0, 0.8, 0.9),  // Emission (glow)
        .reflectivity = 0.8f,
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

    auto scene = Scene<Sampler<>>(
        screenShape, camera,
        std::vector{sphereIntersector1, sphereIntersector2, sphereIntersector3, sphereIntersector4},
        std::move(lights), worldFromCamera);
    scene.rayTrace([](int num, int den) {
        if (num % 10'000 == 0) {
            std::cout << "progress: " << static_cast<float>(num) / den << std::endl;
        }
    });
    scene.writRayTracedImageToFile(
        "/home/dbiswas2808/Documents/Projects/VdbPlayground/rt_test_images/spheres_test.png");
}

TEST_CASE("Full ray tracing on tri mesh") {

    using namespace RayTracer;
    // auto worldFromCamera = Eigen::Matrix4f::Identity();
    Eigen::Affine3f worldFromCamera =
        lookAt_cameraFromWorld(Eigen::Vector3f(0.f, 0.f, 0.f), Eigen::Vector3f(0.f, 0.f, -1.f),
                               Eigen::Vector3f(0.f, 1.f, 0.f))
            .inverse();

    auto screenShape = Eigen::Vector2i(10'000, 10'000);
    auto camera = Camera(Eigen::Vector3f(0.f, 0.f, 0.f), screenShape, 90.f,
                         Eigen::Vector2f(0, std::numeric_limits<float>::infinity()));

    
    auto lightDir1 =
        Light::fromImpl<DirectionalLight>(Eigen::Vector3f(-1.f, 1.f, 1.f).normalized(), 10.f);
    auto lightDir2 =
        Light::fromImpl<DirectionalLight>(Eigen::Vector3f(1.f, -1.f, 1.f).normalized(), 10.f);
    auto lightPoint1 =
        Light::fromImpl<PointLight>(Eigen::Vector3f(-16.f, 0.f, 16.f).normalized(), 10.f);

    std::vector<Light> lights{lightDir1, lightDir2, lightPoint1};

    // Create BRDF with appropriate values for testing
    BRDF material1 = {
        .diffuse = Eigen::Vector3f(10, 3.8, 8.8),   // Diffuse reflectance (kd)
        .specular = Eigen::Vector3f(0.5, 0.1, 0.5),  // Specular reflectance (ks)
        .ambient = Eigen::Vector3f(0.2, 0.2, 0.1),   // Ambient reflectance (ka)
        .emission = Eigen::Vector3f(0.0, 0.0, 0.0),  // Emission (glow)
        .reflectivity = 0.8f,
        .shininess = 30                              // Shininess (n)
    };

    BRDF material2 = {
        .diffuse = Eigen::Vector3f(6.8, 10, 5.8),   // Diffuse reflectance (kd)
        .specular = Eigen::Vector3f(0.1, 0.5, 0.5),  // Specular reflectance (ks)
        .ambient = Eigen::Vector3f(0.1, 0.2, 0.2),   // Ambient reflectance (ka)
        .emission = Eigen::Vector3f(0.0, 0.0, 0.0),  // Emission (glow)
        .reflectivity = 0.8f,
        .shininess = 30                              // Shininess (n)
    };

    BRDF material4 = {
        .diffuse = Eigen::Vector3f(10, 10.6, 6.9),   // Diffuse reflectance (kd)
        .specular = Eigen::Vector3f(0.5, 0.6, 0.9),  // Specular reflectance (ks)
        .ambient = Eigen::Vector3f(0.8, 0.5, 0.6),   // Ambient reflectance (ka)
        .emission = Eigen::Vector3f(0.0, 0.8, 0.9),  // Emission (glow)
        .reflectivity = 0.8f,
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
        tris, makeVertexTestNormals(tris), makeTestTexCoords(tris),
        Eigen::Affine3f(Eigen::Translation3f(0.f, 0.f, -8.f)), Material{material1});

    auto pyramid2 = ShapeIntersector::fromImpl<TriMeshIntersector>(
        tris, makeVertexTestNormals(tris), makeTestTexCoords(tris),
        Eigen::Affine3f(Eigen::Translation3f(-1.f, 0.f, -5.f)), Material{material2});

    Eigen::Vector3f sphereCenter4 = {20.f, 2.f, -100.f};
    auto sphereIntersector4 = ShapeIntersector::fromImpl<SphereIntersector>(
        Sphere{sphereCenter4, 30.f}, Eigen::Affine3f::Identity(), Material{material4});

    auto scene = Scene<Sampler<UniformRandomSampler>>(
        screenShape, camera, std::vector{pyramid1, pyramid2, sphereIntersector4}, std::move(lights),
        worldFromCamera);

    std::atomic<int> atomicProgress;
    std::mutex m;
    auto t_start = std::chrono::high_resolution_clock::now();
    scene.rayTrace(
        [&atomicProgress, &m](int num, int den) {
            atomicProgress++;
            if (auto val = atomicProgress.load(); val % 40'000 == 0) {
                std::scoped_lock lock(m);
                std::cout << "progress: " << static_cast<float>(val) / den << std::endl;
            }
        },
        16);

    auto t_end = std::chrono::high_resolution_clock::now();

    std::cout << "Elapsed time: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count()
              << " ms\n";

    scene.writRayTracedImageToFile(
        "/home/dbiswas2808/Documents/Projects/VdbPlayground/rt_test_images/"
        "pyramid_spheres_test.png");
}

TEST_CASE("Full ray tracing with TLAS") {
    using namespace RayTracer;
    Eigen::Affine3f worldFromCamera =
        lookAt_cameraFromWorld(Eigen::Vector3f(0., 0., 6.),
                               Eigen::Vector3f(0.f, 0.f, -6.f), Eigen::Vector3f(0.f, 1.f, 0.f))
            .inverse();

    auto screenShape = Eigen::Vector2i(10'000, 10'000);
    auto camera = Camera(Eigen::Vector3f(0.f, 0.f, 0.f), screenShape, 90.f,
                         Eigen::Vector2f(0, std::numeric_limits<float>::infinity()));

    auto lightDir1 =
        Light::fromImpl<DirectionalLight>(Eigen::Vector3f(-1.f, 1.f, 0.f).normalized(), 10.f);
    auto lightDir2 =
        Light::fromImpl<DirectionalLight>(Eigen::Vector3f(1.f, -1.f, 1.f).normalized(), 10.f);
    auto lightDir5 =
        Light::fromImpl<DirectionalLight>(Eigen::Vector3f(-1.f, -1.f, 1.f).normalized(), 10.f);
    auto lightDir4 =
        Light::fromImpl<DirectionalLight>(Eigen::Vector3f(1.f, -1.f, -1.f).normalized(), 10.f);
    auto lightDir3 =
        Light::fromImpl<DirectionalLight>(Eigen::Vector3f(0.f, 0.f, -1.f).normalized(), 10.f);
    auto lightPoint1 =
        Light::fromImpl<PointLight>(Eigen::Vector3f(-0.f, 0.f, 0.f).normalized(), 10.f);
    auto lightPoint2 =
        Light::fromImpl<PointLight>(Eigen::Vector3f(5.f, 5.f, -2.f).normalized(), 10.f);

    std::vector<Light> lights{lightDir3, lightDir2, lightDir1,  lightPoint1,
                              lightDir4, lightDir5, lightPoint2};

    // Create BRDF with appropriate values for testing
    BRDF material1 = {
        .diffuse = Eigen::Vector3f(10, 10, 10),      // Diffuse reflectance (kd)
        .specular = Eigen::Vector3f(10., 0.1, 1.5),  // Specular reflectance (ks)
        .ambient = Eigen::Vector3f(0.2, 0.2, 0.1),   // Ambient reflectance (ka)
        .emission = Eigen::Vector3f(8.0, 8.0, 8.0),  // Emission (glow)
        .reflectivity = 0.8f,
        .shininess = 30  // Shininess (n)
    };

    BRDF material2 = {
        .diffuse = Eigen::Vector3f(2.8, 3, 10.8),    // Diffuse reflectance (kd)
        .specular = Eigen::Vector3f(9.1, 0.5, 0.5),  // Specular reflectance (ks)
        .ambient = Eigen::Vector3f(0.1, 0.2, 0.2),   // Ambient reflectance (ka)
        .emission = Eigen::Vector3f(4.0, 5.0, 7.0),  // Emission (glow)
        .reflectivity = 0.5f,
        .shininess = 30  // Shininess (n)
    };

    BRDF material3 = {
        .diffuse = Eigen::Vector3f(1, 10.6, 6.9),     // Diffuse reflectance (kd)
        .specular = Eigen::Vector3f(0.5, 0.6, 11.9),  // Specular reflectance (ks)
        .ambient = Eigen::Vector3f(0.8, 0.5, 0.6),    // Ambient reflectance (ka)
        .emission = Eigen::Vector3f(4.0, 8.0, 4.0),   // Emission (glow)
        .reflectivity = 0.8f,
        .shininess = 30  // Shininess (n)
    };

    BRDF material4 = {
        .diffuse = Eigen::Vector3f(9, 10.6, 6.9),     // Diffuse reflectance (kd)
        .specular = Eigen::Vector3f(0.5, 0.6, 11.9),  // Specular reflectance (ks)
        .ambient = Eigen::Vector3f(0.8, 0.5, 0.6),    // Ambient reflectance (ka)
        .emission = Eigen::Vector3f(4.0, 8.0, 4.0),   // Emission (glow)
        .reflectivity = 0.3f,
        .shininess = 30  // Shininess (n)
    };

    BRDF material5 = {
        .diffuse = Eigen::Vector3f(7, 2.6, 6.9),      // Diffuse reflectance (kd)
        .specular = Eigen::Vector3f(0.5, 0.6, 11.9),  // Specular reflectance (ks)
        .ambient = Eigen::Vector3f(0.8, 0.5, 0.6),    // Ambient reflectance (ka)
        .emission = Eigen::Vector3f(4.0, 8.0, 4.0),   // Emission (glow)
        .reflectivity = 0.15f,
        .shininess = 30  // Shininess (n)
    };

    // Sample 5 vertices of pyramid mesh test case
    std::vector<Eigen::Vector3f> tris = {
        Eigen::Vector3f::Zero(),         3 * Eigen::Vector3f(1, -1, -1),
        3 * Eigen::Vector3f(1, 1, -1),   Eigen::Vector3f::Zero(),
        3 * Eigen::Vector3f(1, 1, -1),   3 * Eigen::Vector3f(-1, 1, -1),
        Eigen::Vector3f::Zero(),         3 * Eigen::Vector3f(-1, 1, -1),
        3 * Eigen::Vector3f(-1, -1, -1), Eigen::Vector3f::Zero(),
        3 * Eigen::Vector3f(-1, -1, -1), 3 * Eigen::Vector3f(1, -1, -1),
        3 * Eigen::Vector3f(1, 1, -1),   3 * Eigen::Vector3f(-1, -1, -1),
        3 * Eigen::Vector3f(-1, 1, -1),  3 * Eigen::Vector3f(1, 1, -1),
        3 * Eigen::Vector3f(1, -1, -1),  3 * Eigen::Vector3f(-1, -1, -1)};

    std::vector<Eigen::Vector3f> trisTest1 = {
        Eigen::Vector3f::Zero(), 3 * Eigen::Vector3f(1, -1, -1), 3 * Eigen::Vector3f(1, 1, -1)};
    std::vector<Eigen::Vector3f> trisTest2 = {
        Eigen::Vector3f::Zero(), 3 * Eigen::Vector3f(-1, 1, -1), 3 * Eigen::Vector3f(-1, -1, -1)};

    Eigen::Vector3f sphereCenter4 = {-0.f, 0.f, -6.f};
    auto sphereIntersector4 = ShapeIntersector::fromImpl<SphereIntersector>(
        Sphere{sphereCenter4, 1.f}, Eigen::Affine3f::Identity(), Material{material3});

    auto sphereIntersector5 = ShapeIntersector::fromImpl<SphereIntersector>(
        Sphere{sphereCenter4, 1.3f}, Eigen::Affine3f(Eigen::Translation3f(0.f, 3.5f, 0.f)),
        Material{material1});

    auto sphereIntersector6 = ShapeIntersector::fromImpl<SphereIntersector>(
        Sphere{sphereCenter4, 1.7f}, Eigen::Affine3f(Eigen::Translation3f(0.f, -3.5f, 0.f)),
        Material{material5});

    AggregateMeshIntersector aggMeshIntersector;
    aggMeshIntersector.addMeshIntersector(
        BVHMesh::makeMesh(tris, makeVertexTestNormals(tris), makeTestTexCoords(tris),
                          Material{material2}),
        Eigen::Affine3f(Eigen::Translation3f(-4.f, -0.8f, -6.f)));
    aggMeshIntersector.addMeshIntersector(
        BVHMesh::makeMesh(tris, makeVertexTestNormals(tris), makeTestTexCoords(tris),
                          Material{material4}),
        Eigen::Affine3f(Eigen::Translation3f(4.f, 0.8f, -6.f)));
    aggMeshIntersector.buildTlas();

    auto multiMeshIntersector =
        ShapeIntersector::fromImpl<AggregateMeshIntersector>(std::move(aggMeshIntersector));
    auto scene =
        Scene<Sampler<UniformRandomSampler>>(screenShape, camera,
                                             std::vector{multiMeshIntersector, sphereIntersector4,
                                                         sphereIntersector5, sphereIntersector6},
                                             std::move(lights), worldFromCamera);

    std::atomic<int> atomicProgress;
    std::mutex m;
    auto t_start = std::chrono::high_resolution_clock::now();
    scene.rayTrace(
        [&atomicProgress, &m](int num, int den) {
            atomicProgress++;
            if (auto val = atomicProgress.load(); val % 40'000 == 0) {
                std::scoped_lock lock(m);
                std::cout << "progress: " << static_cast<float>(val) / den << std::endl;
            }
        },
        16);

    auto t_end = std::chrono::high_resolution_clock::now();

    std::cout << "Elapsed time: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count()
              << " ms\n";

    scene.writRayTracedImageToFile(
        "/home/dbiswas2808/Documents/Projects/VdbPlayground/rt_test_images/"
        "TLAS_pyramid_spheres_test.png");
}

TEST_CASE("Full ray tracing On mesh") {
    using namespace RayTracer;
    auto delta = 4.f;
    Eigen::Affine3f worldFromCamera =
        lookAt_cameraFromWorld(Eigen::Vector3f(0., 3, 1.),
                               Eigen::Vector3f(0.f, 0.f, -6.f), Eigen::Vector3f(0.f, 1.f, 0.f))
            .inverse();

    auto screenShape = Eigen::Vector2i(10'000, 10'000);
    auto camera = Camera(Eigen::Vector3f(0.f, 0.f, 0.f), screenShape, 90.f,
                         Eigen::Vector2f(0, std::numeric_limits<float>::infinity()));

    auto lightDir1 =
        Light::fromImpl<DirectionalLight>(Eigen::Vector3f(-1.f, 1.f, 0.f).normalized(), 10.f);
    auto lightDir2 =
        Light::fromImpl<DirectionalLight>(Eigen::Vector3f(1.f, -1.f, 1.f).normalized(), 10.f);
    auto lightDir5 =
        Light::fromImpl<DirectionalLight>(Eigen::Vector3f(-1.f, -1.f, 1.f).normalized(), 10.f);
    auto lightDir4 =
        Light::fromImpl<DirectionalLight>(Eigen::Vector3f(1.f, -1.f, -1.f).normalized(), 10.f);
    auto lightDir3 =
        Light::fromImpl<DirectionalLight>(Eigen::Vector3f(0.f, 0.f, -1.f).normalized(), 10.f);
    auto lightDir6 =
        Light::fromImpl<DirectionalLight>(Eigen::Vector3f(0.f, -1.f, -1.f).normalized(), 10.f);
    auto lightPoint1 =
        Light::fromImpl<PointLight>(Eigen::Vector3f(-0.f, 0.f, 0.f).normalized(), 10.f);
    auto lightPoint2 =
        Light::fromImpl<PointLight>(Eigen::Vector3f(5.f, 5.f, -2.f).normalized(), 10.f);

    std::vector<Light> lights{lightDir3, lightDir2, lightDir1, lightPoint1,
                              lightDir4, lightDir5, lightDir6, lightPoint2};

    auto meshes = MeshFileReaders::readObjFile(
        "/home/dbiswas2808/Documents/Projects/VdbPlayground/lib/RayTracer/test_model/705A.obj");

    auto eigenFromArray = [](auto arr) {
        return Eigen::Vector3f(arr[0], arr[1], arr[2]);
    };
    std::vector<Eigen::Vector3f> tris;
    std::vector<Eigen::Vector3f> normals;

    AABB aabb;
    for (const auto& mesh : meshes) {
        for (auto f : mesh.faces) {
            auto [idx0, idx1, idx2] = f;
            tris.push_back(eigenFromArray(mesh.vertices[idx0]));
            aabb.extend(tris.back());
            normals.push_back(eigenFromArray(mesh.normals[idx0]));

            tris.push_back(eigenFromArray(mesh.vertices[idx1]));
            aabb.extend(tris.back());
            normals.push_back(eigenFromArray(mesh.normals[idx1]));

            tris.push_back(eigenFromArray(mesh.vertices[idx2]));
            aabb.extend(tris.back());
            normals.push_back(eigenFromArray(mesh.normals[idx2]));
        }
    }

    BRDF material1 = {
        .diffuse = Eigen::Vector3f(10, 3.8, 8.8),   // Diffuse reflectance (kd)
        .specular = Eigen::Vector3f(0.5, 0.1, 0.5),  // Specular reflectance (ks)
        .ambient = Eigen::Vector3f(0.2, 0.2, 0.1),   // Ambient reflectance (ka)
        .emission = Eigen::Vector3f(3.0, 1.0, 8.0),  // Emission (glow)
        .reflectivity = 0.2f,
        .shininess = 30                              // Shininess (n)
    };

    BRDF material2 = {
        .diffuse = Eigen::Vector3f(2.8, 3, 10.8),    // Diffuse reflectance (kd)
        .specular = Eigen::Vector3f(9.1, 0.5, 0.5),  // Specular reflectance (ks)
        .ambient = Eigen::Vector3f(0.1, 0.2, 0.2),   // Ambient reflectance (ka)
        .emission = Eigen::Vector3f(4.0, 5.0, 7.0),  // Emission (glow)
        .reflectivity = 0.2f,
        .shininess = 30  // Shininess (n)
    };

    BRDF material3 = {
        .diffuse = Eigen::Vector3f(1, 10.6, 6.9),     // Diffuse reflectance (kd)
        .specular = Eigen::Vector3f(0.5, 0.6, 11.9),  // Specular reflectance (ks)
        .ambient = Eigen::Vector3f(0.8, 0.5, 0.6),    // Ambient reflectance (ka)
        .emission = Eigen::Vector3f(4.0, 8.0, 4.0),   // Emission (glow)
        .reflectivity = 0.2f,
        .shininess = 30  // Shininess (n)
    };

    BRDF material4 = {
        .diffuse = Eigen::Vector3f(10, 10.6, 6.9),   // Diffuse reflectance (kd)
        .specular = Eigen::Vector3f(0.5, 0.6, 0.9),  // Specular reflectance (ks)
        .ambient = Eigen::Vector3f(0.8, 0.5, 0.6),   // Ambient reflectance (ka)
        .emission = Eigen::Vector3f(10.0, 4.8, 0.9),  // Emission (glow)
        .reflectivity = 0.2f,
        .shininess = 35                              // Shininess (n)
    };

    AggregateMeshIntersector aggMeshIntersector;
    aggMeshIntersector.addMeshIntersector(
        BVHMesh::makeMesh(tris, normals, makeTestTexCoords(tris), Material{material1}),
        Eigen::Affine3f(Eigen::Translation3f(-2.3f, 0.f, -9.f)));
    aggMeshIntersector.addMeshIntersector(
        BVHMesh::makeMesh(tris, normals, makeTestTexCoords(tris), Material{material2}),
        Eigen::Affine3f(Eigen::Translation3f(-2.3f, -5.f, -9.f)));
    aggMeshIntersector.addMeshIntersector(
        BVHMesh::makeMesh(tris, normals, makeTestTexCoords(tris), Material{material3}),
        Eigen::Affine3f(Eigen::Translation3f(2.3f, -5.f, -9.f)));
    aggMeshIntersector.addMeshIntersector(
        BVHMesh::makeMesh(tris, normals, makeTestTexCoords(tris), Material{material4}),
        Eigen::Affine3f(Eigen::Translation3f(2.3f, 0.f, -9.f)));

    aggMeshIntersector.buildTlas();

    auto multiMeshIntersector =
        ShapeIntersector::fromImpl<AggregateMeshIntersector>(std::move(aggMeshIntersector));

    auto scene = Scene<Sampler<UniformRandomSampler>>(
        screenShape, camera, std::vector{multiMeshIntersector}, std::move(lights), worldFromCamera,
        Eigen::Vector2i(250, 250));

    std::atomic<int> atomicProgress;
    std::mutex m;
    auto t_start = std::chrono::high_resolution_clock::now();
    scene.rayTrace(
        [&atomicProgress, &m](int num, int den) {
            atomicProgress++;
            if (auto val = atomicProgress.load(); val % 40'000 == 0) {
                std::scoped_lock lock(m);
                std::cout << "progress: " << static_cast<float>(val) / den << std::endl;
            }
        },
        16);

        

    auto t_end = std::chrono::high_resolution_clock::now();

    std::cout << "Elapsed time: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count()
              << " ms\n";

    scene.writRayTracedImageToFile(
        "/home/dbiswas2808/Documents/Projects/VdbPlayground/rt_test_images/"
        "TLAS_tank.png");
}

}  // namespace VdbFields