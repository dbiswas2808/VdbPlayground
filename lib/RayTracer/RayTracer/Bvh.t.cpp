#include <RayTracer/Bvh.h>
#include <catch2/catch_all.hpp>

namespace VdbFields {
TEST_CASE("BVH") {
    using namespace RayTracer;

    SECTION("Build bvh") {
        const Eigen::Vector3f v0(0, 0, 0);
        const Eigen::Vector3f v1(1, 0, 0);
        const Eigen::Vector3f v2(0, 1, 0);

        const Eigen::Vector3f v3(0, 0, 1);
        const Eigen::Vector3f v4(1, 0, 1);
        const Eigen::Vector3f v5(0, 1, 1);

        const Eigen::Vector3f v6(0, 0, 0);
        const Eigen::Vector3f v7(1, 0, 0);
        const Eigen::Vector3f v8(0, 0, 1);

        const Eigen::Vector3f v9(0, 0, 0);
        const Eigen::Vector3f v10(1, 0, 0);
        const Eigen::Vector3f v11(0, 1, 0);

        const Eigen::Vector3f v12(0, 0, 0);
        const Eigen::Vector3f v13(1, 0, 0);
        const Eigen::Vector3f v14(0, 0, 1);

        const Eigen::Vector3f v15(0, 0, 0);
        const Eigen::Vector3f v16(1, 0, 0);
        const Eigen::Vector3f v17(0, 1, 0);

        const Eigen::Vector3f v18(0, 0, 0);
        const Eigen::Vector3f v19(1, 0, 0);
        const Eigen::Vector3f v20(0, 0, 1);

        const Eigen::Vector3f v21(0, 0, 0);
        const Eigen::Vector3f v22(1, 0, 0);
        const Eigen::Vector3f v23(0, 1, 0);

        const Eigen::Vector3f v24(0, 0, 0);
        const Eigen::Vector3f v25(1, 0, 0);
        const Eigen::Vector3f v26(0, 0, 1);

        const Eigen::Vector3f v27(0, 0, 0);
        const Eigen::Vector3f v28(1, 0, 0);
        const Eigen::Vector3f v29(0, 1, 0);

        BVHMesh mesh;
        // Make BVH test mesh as a flat plane with 8 triangles
        mesh.triangles.push_back({v0, v1, v2, (v0 + v1 + v2) / 3});
        mesh.triangles.push_back({v3, v4, v5, (v3 + v4 + v5) / 3});
        mesh.triangles.push_back({v6, v7, v8, (v6 + v7 + v8) / 3});
        mesh.triangles.push_back({v9, v10, v11, (v9 + v10 + v11) / 3});
        mesh.triangles.push_back({v12, v13, v14, (v12 + v13 + v14) / 3});
        mesh.triangles.push_back({v15, v16, v17, (v15 + v16 + v17) / 3});
        mesh.triangles.push_back({v18, v19, v20, (v18 + v19 + v20) / 3});
        mesh.triangles.push_back({v21, v22, v23, (v21 + v22 + v23) / 3});
        mesh.triangles.push_back({v24, v25, v26, (v24 + v25 + v26) / 3});
        mesh.triangles.push_back({v27, v28, v29, (v27 + v28 + v29) / 3});

        BVH bvh{cow<BVHMesh>(mesh)};
        CHECK_NOTHROW(bvh.buildBVH());
    }

    SECTION("4 triangles") {
        // Create BRDF with appropriate values for testing
        BRDF material = {
            .diffuse = Eigen::Vector3f(10, 10, 10),      // Diffuse reflectance (kd)
            .specular = Eigen::Vector3f(10., 0.1, 1.5),  // Specular reflectance (ks)
            .ambient = Eigen::Vector3f(0.2, 0.2, 0.1),   // Ambient reflectance (ka)
            .emission = Eigen::Vector3f(8.0, 8.0, 8.0),  // Emission (glow)
            .reflectivity = 0.8f,
            .shininess = 30  // Shininess (n)
        };

        Eigen::Vector3f v0(-2, -6, -1);
        Eigen::Vector3f v1(-2, -3, 0);
        Eigen::Vector3f v2(-4, -3, 0);
        Eigen::Vector3f n0 = VdbFields::normal(v0, v1, v2);

        Eigen::Vector3f v3(-2, 3, -2);
        Eigen::Vector3f v4(-2, 6, 0);
        Eigen::Vector3f v5(-4, 6, 0);
        Eigen::Vector3f n1 = VdbFields::normal(v3, v4, v5);

        Eigen::Vector3f v6(2, -6, 0);
        Eigen::Vector3f v7(2, -3, 0);
        Eigen::Vector3f v8(1, -3, 0);
        Eigen::Vector3f n2 = VdbFields::normal(v6, v7, v8);

        Eigen::Vector3f v9(4, 3, 0);
        Eigen::Vector3f v10(4, 6, 0);
        Eigen::Vector3f v11(2, 6, 0);
        Eigen::Vector3f n3 = VdbFields::normal(v9, v10, v11);

        auto vertices = std::vector{v0, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11};
        auto vrtxNormals = std::vector{n0, n0, n0, n1, n1, n1, n2, n2, n2, n3, n3 , n3};
        auto mesh =
            BVHMesh::makeMesh(vertices, vrtxNormals,
                              std::vector(vertices.size(), Eigen::Vector2f()), Material{material});

        cow<BVH> bvh{cow<BVHMesh>(std::move(mesh))};
        BVHInstance bvhInstance{bvh, Eigen::Affine3f::Identity()};

        for (const auto& tri : bvh->getMesh().triangles) {
            const auto& pt = tri.centroid;
            const auto& n = VdbFields::normal(tri.vs[0], tri.vs[1], tri.vs[2]);
            BVHRay ray{pt + Eigen::Vector3f(0, 0, 1), Eigen::Vector3f(0, 0, -1),
                       Eigen::Vector3f(std::numeric_limits<float>::infinity(),
                                       std::numeric_limits<float>::infinity(), -1)};
            bvhInstance.intersect(ray);
            CHECK(ray.hasIntersection());
            CHECK((ray.getIntersect() - pt).squaredNorm() < epsilon_mm<float>);
            CHECK((bvhInstance.getNormal(ray.hit.getTriIdx(), ray.hit.uv) - n).squaredNorm() <
                  epsilon_mm<float>);
        }

        BVHRay ray{Eigen::Vector3f(0, 0, 1), Eigen::Vector3f(0, 0, -1),
                   Eigen::Vector3f(std::numeric_limits<float>::infinity(),
                                   std::numeric_limits<float>::infinity(), -1)};
        bvhInstance.intersect(ray);
        CHECK(not ray.hasIntersection());
    }
}

TEST_CASE("BVHInstance") {
    using namespace RayTracer;
    SECTION("4 triangles") {
        // Create BRDF with appropriate values for testing
        BRDF material = {
            .diffuse = Eigen::Vector3f(10, 10, 10),      // Diffuse reflectance (kd)
            .specular = Eigen::Vector3f(10., 0.1, 1.5),  // Specular reflectance (ks)
            .ambient = Eigen::Vector3f(0.2, 0.2, 0.1),   // Ambient reflectance (ka)
            .emission = Eigen::Vector3f(8.0, 8.0, 8.0),  // Emission (glow)
            .reflectivity = 0.8f,
            .shininess = 30  // Shininess (n)
        };

        Eigen::Vector3f v0(-2, -6, -1);
        Eigen::Vector3f v1(-2, -3, 0);
        Eigen::Vector3f v2(-4, -3, 0);
        Eigen::Vector3f n0 = VdbFields::normal(v0, v1, v2);

        Eigen::Vector3f v3(-2, 3, -2);
        Eigen::Vector3f v4(-2, 6, 0);
        Eigen::Vector3f v5(-4, 6, 0);
        Eigen::Vector3f n1 = VdbFields::normal(v3, v4, v5);

        Eigen::Vector3f v6(2, -6, 0);
        Eigen::Vector3f v7(2, -3, 0);
        Eigen::Vector3f v8(1, -3, 0);
        Eigen::Vector3f n2 = VdbFields::normal(v6, v7, v8);

        Eigen::Vector3f v9(4, 3, 0);
        Eigen::Vector3f v10(4, 6, 0);
        Eigen::Vector3f v11(2, 6, 0);
        Eigen::Vector3f n3 = VdbFields::normal(v9, v10, v11);

        auto vertices = std::vector{v0, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11};
        auto vrtxNormals = std::vector{n0, n0, n0, n1, n1, n1, n2, n2, n2, n3, n3 , n3};
        auto mesh =
            BVHMesh::makeMesh(vertices, vrtxNormals,
                              std::vector(vertices.size(), Eigen::Vector2f()), Material{material});

        cow<BVH> bvh {cow<BVHMesh>(std::move(mesh))};
        BVHInstance bvhInstance{bvh, Eigen::Affine3f(Eigen::Translation3f(0, 0, -1))};

        for (const auto& tri : bvh->getMesh().triangles) {
            const auto& pt = (tri.centroid + Eigen::Vector3f(0, 0, -1)).eval();
            const auto& normalExpected = VdbFields::normal(tri.vs[0], tri.vs[1], tri.vs[2]);
            BVHRay ray{pt + Eigen::Vector3f(0, 0, 1), Eigen::Vector3f(0, 0, -1),
                       Eigen::Vector3f(std::numeric_limits<float>::infinity(),
                                       std::numeric_limits<float>::infinity(), -1)};
            bvhInstance.intersect(ray);
            auto normalActual = bvhInstance.getNormal(ray.hit.getTriIdx(), ray.hit.uv);

            CHECK(ray.hasIntersection());
            CHECK((ray.getIntersect() - pt).squaredNorm() < epsilon_mm<float>);
            CHECK((normalActual - normalExpected).squaredNorm() < epsilon_mm<float>);
        }

        BVHRay ray{Eigen::Vector3f(0, 0, 1), Eigen::Vector3f(0, 0, -1),
                   Eigen::Vector3f(std::numeric_limits<float>::infinity(),
                                   std::numeric_limits<float>::infinity(), -1)};
        bvhInstance.intersect(ray);
        CHECK(not ray.hasIntersection());
    }
}

TEST_CASE("TLAS") {
    using namespace RayTracer;
    SECTION("4 Triangles") {
        // Create BRDF with appropriate values for testing
        BRDF material = {
            .diffuse = Eigen::Vector3f(10, 10, 10),      // Diffuse reflectance (kd)
            .specular = Eigen::Vector3f(10., 0.1, 1.5),  // Specular reflectance (ks)
            .ambient = Eigen::Vector3f(0.2, 0.2, 0.1),   // Ambient reflectance (ka)
            .emission = Eigen::Vector3f(8.0, 8.0, 8.0),  // Emission (glow)
            .reflectivity = 0.8f,
            .shininess = 30  // Shininess (n)
        };

        using namespace RayTracer;
        BVHMesh mesh;

        Eigen::Vector3f v0(-2, -6, -1);
        Eigen::Vector3f v1(-2, -3, 0);
        Eigen::Vector3f v2(-4, -3, 0);
        Eigen::Vector3f n0 = VdbFields::normal(v0, v1, v2);

        Eigen::Vector3f v3(-2, 3, -2);
        Eigen::Vector3f v4(-2, 6, 0);
        Eigen::Vector3f v5(-4, 6, 0);
        Eigen::Vector3f n1 = VdbFields::normal(v3, v4, v5);

        Eigen::Vector3f v6(2, -6, 0);
        Eigen::Vector3f v7(2, -3, 0);
        Eigen::Vector3f v8(1, -3, 0);
        Eigen::Vector3f n2 = VdbFields::normal(v6, v7, v8);

        Eigen::Vector3f v9(4, 3, 0);
        Eigen::Vector3f v10(4, 6, 0);
        Eigen::Vector3f v11(2, 6, 0);
        Eigen::Vector3f n3 = VdbFields::normal(v9, v10, v11);

        auto vertices = {v0, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11};
        auto vrtxNormals = std::vector{n0, n0, n0, n1, n1, n1, n2, n2, n2, n3, n3 , n3};
        cow<BVH> bvh{cow<BVHMesh>(BVHMesh::makeMesh(
            vertices, vrtxNormals, std::vector(vertices.size(), Eigen::Vector2f()), Material{material}))};

        std::array<Eigen::Translation3f, 4> txs{
            Eigen::Translation3f(0, 0, -1), Eigen::Translation3f(12.1, 0, -1),
            Eigen::Translation3f(0, -14.1, -1), Eigen::Translation3f(0, 14.1, -1)};

        BVHInstance bvhInstance0{bvh, Eigen::Affine3f(txs[0])};
        BVHInstance bvhInstance1{bvh, Eigen::Affine3f(txs[1])};
        BVHInstance bvhInstance2{bvh, Eigen::Affine3f(txs[2])};
        BVHInstance bvhInstance3{bvh, Eigen::Affine3f(txs[3])};

        BVHInstance bvhInstance4{bvh, Eigen::Translation3f(0, 0, -2) * Eigen::Affine3f(txs[0])};
        BVHInstance bvhInstance5{bvh, Eigen::Translation3f(0, 0, -2) * Eigen::Affine3f(txs[1])};
        BVHInstance bvhInstance6{bvh, Eigen::Translation3f(0, 0, -2) * Eigen::Affine3f(txs[2])};
        BVHInstance bvhInstance7{bvh, Eigen::Translation3f(0, 0, -2) * Eigen::Affine3f(txs[3])};

        TLAS tlas{{bvhInstance0, bvhInstance1, bvhInstance2, bvhInstance3, bvhInstance4,
                   bvhInstance5, bvhInstance6, bvhInstance7}};
        tlas.build();
        for (auto tx : txs) {
            for (const auto& tri : mesh.triangles) {
                const Eigen::Vector3f& pt = tx * tri.centroid;
                const auto& n = VdbFields::normal(tri.vs[0], tri.vs[1], tri.vs[2]);
                BVHRay ray{pt + Eigen::Vector3f(0, 0, 1), Eigen::Vector3f(0, 0, -1),
                           Eigen::Vector3f(std::numeric_limits<float>::infinity(),
                                           std::numeric_limits<float>::infinity(), -1)};
                tlas.intersect(ray);
                CHECK(ray.hasIntersection());
                CHECK((ray.getIntersect() - pt).squaredNorm() < std::pow(epsilon_mm<float>, 2));
                CHECK((tlas.getNormal(ray.hit) - n).squaredNorm() < std::pow(epsilon_mm<float>, 2));
            }
        }

        BVHRay ray{Eigen::Vector3f(0, 0, 1), Eigen::Vector3f(0, 0, -1),
                   Eigen::Vector3f(std::numeric_limits<float>::infinity(),
                                   std::numeric_limits<float>::infinity(), -1)};
        tlas.intersect(ray);
        CHECK(not ray.hasIntersection());
    }
}
}  // namespace VdbFields