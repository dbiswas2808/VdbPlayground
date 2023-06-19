#include <Core/Model.h>
#include <Morphology/MeanFlux.h>
#include <SkeletonGenerators/GrassfireDivergenceSkeleton.h>
#include <openvdb/openvdb.h>
#include <openvdb/tools/LevelSetUtil.h>
#include <openvdb/tools/MeshToVolume.h>
#include <openvdb/tools/VolumeToMesh.h>

#include <ranges>

namespace VdbFields {
Core::Mesh SkeletonGenerators::grassfireDivergenceSkeleton(const Core::Mesh &mesh,
                                                           float voxelSize) {
    using namespace openvdb;
    using namespace VdbFields::Morphology;

    auto txForm = math::Transform::createLinearTransform(voxelSize);

    std::vector<Vec3s> vertices;
    std::vector<Vec3I> tris;
    std::vector<Vec4I> quads;
    std::ranges::copy(mesh.m_points | std::views::transform([](Eigen::Vector3d v) {
                          return Vec3s(v[0], v[1], v[2]);
                      }),
                      std::back_inserter(vertices));

    std::ranges::copy(mesh.m_triFaces | std::views::transform([](Eigen::Vector3i v) {
                          return Vec3I(v[0], v[1], v[2]);
                      }),
                      std::back_inserter(tris));

    std::ranges::copy(mesh.m_quadFaces | std::views::transform([](Eigen::Vector4i v) {
                          return Vec4I(v[0], v[1], v[2], v[3]);
                      }),
                      std::back_inserter(quads));

    auto signedField =
        tools::meshToSignedDistanceField<FloatGrid>(*txForm, vertices, tris, quads, 3.0f, 100.0f);

    auto gradGrid = tools::gradient(*signedField);
    auto avgFluxProcessor = MeanFluxProcessor<Vec3fGrid, MeanFluxScheme::NEIGHBOR_98>{*gradGrid};
    auto outGrid = avgFluxProcessor.process();

    vertices.clear();
    tris.clear();
    quads.clear();
    openvdb::tools::volumeToMesh(*outGrid, vertices, tris, quads, 0.05);
    Core::Mesh skeletonMesh;
    std::ranges::copy(vertices | std::views::transform([](auto v) -> Eigen::Vector3d {
                          return Eigen::Vector3d(v[0], v[1], v[2]);
                      }),
                      std::back_inserter(skeletonMesh.m_points));

    std::ranges::copy(tris | std::views::transform([](auto v) -> Eigen::Vector3i {
                          return Eigen::Vector3i(v[0], v[1], v[2]);
                      }),
                      std::back_inserter(skeletonMesh.m_triFaces));

    for (auto qVs : quads) {
        skeletonMesh.m_triFaces.push_back(
            {static_cast<int>(qVs[0]), static_cast<int>(qVs[1]), static_cast<int>(qVs[3])});
        skeletonMesh.m_triFaces.push_back(
            {static_cast<int>(qVs[1]), static_cast<int>(qVs[2]), static_cast<int>(qVs[3])});
    }

    return skeletonMesh;
}
}  // namespace VdbFields