#include <SkeletonGenerators/GrassfireDivergenceSkeleton.h>

#include <ranges>

#include <openvdb/openvdb.h>
#include <openvdb/tools/LevelSetUtil.h>
#include <openvdb/tools/MeshToVolume.h>
#include <openvdb/tools/VolumeToMesh.h>

#include <Core/Model.h>
#include <Morphology/MeanFlux.h>

namespace VdbFields {
namespace {
[[nodiscard]] std::vector<openvdb::Vec3s> getVdbMeshVertices(const Core::Mesh &mesh) {
    std::vector<openvdb::Vec3s> vertices;
    std::ranges::copy(mesh.m_points | std::views::transform([](Eigen::Vector3d v) {
                          return openvdb::Vec3s(v[0], v[1], v[2]);
                      }),
                      std::back_inserter(vertices));
    return vertices;
}

[[nodiscard]] std::vector<openvdb::Vec3I> getVdbMeshTris(const Core::Mesh &mesh) {
    std::vector<openvdb::Vec3I> tris;
    std::ranges::copy(mesh.m_triFaces | std::views::transform([](Eigen::Vector3i v) {
                          return openvdb::Vec3I(v[0], v[1], v[2]);
                      }),
                      std::back_inserter(tris));
    return tris;
}

[[nodiscard]] std::vector<openvdb::Vec4I> getVdbMeshQuads(const Core::Mesh &mesh) {
    std::vector<openvdb::Vec4I> quads;
    std::ranges::copy(mesh.m_quadFaces | std::views::transform([](Eigen::Vector4i v) {
                          return openvdb::Vec4I(v[0], v[1], v[2], v[3]);
                      }),
                      std::back_inserter(quads));
    return quads;
}

[[nodiscard]] Core::Mesh convertVdbMeshDataToCoreMesh(const std::vector<openvdb::Vec3s> &vertices,
                                                      const std::vector<openvdb::Vec3I> &tris,
                                                      const std::vector<openvdb::Vec4I> &quads) {
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
}  // namespace

Core::Mesh SkeletonGenerators::grassfireDivergenceSkeleton(const Core::Mesh &mesh,
                                                           float voxelSize) {
    using namespace openvdb;
    using namespace VdbFields::Morphology;

    // Create a transform with the specified voxel size.
    auto txForm = math::Transform::createLinearTransform(voxelSize);

    // Convert to vdb types for vertices, triangles and quads
    std::vector<Vec3s> vertices = getVdbMeshVertices(mesh);
    std::vector<Vec3I> tris = getVdbMeshTris(mesh);
    std::vector<Vec4I> quads = getVdbMeshQuads(mesh);

    // Create a FloatGrid and populate it with a narrow-band SDF.
    auto signedField =
        tools::meshToSignedDistanceField<FloatGrid>(*txForm, vertices, tris, quads, 3.0f, 100.0f);

    // Extract the active voxel segment masks and merge them to a single grid
    std::vector<BoolGrid::Ptr> outputMasks;
    tools::extractActiveVoxelSegmentMasks(*signedField, outputMasks);

    if (outputMasks.empty()) {
        return {};
    }

    for (auto &mask : std::ranges::subrange(std::ranges::next(outputMasks.begin()), outputMasks.end())) {
        outputMasks[0]->merge(*mask);
    }

    // Compute the gradient of the signed distance field and then generate the mean flux field
    auto gradGrid = tools::gradient(*signedField);
    auto interrupter = util::NullInterrupter();
    auto avgFluxProcessor =
        MeanFluxProcessor<Vec3fGrid, LevelSetOperators::MeanFluxScheme::neighbor98, FloatGrid,
                          BoolGrid>{*gradGrid, *outputMasks[0], &interrupter};
    auto outGrid = avgFluxProcessor.process();



    // Convert the mean flux field to a mesh along the 0.05 level set
    vertices.clear();
    tris.clear();
    quads.clear();

    static constexpr float medialIsoValue = 0.1f;
    openvdb::tools::volumeToMesh(*outGrid, vertices, tris, quads, medialIsoValue);
    return convertVdbMeshDataToCoreMesh(vertices, tris, quads);
}
}  // namespace VdbFields