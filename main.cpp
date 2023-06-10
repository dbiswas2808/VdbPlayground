#include <Eigen/Dense>
#include <glm/vec3.hpp>
#include <iostream>
#include <morphology/mean_flux.h>
#include <openvdb/openvdb.h>
#include <openvdb/tools/GridOperators.h>
#include <openvdb/tools/LevelSetPlatonic.h>
#include <openvdb/tools/VolumeToMesh.h>
#include <openvdb/util/NullInterrupter.h>
#include <polyscope/point_cloud.h>
#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>
#include <polyscope/volume_grid.h>
#include <ranges>
#include <vector>

namespace PolyscopeSurfaceUtil {
struct PolyscopeMesh {
  std::vector<Eigen::Vector3f> points;
  std::vector<Eigen::Vector3i> tris;
};

#pragma GCC optimize("O0")

[[nodiscard]] PolyscopeMesh generateTestPolyscopeMeshFromVdbGrid() {
  auto grid = Morphology::createLevelSetTestCases<openvdb::FloatGrid>(
      20, 10.f, {}, 0.1, 100.f);

  auto gradGrid = openvdb::tools::gradient(*grid);
  auto avgFluxProcessor = Morphology::MeanFluxProcessor{*gradGrid};
  auto outGrid = avgFluxProcessor.process();

  std::vector<openvdb::Vec3s> vertices;
  std::vector<openvdb::Vec3I> tris;
  std::vector<openvdb::Vec4I> quads;

  openvdb::tools::volumeToMesh(*outGrid, vertices, tris, quads, 0.05);
  PolyscopeMesh mesh;
  std::ranges::copy(vertices |
                        std::views::transform([](auto v) -> Eigen::Vector3f {
                          return Eigen::Vector3f(v[0], v[1], v[2]);
                        }),
                    std::back_inserter(mesh.points));

  std::ranges::copy(tris | std::views::transform([](auto v) -> Eigen::Vector3i {
                      return Eigen::Vector3i(v[0], v[1], v[2]);
                    }),
                    std::back_inserter(mesh.tris));

  for (auto qVs : quads) {
    mesh.tris.push_back({qVs[0], qVs[1], qVs[3]});
    mesh.tris.push_back({qVs[1], qVs[2], qVs[3]});
  }

  return mesh;
}
} // namespace PolyscopeSurfaceUtil

int main(int, char **) {
  polyscope::view::moveScale = 10.0;
  polyscope::view::projectionMode = polyscope::ProjectionMode::Orthographic;
  polyscope::options::programName = "Geometry processor";
  polyscope::init();

  auto polyMesh = PolyscopeSurfaceUtil::generateTestPolyscopeMeshFromVdbGrid();
  auto *surfaceMesh = polyscope::registerSurfaceMesh(
      "my points", polyMesh.points, polyMesh.tris);
  auto *volumeGrid = polyscope::registerVolumeGrid(
      "Grid mesh", 0.5, glm::vec3(0, 0, 0), glm::vec3(5., 5., 5.));

  polyscope::show();
}