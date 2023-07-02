#include <Core/ModelLoader.h>

#include <ranges>
#include <Eigen/Core>

#include <Core/Model.h>
#include <MeshFileReaders/StlFileReader.h>


namespace VdbFields {
Core::Model Core::loadTestPolytopeMeshModel(int faceCount, double scale,
                                            Eigen::Vector3d center) {
  using namespace Eigen;

  std::vector<Vector3d> vtx;
  std::vector<Vector3i> tri;
  std::vector<Vector4i> qua;

  if (faceCount == 4) { // Tetrahedron
    vtx.push_back(Vector3d(0.0f, 1.0f, 0.0f));
    vtx.push_back(Vector3d(-0.942810297f, -0.333329707f, 0.0f));
    vtx.push_back(Vector3d(0.471405149f, -0.333329707f, 0.816497624f));
    vtx.push_back(Vector3d(0.471405149f, -0.333329707f, -0.816497624f));

    tri.push_back(Vector3i(0, 2, 3));
    tri.push_back(Vector3i(0, 3, 1));
    tri.push_back(Vector3i(0, 1, 2));
    tri.push_back(Vector3i(1, 3, 2));

  } else if (faceCount == 6) { // Cube
    vtx.push_back(Vector3d(-0.5f, -0.5f, -0.5f));
    vtx.push_back(Vector3d(0.5f, -0.5f, -0.5f));
    vtx.push_back(Vector3d(0.5f, -0.5f, 0.5f));
    vtx.push_back(Vector3d(-0.5f, -0.5f, 0.5f));
    vtx.push_back(Vector3d(-0.5f, 0.5f, -0.5f));
    vtx.push_back(Vector3d(0.5f, 0.5f, -0.5f));
    vtx.push_back(Vector3d(0.5f, 0.5f, 0.5f));
    vtx.push_back(Vector3d(-0.5f, 0.5f, 0.5f));

    qua.push_back(Vector4i(1, 0, 4, 5));
    qua.push_back(Vector4i(2, 1, 5, 6));
    qua.push_back(Vector4i(3, 2, 6, 7));
    qua.push_back(Vector4i(0, 3, 7, 4));
    qua.push_back(Vector4i(2, 3, 0, 1));
    qua.push_back(Vector4i(5, 4, 7, 6));

  } else if (faceCount == 8) { // Octahedron

    vtx.push_back(Vector3d(0.0f, 0.0f, -1.0f));
    vtx.push_back(Vector3d(1.0f, 0.0f, 0.0f));
    vtx.push_back(Vector3d(0.0f, 0.0f, 1.0f));
    vtx.push_back(Vector3d(-1.0f, 0.0f, 0.0f));
    vtx.push_back(Vector3d(0.0f, -1.0f, 0.0f));
    vtx.push_back(Vector3d(0.0f, 1.0f, 0.0f));

    tri.push_back(Vector3i(0, 4, 3));
    tri.push_back(Vector3i(0, 1, 4));
    tri.push_back(Vector3i(1, 2, 4));
    tri.push_back(Vector3i(2, 3, 4));
    tri.push_back(Vector3i(0, 3, 5));
    tri.push_back(Vector3i(0, 5, 1));
    tri.push_back(Vector3i(1, 5, 2));
    tri.push_back(Vector3i(2, 5, 3));

  } else if (faceCount == 12) { // Dodecahedron
    vtx.push_back(Vector3d(0.354437858f, 0.487842113f, -0.789344311f));
    vtx.push_back(Vector3d(0.573492587f, -0.186338872f, -0.78934437f));
    vtx.push_back(Vector3d(0.0f, -0.603005826f, -0.78934443f));
    vtx.push_back(Vector3d(-0.573492587f, -0.186338872f, -0.78934437f));
    vtx.push_back(Vector3d(-0.354437858f, 0.487842113f, -0.789344311f));
    vtx.push_back(Vector3d(-0.573492587f, 0.789345026f, -0.186338797f));
    vtx.push_back(Vector3d(-0.927930415f, -0.301502913f, -0.186338872f));
    vtx.push_back(Vector3d(0.0f, -0.975683928f, -0.186338902f));
    vtx.push_back(Vector3d(0.927930415f, -0.301502913f, -0.186338872f));
    vtx.push_back(Vector3d(0.573492587f, 0.789345026f, -0.186338797f));
    vtx.push_back(Vector3d(0.0f, 0.975683868f, 0.186338902f));
    vtx.push_back(Vector3d(-0.927930415f, 0.301502913f, 0.186338872f));
    vtx.push_back(Vector3d(-0.573492587f, -0.789345026f, 0.186338797f));
    vtx.push_back(Vector3d(0.573492587f, -0.789345026f, 0.186338797f));
    vtx.push_back(Vector3d(0.927930415f, 0.301502913f, 0.186338872f));
    vtx.push_back(Vector3d(0.0f, 0.603005826f, 0.78934443f));
    vtx.push_back(Vector3d(0.573492587f, 0.186338872f, 0.78934437f));
    vtx.push_back(Vector3d(0.354437858f, -0.487842113f, 0.789344311f));
    vtx.push_back(Vector3d(-0.354437858f, -0.487842113f, 0.789344311f));
    vtx.push_back(Vector3d(-0.573492587f, 0.186338872f, 0.78934437f));

    qua.push_back(Vector4i(0, 1, 2, 3));
    tri.push_back(Vector3i(0, 3, 4));
    qua.push_back(Vector4i(0, 4, 5, 10));
    tri.push_back(Vector3i(0, 10, 9));
    qua.push_back(Vector4i(0, 9, 14, 8));
    tri.push_back(Vector3i(0, 8, 1));
    qua.push_back(Vector4i(1, 8, 13, 7));
    tri.push_back(Vector3i(1, 7, 2));
    qua.push_back(Vector4i(2, 7, 12, 6));
    tri.push_back(Vector3i(2, 6, 3));
    qua.push_back(Vector4i(3, 6, 11, 5));
    tri.push_back(Vector3i(3, 5, 4));
    qua.push_back(Vector4i(5, 11, 19, 15));
    tri.push_back(Vector3i(5, 15, 10));
    qua.push_back(Vector4i(6, 12, 18, 19));
    tri.push_back(Vector3i(6, 19, 11));
    qua.push_back(Vector4i(7, 13, 17, 18));
    tri.push_back(Vector3i(7, 18, 12));
    qua.push_back(Vector4i(8, 14, 16, 17));
    tri.push_back(Vector3i(8, 17, 13));
    qua.push_back(Vector4i(9, 10, 15, 16));
    tri.push_back(Vector3i(9, 16, 14));
    qua.push_back(Vector4i(15, 19, 18, 17));
    tri.push_back(Vector3i(15, 17, 16));

  } else if (faceCount == 20) { // Icosahedron
    vtx.push_back(Vector3d(0.0f, 0.0f, -1.0f));
    vtx.push_back(Vector3d(0.0f, 0.894427359f, -0.447213143f));
    vtx.push_back(Vector3d(0.850650847f, 0.276393682f, -0.447213203f));
    vtx.push_back(Vector3d(0.525731206f, -0.723606944f, -0.447213262f));
    vtx.push_back(Vector3d(-0.525731206f, -0.723606944f, -0.447213262f));
    vtx.push_back(Vector3d(-0.850650847f, 0.276393682f, -0.447213203f));
    vtx.push_back(Vector3d(-0.525731206f, 0.723606944f, 0.447213262f));
    vtx.push_back(Vector3d(-0.850650847f, -0.276393682f, 0.447213203f));
    vtx.push_back(Vector3d(0.0f, -0.894427359f, 0.447213143f));
    vtx.push_back(Vector3d(0.850650847f, -0.276393682f, 0.447213203f));
    vtx.push_back(Vector3d(0.525731206f, 0.723606944f, 0.447213262f));
    vtx.push_back(Vector3d(0.0f, 0.0f, 1.0f));

    tri.push_back(Vector3i(2, 0, 1));
    tri.push_back(Vector3i(3, 0, 2));
    tri.push_back(Vector3i(4, 0, 3));
    tri.push_back(Vector3i(5, 0, 4));
    tri.push_back(Vector3i(1, 0, 5));
    tri.push_back(Vector3i(6, 1, 5));
    tri.push_back(Vector3i(7, 5, 4));
    tri.push_back(Vector3i(8, 4, 3));
    tri.push_back(Vector3i(9, 3, 2));
    tri.push_back(Vector3i(10, 2, 1));
    tri.push_back(Vector3i(10, 1, 6));
    tri.push_back(Vector3i(6, 5, 7));
    tri.push_back(Vector3i(7, 4, 8));
    tri.push_back(Vector3i(8, 3, 9));
    tri.push_back(Vector3i(9, 2, 10));
    tri.push_back(Vector3i(6, 11, 10));
    tri.push_back(Vector3i(10, 11, 9));
    tri.push_back(Vector3i(9, 11, 8));
    tri.push_back(Vector3i(8, 11, 7));
    tri.push_back(Vector3i(7, 11, 6));
  }

  // Apply scale and translation to all the vertices
  for ( size_t i = 0; i<vtx.size(); ++i ) vtx[i] = scale * vtx[i] + center;

  return {std::move(vtx), std::move(tri), std::move(qua)};
}
} // namespace VdbFields