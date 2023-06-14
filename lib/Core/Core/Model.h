#include <Eigen/Core>
#include <vector>

namespace VdbFields::Core {
struct Mesh {
  std::vector<Eigen::Vector3d> m_points;
  std::vector<Eigen::Vector3i> m_triFaces;
  std::vector<Eigen::Vector4i> m_quadFaces;
};

struct Model {
  Mesh m_mesh;
};

} // namespace VdbFields::Core