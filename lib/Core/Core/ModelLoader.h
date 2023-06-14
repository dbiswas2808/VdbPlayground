#include <Eigen/Core>

namespace VdbFields::Core {
class Model;

[[nodiscard]] Model
loadTestPolytopeMeshModel(int faceCount, double scale = 1.0,
                          Eigen::Vector3d center = Eigen::Vector3d{});
} // namespace VdbFields::Core