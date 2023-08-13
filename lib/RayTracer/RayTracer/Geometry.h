#include <vector>
#include <Eigen/Core>

namespace VdbFields::RayTracer {
class Geometry {};

struct TriMesh : Geometry {
      std::vector<Eigen::Vector3f> points;
      std::vector<Eigen::Vector3i> tris;
};

struct Sphere : Geometry {
      Eigen::Vector3f center;
      float radius_mm;
};
}