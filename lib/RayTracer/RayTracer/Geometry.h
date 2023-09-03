#pragma once
#include <vector>
#include <Eigen/Geometry>

namespace VdbFields::RayTracer {
struct TriMesh {
      std::vector<Eigen::Vector3f> points;
      std::vector<Eigen::Vector3i> tris;
};

struct Sphere {
      Eigen::Vector3f center;
      float radius_mm;
};
}