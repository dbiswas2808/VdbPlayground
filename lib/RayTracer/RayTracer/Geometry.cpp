#include <RayTracer/Geometry.h>

namespace VdbFields::RayTracer {
std::array<Eigen::Vector3f, 4> TriMesh::getTriangle(size_t triIdx) const {
    return {points.col(tris[3 * triIdx + 0]), points.col(tris[3 * triIdx + 1]),
            points.col(tris[3 * triIdx + 2])};
}
}