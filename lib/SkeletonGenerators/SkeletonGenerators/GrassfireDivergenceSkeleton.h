namespace VdbFields {
namespace Core {
struct Mesh;
}

namespace SkeletonGenerators {
Core::Mesh grassfireDivergenceSkeleton(const Core::Mesh &model,
                                       float voxelSize = 0.1);
}
} // namespace VdbFields