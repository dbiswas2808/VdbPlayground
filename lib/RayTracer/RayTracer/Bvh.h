#include <memory>
#include <numeric>
#include <ranges>
#include <span>
#include <Eigen/Geometry>
#include <RayTracer/Common.h>
#include <RayTracer/Geometry.h>
#include <RayTracer/Ray.h>

namespace VdbFields::RayTracer{
struct AABB {
    Eigen::Vector3f aabbMin = Eigen::Vector3f::Constant(std::numeric_limits<float>::infinity());
    Eigen::Vector3f aabbMax = Eigen::Vector3f::Constant(-std::numeric_limits<float>::infinity());

    void extend(const Eigen::Vector3f& point) {
        aabbMin = aabbMin.cwiseMin(point);
        aabbMax = aabbMax.cwiseMax(point);
    }

    void extend(const AABB& other) {
        aabbMin = aabbMin.cwiseMin(other.aabbMin);
        aabbMax = aabbMax.cwiseMax(other.aabbMax);
    }

    [[nodiscard]] bool isEmpty() const {
        return aabbMin.x() > aabbMax.x() || aabbMin.y() > aabbMax.y() || aabbMin.z() > aabbMax.z();
    }

    [[nodiscard]] float area() const {
        Eigen::Vector3f diff = aabbMax - aabbMin;
        return isEmpty() ? 0 : 2 * (diff.x() * diff.y() + diff.x() * diff.z() + diff.y() * diff.z());
    }

    [[nodiscard]] bool contains(const Eigen::Vector3f& pt) const {
        return pt.x() >= aabbMin.x() && pt.x() <= aabbMax.x() && pt.y() >= aabbMin.y() &&
               pt.y() <= aabbMax.y() && pt.z() >= aabbMin.z() && pt.z() <= aabbMax.z();
    }
};

struct BVHNode {
    AABB bounds;
    uint leftFirst;  // If leaf level this is the first triangle index, otherwise it is the index of
                     // the left child
    uint triCount;

    [[nodiscard]] std::ranges::iota_view<uint, uint> getTriangleIndices() const {
        return std::ranges::iota_view(leftFirst, leftFirst + triCount);
    }

    [[nodiscard]] bool contains(const Eigen::Vector3f& pt) const {
        return bounds.contains(pt);
    }

    [[nodiscard]] bool isValidNode() const { return triCount > 0 || leftFirst > 0; }

    [[nodiscard]] bool isLeaf() const { return triCount > 0; }
};

struct BVHRay {
    Eigen::Vector3f origin;
    Eigen::Vector3f direction;
    Eigen::Vector3f invDirection;
    Eigen::Vector3f normal;
    float t = std::numeric_limits<float>::infinity();

    [[nodiscard]] bool hasIntersection() const {
        return t < std::numeric_limits<float>::infinity();
    }
    [[nodiscard]] Eigen::Vector3f getIntersect() const { return origin + direction * t; }
};

[[nodiscard]] float intersectAABB(const BVHRay& rayOrigin, const Eigen::Vector3f& aabbMin,
                                  const Eigen::Vector3f& aabbMax);

struct BVHMesh {
    struct Triangle {
        Eigen::Vector3f v0;
        Eigen::Vector3f v1;
        Eigen::Vector3f v2;
        Eigen::Vector3f centroid;
    };

    std::vector<Triangle> triangles;

    [[nodiscard]] static BVHMesh makeMesh(std::span<const Eigen::Vector3f> triangleSoup) {
        assert(triangleSoup.size() % 3 == 0);

        std::vector<Triangle> result;

        for (size_t ii = 0; ii < triangleSoup.size(); ii += 3) {
            Eigen::Vector3f v0 = triangleSoup[ii];
            Eigen::Vector3f v1 = triangleSoup[ii + 1];
            Eigen::Vector3f v2 = triangleSoup[ii + 2];
            result.push_back({v0, v1, v2, (v0 + v1 + v2) / 3});
        }

        return {std::move(result)};
    }
};

class BVH {
   public:
    BVH(cow<const BVHMesh> mesh)
        : m_mesh(std::move(mesh)), m_triIdx(m_mesh->triangles.size()) {
        std::iota(m_triIdx.begin(), m_triIdx.end(), 0);
    }

    void intersect(BVHRay& ray) const;

    void buildBVH();

    [[nodiscard]] const BVHMesh& getMesh() const { return *m_mesh; }

    [[nodiscard]] const BVHNode& getRoot() const { return m_nodes[m_rootIndex]; }

   private:
    [[nodiscard]] const BVHMesh::Triangle& getTriangle(uint triIdx) const {
        return m_mesh->triangles[m_triIdx[triIdx]];
    }

    [[nodiscard]] float evaluateSAH(const BVHNode& node, uint axis, float splitPos) const;

    void updateNodeBounds(BVHNode& node);

    void subdivide(BVHNode& node, uint depth = 0);

    [[nodiscard]] std::pair<BVHNode&, BVHNode&> makeNewLeaves(BVHNode& node, uint leftFirst);

    [[nodiscard]] std::optional<uint> findSplitTriIndex(BVHNode& node, uint axis, float splitPos);


    struct BestSplitData;
    [[nodiscard]] BestSplitData findBestSplit(const BVHNode& node) const;


    uint m_rootIndex = 0;
    std::vector<BVHNode> m_nodes;
    uint m_numNodes = 0;
    cow<const BVHMesh> m_mesh;
    std::vector<size_t> m_triIdx;
};

class BVHInstance {
   public:
    BVHInstance(cow<BVH> bvh, Eigen::Affine3f worldFromGeom)
        : m_bvh(std::move(bvh)),
          m_worldFromGeom(worldFromGeom),
          m_geomFromWorld(worldFromGeom.inverse()) {
        auto bbox = m_bvh.read().getRoot().bounds;
        for (int i = 0; i < 8; i++) {
            aabb.extend(m_worldFromGeom *
                        Eigen::Vector3f(i & 0b100 ? bbox.aabbMax[0] : bbox.aabbMin[0],
                                        i & 0b010 ? bbox.aabbMax[1] : bbox.aabbMin[1],
                                        i & 0b001 ? bbox.aabbMax[2] : bbox.aabbMin[2]));
        }
    };
    void intersect(BVHRay& ray) const;
    [[nodiscard]] const AABB& getAABB() const { return aabb; }

   private:
    cow<BVH> m_bvh;
    Eigen::Affine3f m_worldFromGeom;
    Eigen::Affine3f m_geomFromWorld;
    AABB aabb;
};

struct TLASNode {
    AABB bounds;
    uint leftRight;  // If leaf level this is the first triangle index, otherwise it is the index of
                     // the left child
    uint bvhIdx;
    bool isLeaf() const {
        return leftRight == 0;
    }
};


class TLAS {
   public:
    TLAS(std::vector<BVHInstance> bvhInstances);
    void build();
    void intersect(BVHRay& ray) const;

   private:
    [[nodiscard]] int findBestMatch(std::span<const int> TLASNodes, int nodeA) const;

    std::vector<TLASNode> m_tlasNode;
    std::vector<BVHInstance> m_bvhInstances;
    AABB m_aabb;
};
}  // namespace VdbFields::RayTracer