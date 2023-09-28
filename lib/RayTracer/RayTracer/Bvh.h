#include <numeric>
#include <ranges>
#include <span>
#include <Eigen/Geometry>
#include <RayTracer/Common.h>

namespace VdbFields::RayTracer{
struct AABB {
    Eigen::Vector3f aabbMin = Eigen::Vector3f::Constant(std::numeric_limits<float>::infinity());
    Eigen::Vector3f aabbMax = Eigen::Vector3f::Constant(-std::numeric_limits<float>::infinity());

    void extend(const Eigen::Vector3f& point);
    void extend(const AABB& other);
    [[nodiscard]] bool isEmpty() const;
    [[nodiscard]] float area() const;
    [[nodiscard]] bool contains(const Eigen::Vector3f& pt) const;

    [[nodiscard]] friend AABB operator*(const Eigen::Affine3f& transform, const AABB& aabb);
};

struct BVHRay {
    Eigen::Vector3f origin;
    Eigen::Vector3f direction;
    Eigen::Vector3f invDirection;
    Eigen::Vector3f normal;
    float t = std::numeric_limits<float>::infinity();

    [[nodiscard]] BVHRay transform(const Eigen::Affine3f& transform) const;

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

    [[nodiscard]] static BVHMesh makeMesh(std::span<const Eigen::Vector3f> triangleSoup);
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
    BVHInstance(cow<BVH> bvh, const Eigen::Affine3f& worldFromGeom);
    void intersect(BVHRay& ray) const;
    [[nodiscard]] const AABB& getAABB() const { return m_aabb; }

   private:
    cow<BVH> m_bvh;
    Eigen::Affine3f m_worldFromGeom;
    Eigen::Affine3f m_geomFromWorld;
    AABB m_aabb;
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