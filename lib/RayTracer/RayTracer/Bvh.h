#include <numeric>
#include <ranges>
#include <span>
#include <memory>
#include <iostream>
#include <Eigen/Geometry>
#include <RayTracer/Common.h>
#include <RayTracer/Material.h>
#pragma GCC optimize ("O0")
namespace VdbFields::RayTracer{
struct BVHRay {
    struct Hit{
        float t = std::numeric_limits<float>::infinity();
        Eigen::Vector2f uv;
        uint32_t instPrimId;

        [[nodiscard]] size_t getTriIdx() const {
            return instPrimId & 0x000FFFFF;
        }

        [[nodiscard]] size_t getBvhInstIdx() const {
            return instPrimId >> 20;
        } 

        void setTriIdx(size_t triIdx) {
            instPrimId = (instPrimId & 0xFFF00000) | (triIdx & 0x000FFFFF);
        }

        void setBvhInstIdx(size_t bvhInstIdx) {
            instPrimId = (instPrimId & 0x000FFFFF) | (bvhInstIdx << 20);
        }

        [[nodiscard]] bool isHit() const {
            return std::isfinite(t);
        }
    };

    Eigen::Vector3f origin;
    Eigen::Vector3f direction;
    Eigen::Vector3f invDirection;
    Hit hit = Hit{};

    [[nodiscard]] BVHRay transform(const Eigen::Affine3f& transform) const;

    [[nodiscard]] bool hasIntersection() const {
        return hit.isHit();
    }
    [[nodiscard]] Eigen::Vector3f getIntersect() const { return origin + direction * hit.t; }

    [[nodiscard]] static BVHRay makeRay(const Eigen::Vector3f& origin,
                                        const Eigen::Vector3f& direction) {
        return {.origin = origin, .direction = direction, .invDirection = direction.cwiseInverse()};
    }
};

[[nodiscard]] float intersectAABB(const BVHRay& rayOrigin, const Eigen::Vector3f& aabbMin,
                                  const Eigen::Vector3f& aabbMax);

class BVH;

struct BVHMesh {
    struct Triangle {
        std::array<Eigen::Vector3f, 3> vs;
        Eigen::Vector3f centroid;
    };

    struct TriEx {
        std::array<Eigen::Vector2f, 3> texUv;
        std::array<Eigen::Vector3f, 3> normal;
    };

    std::vector<Triangle> triangles;
    std::vector<TriEx> triData;
    Material m_material;

    [[nodiscard]] const Material& getMaterial(int, const Eigen::Vector2f&) const { return m_material; }

    [[nodiscard]] static BVHMesh makeMesh(std::span<const Eigen::Vector3f> triangleSoup,
                                          std::span<const Eigen::Vector3f> normals,
                                          std::span<const Eigen::Vector2f> texUvs,
                                          Material material);
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
    BVH(cow<BVHMesh> mesh)
        : m_mesh(std::move(mesh)), m_triIdx(m_mesh->triangles.size()) {
        std::iota(m_triIdx.begin(), m_triIdx.end(), 0);
        this->buildBVH();
    }

    void intersect(BVHRay& ray) const;

    void buildBVH();

    [[nodiscard]] const BVHMesh& getMesh() const { return *m_mesh; }

    [[nodiscard]] const BVHNode& getRoot() const { return m_nodes[m_rootIndex]; }

   private:
    [[nodiscard]] size_t getTriIdx(uint triIdx) const { return m_triIdx[triIdx]; }

    [[nodiscard]] const BVHMesh::Triangle& getTriangle(uint triIdx) const {
        return m_mesh->triangles[getTriIdx(triIdx)];
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
    void intersect(BVHRay& ray_world) const;
    [[nodiscard]] const AABB& getAABB_world() const { return m_aabb_world; }

    [[nodiscard]] Eigen::Vector3f getNormal(int triIdx, const Eigen::Vector2f& uv) const {
        auto vNormals_geom = m_bvh->getMesh().triData[triIdx].normal;
        auto normal_geom = ((1 - uv.x() - uv.y()) * vNormals_geom[0] + uv.x() * vNormals_geom[1] +
                            uv.y() * vNormals_geom[2])
                               .normalized();
        return (txInvTranspose(m_worldFromGeom) * normal_geom).eval();
    }

    [[nodiscard]] Eigen::Vector2f getTexCoord(int triIdx, const Eigen::Vector2f& uv) const { 
        auto texUv = m_bvh->getMesh().triData[triIdx].texUv;
        return (1 - uv.x() - uv.y()) * texUv[0] + uv.x() * texUv[1] + uv.y() * texUv[2];
    }

     [[nodiscard]] const Material& gerMaterial(int triIdx, const Eigen::Vector2f& uv) const { 
        return m_bvh->getMesh().getMaterial(triIdx, uv);
    }

   private:
    cow<BVH> m_bvh;
    Eigen::Affine3f m_worldFromGeom;
    Eigen::Affine3f m_geomFromWorld;
    AABB m_aabb_world;
};

struct TLASNode {
    AABB aabb_world;
    uint leftRight;  // If leaf level this is the first triangle index, otherwise it is the index of
                     // the left child
    uint bvhIdx;
    bool isLeaf() const {
        return leftRight == 0;
    }
};


class TLAS {
   public:
    TLAS() = default;
    TLAS(std::vector<BVHInstance> bvhInstances);
    void build();
    void intersect(BVHRay& ray_world) const;

    [[nodiscard]] Eigen::Vector3f getNormal(const BVHRay::Hit& hit) const {
        const BVHInstance& bvhInstance = m_bvhInstances[hit.getBvhInstIdx()];
        return bvhInstance.getNormal(hit.getTriIdx(), hit.uv);
    }

    [[nodiscard]] Eigen::Vector2f getTexCoord(const BVHRay::Hit& hit) const {
        const BVHInstance& bvhInstance = m_bvhInstances[hit.getBvhInstIdx()];
        return bvhInstance.getTexCoord(hit.getTriIdx(), hit.uv);
    }

    [[nodiscard]] const Material& getMaterial(const BVHRay::Hit& hit) const {
        const BVHInstance& bvhInstance = m_bvhInstances[hit.getBvhInstIdx()];
        return bvhInstance.gerMaterial(hit.getTriIdx(), hit.uv);
    }

   private:
    [[nodiscard]] int findBestMatch(std::span<const int> TLASNodes, int nodeA) const;

    std::vector<TLASNode> m_tlasNode;
    std::vector<BVHInstance> m_bvhInstances;
};
}  // namespace VdbFields::RayTracer