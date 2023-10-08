#include <RayTracer/Bvh.h>
#pragma GCC optimize ("O0")
namespace VdbFields {
namespace RayTracer {
namespace {
[[nodiscard]] Eigen::Vector3f normal(const Eigen::Vector3f& v0, const Eigen::Vector3f& v1,
                                     const Eigen::Vector3f& v2) {
    return (v2 - v1).cross(v0 - v1).normalized();
}

void splitNode(BVHNode& node, BVHNode& lChild, BVHNode& rChild, uint splitTriIdx) {
    lChild.leftFirst = node.leftFirst;
    lChild.triCount = splitTriIdx - node.leftFirst;
    rChild.leftFirst = splitTriIdx;
    rChild.triCount = node.triCount + node.leftFirst - splitTriIdx;
}

void updateNodeWithLeft(BVHNode& node, uint lIdx) {
    node.leftFirst = lIdx;
    node.triCount = 0;
}

consteval uint getMaxDepth() {
    return 64;
}
}  // namespace
}  // namespace RayTracer

// Free functions
float RayTracer::intersectAABB(const BVHRay& ray, const Eigen::Vector3f& aabbMin,
                               const Eigen::Vector3f& aabbMax) {
    return VdbFields::intersectAABB(ray.origin, ray.invDirection, ray.hit.t, aabbMin, aabbMax);
}

namespace RayTracer {
BVHRay BVHRay::transform(const Eigen::Affine3f& transform) const {
    auto result = BVHRay {
        .origin =transform * this->origin,
        .direction = transform.rotation() * this->direction,
        .invDirection = Eigen::Vector3f(),
        .hit = this->hit
    };

    // cache direction inverse for computing triangle intersection
    result.invDirection = result.direction.cwiseInverse();
    return result;
}

// BVH mesh
/*static*/ BVHMesh BVHMesh::makeMesh(std::span<const Eigen::Vector3f> triangleSoup,
                                     std::span<const Eigen::Vector3f> normals,
                                     std::span<const Eigen::Vector2f> texUvs, Material material) {
    assert(triangleSoup.size() % 3 == 0);
    assert(normals.size() == triangleSoup.size());
    assert(texUv.size() == triangleSoup.size());

    std::vector<Triangle> result;
    std::vector<TriEx> triData;

    for (size_t ii = 0; ii < triangleSoup.size(); ii += 3) {
        const Eigen::Vector3f& v0 = triangleSoup[ii];
        const Eigen::Vector3f& v1 = triangleSoup[ii + 1];
        const Eigen::Vector3f& v2 = triangleSoup[ii + 2];
        result.push_back({v0, v1, v2, (v0 + v1 + v2) / 3});
        triData.push_back(TriEx{std::array{texUvs[ii], texUvs[ii + 1], texUvs[ii + 2]},
                                std::array{normals[ii], normals[ii + 1], normals[ii + 2]}});
    }

    return {std::move(result), std::move(triData), material};
}

// BVH implementation
struct BVH::BestSplitData {
    float cost = std::numeric_limits<float>::infinity();
    uint axis;
    float splitPos;
};

[[nodiscard]] BVH::BestSplitData BVH::findBestSplit(const BVHNode& node) const {
    BestSplitData result;

    for (uint ax = 0; ax < 3; ++ax) {
        float boundsMin = node.bounds.aabbMin[ax];
        float boundsMax = node.bounds.aabbMax[ax];

        if (boundsMin == boundsMax) {
            continue;
        }

        constexpr size_t numSplits = 100;
        float scale = (boundsMax - boundsMin) / numSplits;

        for (size_t ii : std::ranges::iota_view(size_t(0), numSplits)) {
            float candidatePos = boundsMin + ii * scale;
            float cost = evaluateSAH(node, ax, candidatePos);
            if (cost < result.cost) {
                result = BestSplitData{.cost = cost, .axis = ax, .splitPos = candidatePos};
            }
        }
    }

    return result;
}

float BVH::evaluateSAH(const BVHNode& node, uint axis, float splitPos) const {
    AABB boxL;
    AABB boxR;
    size_t triCountL = 0;
    size_t triCountR = 0;

    auto extend = [](AABB& aabb, const BVHMesh::Triangle& tri) {
        aabb.extend(tri.vs[0]);
        aabb.extend(tri.vs[1]);
        aabb.extend(tri.vs[2]);
    };

    for (auto triIdx : node.getTriangleIndices()) {
        const auto& triangle = getTriangle(triIdx);
        if (triangle.centroid[axis] < splitPos) {
            triCountL++;
            extend(boxL, triangle);
        } else {
            triCountR++;
            extend(boxR, triangle);
        }
    }

    return boxL.area() * triCountL + boxR.area() * triCountR;
}

void BVH::intersect(BVHRay& ray) const {
    int16_t stackPtr = 0;
    std::array<uint, getMaxDepth()> stack;
    stack[stackPtr] = m_rootIndex;

    for (; stackPtr >= 0;) {
        const BVHNode& node = m_nodes[stack[stackPtr--]];

        if (node.isLeaf()) {
            // Leaf node
            for (uint i = 0; i < node.triCount; ++i) {
                auto triIdx = node.leftFirst + i;
                const auto& [vs, centroid] = getTriangle(triIdx);
                if (auto intersectData =
                        intersectTriangle(ray.origin, ray.direction, vs[0], vs[1], vs[2]);
                    intersectData.t < ray.hit.t) {
                    ray.hit.t = intersectData.t;
                    ray.hit.uv = intersectData.uv;
                    ray.hit.setTriIdx(getTriIdx(triIdx));
                }
            }
        } else {
            // Internal node
            uint nodeIdx0 = node.leftFirst;
            uint nodeIdx1 = node.leftFirst + 1;
            auto& lChild = m_nodes[nodeIdx0];
            auto& rChild = m_nodes[nodeIdx1];

            float t0 = intersectAABB(ray, lChild.bounds.aabbMin, lChild.bounds.aabbMax);
            float t1 = intersectAABB(ray, rChild.bounds.aabbMin, rChild.bounds.aabbMax);

            if (t0 > t1) {
                std::swap(t0, t1);
                std::swap(nodeIdx0, nodeIdx1);
            }

            if (std::isfinite(t1)) {
                stack[++stackPtr] = nodeIdx1;
            }

            if (std::isfinite(t0)) {
                stack[++stackPtr] = nodeIdx0;
            }
        }
    }
}

void BVH::buildBVH() {
    m_nodes.resize(m_mesh->triangles.size() * 2 - 1);

    BVHNode& root = m_nodes[m_numNodes++];
    root.leftFirst = 0;
    root.triCount = static_cast<uint>(m_triIdx.size());

    updateNodeBounds(root);
    // call subdivide recursively
    subdivide(root);

    for (const auto& node : m_nodes) {
        if (node.isLeaf()) {
            for (auto ii : node.getTriangleIndices()) {
                const auto& [vs, centroid] = getTriangle(ii);
                assert(node.contains(vs[0]));
                assert(node.contains(vs[1]));
                assert(node.contains(vs[2]));
                assert(node.contains(centroid));
            }
        }
    }
}

void BVH::updateNodeBounds(BVHNode& node) {
    for (auto triIdx : node.getTriangleIndices()) {
        const auto& [vs, centroid] = getTriangle(triIdx);
        node.bounds.extend(vs[0]);
        node.bounds.extend(vs[1]);
        node.bounds.extend(vs[2]);
    }
}

std::pair<BVHNode&, BVHNode&> BVH::makeNewLeaves(BVHNode& node, uint splitTriIdx) {
    auto lIdx = m_numNodes++;
    auto rIdx = m_numNodes++;
    BVHNode& lChild = m_nodes[lIdx];
    BVHNode& rChild = m_nodes[rIdx];

    splitNode(node, lChild, rChild, splitTriIdx);

    updateNodeWithLeft(node, lIdx);

    updateNodeBounds(lChild);
    updateNodeBounds(rChild);

    return {lChild, rChild};
}

[[nodiscard]] std::optional<uint> BVH::findSplitTriIndex(BVHNode& node, uint axis,
                                                             float splitPos) {
    auto triL = node.leftFirst;
    auto triR = node.leftFirst + node.triCount;
    for (; triL != triR;) {
        if (getTriangle(triL).centroid[axis] > splitPos) {
            std::swap(m_triIdx[triL], m_triIdx[--triR]);
        } else {
            triL++;
        }
    }

    assert(node.leftFirst <= triL && triL <= node.leftFirst + node.triCount);
    if (node.leftFirst < triL && triL < node.leftFirst + node.triCount) {
        return triL;
    }

    return std::nullopt;
}

void BVH::subdivide(BVHNode& node, uint depth) {
    if (node.triCount <= 1 || depth >= getMaxDepth()) {
        return;
    }

    BestSplitData bestSplit = findBestSplit(node);

    if (node.bounds.aabbMin[bestSplit.axis] < bestSplit.splitPos &&
        bestSplit.splitPos < node.bounds.aabbMax[bestSplit.axis]) {
        if (auto triL = findSplitTriIndex(node, bestSplit.axis, bestSplit.splitPos)) {
            auto [leftChild, rightChild] = makeNewLeaves(node, triL.value());
            static_assert(std::is_same_v<decltype(leftChild), BVHNode&>);
            static_assert(std::is_same_v<decltype(rightChild), BVHNode&>);

            subdivide(leftChild, depth + 1);
            subdivide(rightChild, depth + 1);
        }
    }
}


// BVHInstance implementation
BVHInstance::BVHInstance(cow<BVH> bvh, const Eigen::Affine3f& worldFromGeom)
    : m_bvh(std::move(bvh)),
      m_worldFromGeom(worldFromGeom),
      m_geomFromWorld(worldFromGeom.inverse()),
      m_aabb_world(m_worldFromGeom * m_bvh.read().getRoot().bounds){};

void BVHInstance::intersect(BVHRay& ray_world) const {
    auto ray_geom = ray_world.transform(m_geomFromWorld);
    m_bvh->intersect(ray_geom);
    ray_world = ray_geom.transform(m_worldFromGeom);
}

// TLAS implementation
TLAS::TLAS(std::vector<BVHInstance> bvhInstance) : m_bvhInstances(std::move(bvhInstance)) {
}

[[nodiscard]] int TLAS::findBestMatch(std::span<const int> nodeIdices,
                                      int a) const {
    float smallestSurfaceArea_mm2 = std::numeric_limits<float>::infinity();
    int bestB = -1;

    for (int b = 0; b < nodeIdices.size(); ++b) {
        if (a == b) {
            continue;
        }

        auto bbox = m_tlasNode[nodeIdices[a]].aabb_world;
        bbox.extend(m_tlasNode[nodeIdices[b]].aabb_world);
        auto primDiagonal = (bbox.aabbMax - bbox.aabbMin);
        float surfaceArea_mm2 = bbox.area();
        if (surfaceArea_mm2 < smallestSurfaceArea_mm2) {
            smallestSurfaceArea_mm2 = surfaceArea_mm2;
            bestB = b;
        }
    }

    return bestB;
}

void TLAS::build() {
    // assign a TLASleaf node to each BLAS
    std::vector<int> nodesIdx(m_bvhInstances.size());
    m_tlasNode.resize(2 * m_bvhInstances.size());
    size_t nodesUsed = 1;
    for (size_t ii = 0; ii < m_bvhInstances.size(); ii++) {
        nodesIdx[ii] = nodesUsed;
        m_tlasNode[nodesUsed].aabb_world = m_bvhInstances[ii].getAABB_world();
        m_tlasNode[nodesUsed].bvhIdx = ii;
        m_tlasNode[nodesUsed++].leftRight = 0;  // makes it a leaf
    }

    auto numLeafNodes = m_bvhInstances.size();
    auto makeClusterNode = [this](const TLASNode& nodeA, int nodeIdxA, const TLASNode& nodeB,
                                  int nodeIdxB, int nodeIndices) {
        // Make new node
        TLASNode& newNode = m_tlasNode[nodeIndices];
        newNode.leftRight = nodeIdxA + (nodeIdxB << 16);
        newNode.aabb_world = nodeA.aabb_world;
        newNode.aabb_world.extend(nodeB.aabb_world);
    };

    int a = 0;
    int b = findBestMatch(nodesIdx, a);
    for (; numLeafNodes > 1;) {
        int c = findBestMatch(std::span(nodesIdx.begin(), numLeafNodes), b);
        if (a == c) {
            int nodeIdxA = nodesIdx[a];
            int nodeIdxB = nodesIdx[b];
            TLASNode& nodeA = m_tlasNode[nodeIdxA];
            TLASNode& nodeB = m_tlasNode[nodeIdxB];
            makeClusterNode(nodeA, nodeIdxA, nodeB, nodeIdxB, nodesUsed);
            nodesIdx[a] = nodesUsed++; // This is the new merged node Idx
            nodesIdx[b] = nodesIdx[--numLeafNodes]; // Swap the last leaf node into the b position
            b = findBestMatch(std::span(nodesIdx.begin(), numLeafNodes), a);
        } else {
            a = std::exchange(b, c);
        };
    }
    m_tlasNode[0] = m_tlasNode[nodesIdx[a]];
}

void TLAS::intersect(BVHRay& ray_world) const {
    std::array<TLASNode const*, getMaxDepth()> stack = {&m_tlasNode[0]};
    int16_t stackPtr = 0;
    for (;stackPtr >= 0;) {
        const auto& node = *stack[stackPtr--];
        if (node.isLeaf()) {
            auto  tempT = ray_world.hit.t;
            m_bvhInstances[node.bvhIdx].intersect(ray_world);
            if (tempT > ray_world.hit.t) {
                ray_world.hit.setBvhInstIdx(node.bvhIdx);
            }
            assert(tempT >= ray_world.t);
            continue;
        }

        auto const* child1 = &m_tlasNode[node.leftRight & 0xFFFF];
        auto const* child2 = &m_tlasNode[node.leftRight >> 16];

        auto t1 = intersectAABB(ray_world, child1->aabb_world.aabbMin, child1->aabb_world.aabbMax);
        auto t2 = intersectAABB(ray_world, child2->aabb_world.aabbMin, child2->aabb_world.aabbMax);
        if (t1 > t2) {
            std::swap(t1, t2);
            std::swap(child1, child2);
        }

        if (std::isfinite(t1)) {
            stack[++stackPtr] = child1;
            if (std::isfinite(t2)) {
                stack[++stackPtr] = child2;
            }
        }
    }
}
}  // namespace RayTracer
}  // namespace VdbFields