#include <RayTracer/Bvh.h>

namespace VdbFields {
namespace RayTracer {
namespace {
[[nodiscard]] float intersectTriangle(const BVHRay& ray, const Eigen::Vector3f& v0,
                                      const Eigen::Vector3f& v1, const Eigen::Vector3f& v2) {
    Eigen::Vector3f e1 = v1 - v0;
    Eigen::Vector3f e2 = v2 - v0;
    Eigen::Vector3f h = ray.direction.cross(e2);
    float a = e1.dot(h);

    // Must be tangent to the plane
    if (-epsilon_mm<float> < a && a < epsilon_mm<float>) {
        return std::numeric_limits<float>::infinity();
    }

    float f = 1.0f / a;
    Eigen::Vector3f s = ray.origin - v0;
    float u = f * s.dot(h);

    if (not(0.f < u && u < 1.0f)) {
        return std::numeric_limits<float>::infinity();
    }

    Eigen::Vector3f q = s.cross(e1);
    float v = f * ray.direction.dot(q);

    if (not(0.0f < v && u + v < 1.0f)) {
        return std::numeric_limits<float>::infinity();
    }

    float t = f * e2.dot(q);

    if (t > epsilon_mm<float>) {
        return t;
    }

    return std::numeric_limits<float>::infinity();
}

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

float RayTracer::intersectAABB(const BVHRay& ray, const Eigen::Vector3f& aabbMin,
                               const Eigen::Vector3f& aabbMax) {
    const Eigen::Vector3f& invDir = ray.invDirection;
    float tx1 = (aabbMin.x() - ray.origin.x()) * invDir.x();
    float tx2 = (aabbMax.x() - ray.origin.x()) * invDir.x();
    float tmin = std::min(tx1, tx2);
    float tmax = std::max(tx1, tx2);

    float ty1 = (aabbMin.y() - ray.origin.y()) * invDir.y();
    float ty2 = (aabbMax.y() - ray.origin.y()) * invDir.y();
    tmin = std::max(tmin, std::min(ty1, ty2));
    tmax = std::min(tmax, std::max(ty1, ty2));

    float tz1 = (aabbMin.z() - ray.origin.z()) * invDir.z();
    float tz2 = (aabbMax.z() - ray.origin.z()) * invDir.z();

    tmin = std::max(tmin, std::min(tz1, tz2));
    tmax = std::min(tmax, std::max(tz1, tz2));

    return (tmax >= tmin && tmin < ray.t && tmax > 0) ? tmin
                                                      : std::numeric_limits<float>::infinity();
}

namespace RayTracer {
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
        aabb.extend(tri.v0);
        aabb.extend(tri.v1);
        aabb.extend(tri.v2);
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
                const auto& [v0, v1, v2, centroid] = getTriangle(node.leftFirst + i);
                if (auto t = intersectTriangle(ray, v0, v1, v2); t < ray.t) {
                    ray.t = t;
                    ray.normal = normal(v0, v1, v2);
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
                auto [v1, v2, v3, centroid] = getTriangle(ii);
                assert(node.contains(v1));
                assert(node.contains(v2));
                assert(node.contains(v3));
                assert(node.contains(centroid));
            }
        }
    }
}

void BVH::updateNodeBounds(BVHNode& node) {
    for (auto triIdx : node.getTriangleIndices()) {
        const auto& [v0, v1, v2, centroid] = getTriangle(triIdx);
        node.bounds.extend(v0);
        node.bounds.extend(v1);
        node.bounds.extend(v2);
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
        const auto& [v0, v1, v2, centroid] = getTriangle(triL);
        if (centroid[axis] > splitPos) {
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

void BVHInstance::intersect(BVHRay& ray) const {
    auto ray_geom = BVHRay {
        .origin =m_geomFromWorld * ray.origin,
        .direction = m_geomFromWorld.rotation() * ray.direction,
        .invDirection = Eigen::Vector3f(),
        .normal = m_geomFromWorld.rotation().transpose().inverse() * ray.normal,
        .t = ray.t
    };

    ray_geom.invDirection = ray_geom.direction.cwiseInverse();

    m_bvh->intersect(ray_geom);
    ray.t = ray_geom.t;
    ray.normal =  m_worldFromGeom.rotation().transpose().inverse() * ray_geom.normal;
}

TLAS::TLAS(std::vector<BVHInstance> bvhInstance) : m_bvhInstances(std::move(bvhInstance)) {
}

[[nodiscard]] int TLAS::findBestMatch(std::span<const int> tlasNodes,
                                      int a) const {
    float smallestSurfaceArea_mm2 = std::numeric_limits<float>::infinity();
    int bestB = -1;

    int b = 0;
    for (int nodeIdx : tlasNodes) {
        auto bbox = m_tlasNode[tlasNodes[a]].bounds;
        bbox.extend(m_tlasNode[nodeIdx].bounds);
        auto primDiagonal = (bbox.aabbMax - bbox.aabbMin);
        float surfaceArea_mm2 = bbox.area();
        if (surfaceArea_mm2 < smallestSurfaceArea_mm2) {
            smallestSurfaceArea_mm2 = surfaceArea_mm2;
            bestB = b++;
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
        m_tlasNode[nodesUsed].bounds = m_bvhInstances[ii].getAABB();
        m_tlasNode[nodesUsed].bvhIdx = ii;
        m_tlasNode[nodesUsed++].leftRight = 0;  // makes it a leaf
    }

    auto numLeafNodes = m_bvhInstances.size();
    auto makeClusterNode = [this](const TLASNode& nodeA, int nodeIdxA, const TLASNode& nodeB,
                                  int nodeIdxB, int nodeIndices) {
        // Make new node
        TLASNode& newNode = m_tlasNode[nodeIndices];
        newNode.leftRight = nodeIdxA + (nodeIdxB << 16);
        newNode.bounds = nodeA.bounds;
        newNode.bounds.extend(nodeB.bounds.aabbMin);
        newNode.bounds.extend(nodeB.bounds.aabbMax);
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

void TLAS::intersect(BVHRay& ray) const {
    std::array<TLASNode, 64> stack{m_tlasNode.front()};
    int16_t stackPtr = 0;
    for (;stackPtr >= 0;) {
        const auto& node = stack[stackPtr--];
        if (node.isLeaf()) {
            m_bvhInstances[node.bvhIdx].intersect(ray);
            --stackPtr;
            continue;
        }

        auto child1 = m_tlasNode[node.leftRight & 0xFFFF];
        auto child2 = m_tlasNode[node.leftRight >> 16];

        auto t1 = intersectAABB(ray, child1.bounds.aabbMin, child1.bounds.aabbMax);
        auto t2 = intersectAABB(ray, child2.bounds.aabbMin, child2.bounds.aabbMax);
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