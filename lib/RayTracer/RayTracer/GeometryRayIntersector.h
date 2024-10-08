#pragma once
#include <memory>
#include <optional>
#include <span>

#include <RayTracer/Bvh.h>
#include <RayTracer/Geometry.h>
#include <RayTracer/Ray.h>

namespace VdbFields::RayTracer {
// Using external polymorphism for shape intersections to avoid inheritence kludge
class ShapeIntersector {
    struct Concept {
        [[nodiscard]] virtual std::optional<RayIntersect> intersect(const Ray& ray) const = 0;
        [[nodiscard]] virtual bool hasIntersection(const Ray& ray) const = 0;
        [[nodiscard]] virtual std::unique_ptr<Concept> clone() const = 0;

        virtual ~Concept() = default;
    };

    template <class T>
    struct Model : Concept {
        struct DisableCopy {};

        template <class... U>
        Model(DisableCopy, U&&... t) : m_t(std::forward<U>(t)...) {}

        Model(const Model& other) = delete;

        [[nodiscard]] virtual std::optional<RayIntersect> intersect(
            const Ray& ray_world) const final {
            return m_t.intersect(ray_world);
        }

        [[nodiscard]] virtual bool hasIntersection(const Ray& ray_world) const final {
            return m_t.hasIntersection(ray_world);
        }

        [[nodiscard]] virtual std::unique_ptr<Concept> clone() const {
            return std::make_unique<Model>(DisableCopy{}, m_t);
        }

        T m_t;
    };

    explicit ShapeIntersector(std::unique_ptr<Concept> c) : m_intersector(std::move(c)) {}

   public:
    template <class T, class... Args>
    [[nodiscard]] static ShapeIntersector fromImpl(Args&&... args) {
        return ShapeIntersector(std::make_unique<Model<T>>(typename Model<T>::DisableCopy{},
                                                           std::forward<Args>(args)...));
    }

    ShapeIntersector(const ShapeIntersector& other) : m_intersector(other.m_intersector->clone()) {}
    ShapeIntersector(ShapeIntersector&& other) : m_intersector(std::move(other.m_intersector)) {}

    [[nodiscard]] std::optional<RayIntersect> intersect(const Ray& ray_world) const {
        return m_intersector->intersect(ray_world);
    }

    [[nodiscard]] bool hasIntersection(const Ray& ray_world) const {
        return m_intersector->hasIntersection(ray_world);
    }

   private:
    std::unique_ptr<Concept> m_intersector;
};

class SphereIntersector {
   public:
    SphereIntersector(Sphere sphere, const Eigen::Affine3f& worldFromGeom, Material material);
    [[nodiscard]] std::optional<RayIntersect> intersect(const Ray& ray_world) const;
    [[nodiscard]] bool hasIntersection(const Ray& ray_world) const;
    [[nodiscard]] static AABB makeAABB_world(const Sphere& sphere, const Eigen::Affine3f& worldFromGeom);

   private:
    Eigen::Affine3f m_worldFromGeom;
    AABB m_aabb_world;
    Sphere m_sphere;
    Material m_material;
};

class TriMeshIntersector {
   public:
    TriMeshIntersector(std::span<const Eigen::Vector3f> triangleSoup,
                       std::span<const Eigen::Vector3f> vtxNormals,
                       std::span<const Eigen::Vector2f> vtxTexCoords, Eigen::Affine3f worldFromGeom,
                       Material material);

    [[nodiscard]] std::optional<RayIntersect> intersect(const Ray& ray) const;

    [[nodiscard]] bool hasIntersection(const Ray& ray) const {
        return static_cast<bool>(intersect(ray));
    }

   private:
    BVHInstance m_bvh;
};

class AggregateMeshIntersector {
   public:
    AggregateMeshIntersector() = default;

    [[nodiscard]] std::optional<RayIntersect> intersect(const Ray& ray_world) const;
    [[nodiscard]] bool hasIntersection(const Ray& ray_world) const;
    void addMeshIntersector(cow<BVHMesh> mesh, const Eigen::Affine3f& worldFromGeom);
    void buildTlas();

   private:
    std::vector<BVHInstance> m_bvhInstance;
    cow<TLAS> m_tlas;
};

class AggregratePrimitiveIntersector {
   public:
    explicit AggregratePrimitiveIntersector(std::vector<ShapeIntersector> shapeIntersectors);
    [[nodiscard]] virtual std::optional<RayIntersect> intersect(const Ray& ray_world) const;
    [[nodiscard]] virtual bool hasIntersection(const Ray& ray_world) const {
        return intersect(ray_world).has_value();
    }

    void addShapeIntersector(ShapeIntersector shapeIntersector) {
        m_shapeIntersectors.push_back(std::move(shapeIntersector));
    }

   private:
    std::vector<ShapeIntersector> m_shapeIntersectors;
};
}  // namespace VdbFields::RayTracer