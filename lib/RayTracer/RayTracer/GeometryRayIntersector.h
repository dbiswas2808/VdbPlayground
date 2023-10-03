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

    [[nodiscard]] virtual std::optional<RayIntersect> intersect(const Ray& ray_world) const {
        return m_intersector->intersect(ray_world);
    }

    [[nodiscard]] virtual bool hasIntersection(const Ray& ray_world) const {
        return m_intersector->hasIntersection(ray_world);
    }

   private:
    std::unique_ptr<Concept> m_intersector;
};

class SphereIntersector {
   public:
    SphereIntersector(Sphere sphere, const Eigen::Affine3f& worldFromGeom, Material material)
        : m_sphere(sphere), m_worldFromGeom(worldFromGeom), m_material(material) {}

    [[nodiscard]] virtual std::optional<RayIntersect> intersect(const Ray& ray_world) const;
    [[nodiscard]] virtual bool hasIntersection(const Ray& ray_world) const;

   private:
    Eigen::Affine3f m_worldFromGeom;
    Sphere m_sphere;
    Material m_material;
};

class TriMeshIntersector {
   public:
    TriMeshIntersector(std::span<const Eigen::Vector3f> triangleSoup, Eigen::Affine3f worldFromGeom,
                       Material material)
        : bvh(cow<BVHMesh>(BVHMesh::makeMesh(triangleSoup,
                                             std::vector(triangleSoup.size(), Eigen::Vector3f()),
                                             std::vector(triangleSoup.size(), Eigen::Vector2f())))),
          m_worldFromGeom(worldFromGeom),
          m_geomFromWorld(worldFromGeom.inverse()),
          m_material(material) {}

    [[nodiscard]] virtual std::optional<RayIntersect> intersect(const Ray& ray) const final;

    [[nodiscard]] virtual bool hasIntersection(const Ray& ray) const final {
        return static_cast<bool>(intersect(ray));
    }

   private:
    Eigen::Affine3f m_worldFromGeom;
    Eigen::Affine3f m_geomFromWorld;
    BVH bvh;
    Material m_material;
};

class AggregratePrimitiveIntersector {
   public:
    AggregratePrimitiveIntersector(std::vector<ShapeIntersector> shapeIntersectors)
        : m_shapeIntersectors(std::move(shapeIntersectors)) {}

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