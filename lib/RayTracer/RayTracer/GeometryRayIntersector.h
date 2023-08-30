#include <memory>
#include <optional>
#include <RayTracer/Geometry.h>

namespace VdbFields::RayTracer {
struct Ray {
    Eigen::Vector3f origin;
    Eigen::Vector3f direction;
    Eigen::Vector2f m_minMaxT;

    [[nodiscard]] Ray transform(Eigen::Matrix4f tx) const;
};

struct RayIntersect {
    Eigen::Vector3f point_world;
    float hitT_mm;
    Eigen::Vector3f normal_world;
};

struct BRDF {};

class ShapeIntersector {
    struct Concept {
        [[nodiscard]] virtual std::optional<RayIntersect> intersect(const Ray& ray) const = 0;
        [[nodiscard]] virtual bool hasIntersection(const Ray& ray) const = 0;
        [[nodiscard]] virtual BRDF getBRDF(const RayIntersect& intersect) const = 0;
        [[nodiscard]] virtual std::unique_ptr<Concept> clone() const = 0;
    };

    template <class T>
    struct Model : Concept {
        struct DisableCopy {};

        template <class... U>
        Model(DisableCopy, U&&... t) : m_t(std::forward<U>(t)...) {}

        Model(const Model& other) = delete;

        [[nodiscard]] virtual std::optional<RayIntersect> intersect(const Ray& ray_camera) const final {
            return m_t.intersect(ray_camera);
        }

        [[nodiscard]] virtual bool hasIntersection(const Ray& ray_camera) const final {
            return m_t.hasIntersection(ray_camera);
        }

        [[nodiscard]] virtual BRDF getBRDF(const RayIntersect& intersect) const final {
            return m_t.getBRDF(intersect);
        }

        [[nodiscard]] virtual std::unique_ptr<Concept> clone() const {
            return std::make_unique<Model>(DisableCopy{}, m_t);
        }

        T m_t;
    };

    explicit ShapeIntersector(std::unique_ptr<Concept> c) : m_intersector(std::move(c)) {}

   public:
    template <class T, class... Args>
    [[nodiscard]] static ShapeIntersector fromImpl(Args&&... arg) {
        return ShapeIntersector(std::make_unique<Model<T>>(typename Model<T>::DisableCopy{},
                                                           std::forward<Args>(arg)...));
    }

    explicit ShapeIntersector(const ShapeIntersector& other)
        : m_intersector(other.m_intersector->clone()) {}
    ShapeIntersector(ShapeIntersector&& other) : m_intersector(std::move(other.m_intersector)) {}

    [[nodiscard]] virtual std::optional<RayIntersect> intersect(const Ray& ray_camera) const {
        return m_intersector->intersect(ray_camera);
    }
    [[nodiscard]] virtual bool hasIntersection(const Ray& ray_camera) const {
        return m_intersector->hasIntersection(ray_camera);
    }
    [[nodiscard]] virtual BRDF getBRDF(const RayIntersect& intersect) const {
        return m_intersector->getBRDF(intersect);
    }

   private:
    std::unique_ptr<Concept> m_intersector;
};

class SphereIntersector {
   public:
    SphereIntersector(Sphere sphere, Eigen::Matrix4f worldFromCamera, Eigen::Matrix4f worldFromGeom)
        : m_sphere(sphere), m_worldFromCamera(worldFromCamera), m_worldFromGeom(worldFromGeom) {}

    [[nodiscard]] virtual std::optional<RayIntersect> intersect(const Ray& ray_camera) const final;
    [[nodiscard]] virtual bool hasIntersection(const Ray& ray_camera) const final { return false; }
    [[nodiscard]] virtual BRDF getBRDF(const RayIntersect& intersect) const final { return {}; }

   private:
    Eigen::Matrix4f m_worldFromCamera;
    Eigen::Matrix4f m_worldFromGeom;
    Sphere m_sphere;
};

// class TriMeshIntersector {
//    public:
//     TriMeshIntersector(TriMesh&& triMesh, Eigen::Matrix4f worldFromGeom)
//         : m_triMesh(triMesh), m_worldFromGeom(worldFromGeom) {}

//     [[nodiscard]] virtual std::optional<RayIntersect> intersect(const Ray& ray) const final;
//     [[nodiscard]] virtual bool hasIntersection(const Ray& ray) const final { return false; }
//     [[nodiscard]] virtual BRDF getBRDF(const RayIntersect& intersect) const final { return {}; }

//    private:
//     Eigen::Matrix4f m_worldFromGeom;
//     TriMesh m_triMesh;
// };
}  // namespace VdbFields::RayTracer