#include <memory>
#include <numbers>
#include <optional>
#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <Eigen/Geometry> 
#include <RayTracer/Geometry.h>

namespace VdbFields::RayTracer {
namespace detail {
[[nodiscard]] Eigen::Matrix3f ndcFromPixel(Eigen::Vector2i shape_px, float fov_deg);
}

struct DefaultSampler {
      [[nodiscard]] Eigen::Vector2f operator()() const {
            return m_pixel.cast<float>() + 0.5 * Eigen::Vector2f(1, 1);
      }
      void setPixel(Eigen::Vector2i pixel) { m_pixel = pixel; }
      Eigen::Vector2i m_pixel;
};

template <class SamplerStrategy = DefaultSampler>
class Sampler {
     private:
      struct SampleGenerator {
            SampleGenerator(SamplerStrategy samplerStrategy)
                : m_samplerStrategy(std::move(samplerStrategy)) {}

            SampleGenerator& operator++() {
                  m_sample_px = m_samplerStrategy();
                  return *this;
            }

            constexpr void setPixel(const Eigen::Vector2i pixel) {
                  m_samplerStrategy.setPixel(pixel);
            }

            [[nodiscard]] constexpr Eigen::Vector2f operator*() const { return m_sample_px; }

           private:
            Eigen::Vector2i m_pixel = Eigen::Vector2i(0, 0);
            Eigen::Vector2f m_sample_px = Eigen::Vector2f(0, 0);
            SamplerStrategy m_samplerStrategy;
      };

     public:
      Sampler(SamplerStrategy samplerStrategy = SamplerStrategy())
          : generator{std::move(samplerStrategy)} {}
      ~Sampler() = default;

      constexpr void setPixel(const Eigen::Vector2i pixel) { generator.setPixel(pixel); }

      [[nodiscard]] constexpr Eigen::Vector2f getSample_px() {
            ++generator;
            return *generator;
      }

     private:
      SampleGenerator generator;
};

struct Ray {
      Eigen::Vector3f origin_world;
      Eigen::Vector3f direction_world;
      Eigen::Vector2f m_minMaxT_mm;
};

struct Camera {
    explicit Camera(Eigen::Vector3f origin_camera, Eigen::Vector2i shape_px, float fov_deg,
                    Eigen::Vector2f minMaxT_mm, Eigen::Matrix4f worldFromCamera);

    [[nodiscard]] Ray getRay(Eigen::Vector2f sample_px) const;

   private:
    Eigen::Vector3f m_origin_camera;
    Eigen::Vector2f m_minMaxT_mm;
    Eigen::Matrix3f m_ndcFromPixel;
    Eigen::MatrixX4f m_worldFromCamera;
};

class RayTracer {};

struct BRDF {};

struct RayIntersect {
      Eigen::Vector3f point_world;
      float hitT_mm;
};

class ShapeIntersector {
      public:
      virtual std::optional<RayIntersect> intersect(const Ray& ray) const = 0;
      virtual bool hasIntersection(const Ray& ray) const = 0;
      virtual BRDF getBRDF(const RayIntersect& intersect) const = 0;
};

class SphereIntersector : public ShapeIntersector {
   public:
    SphereIntersector(Sphere sphere, Eigen::Matrix4f worldFromGeom)
        : m_sphere(sphere), m_worldFromGeom(worldFromGeom) {}

    [[nodiscard]] virtual std::optional<RayIntersect> intersect(const Ray& ray) const final;
    [[nodiscard]] virtual bool hasIntersection(const Ray& ray) const final { return false; } 
    [[nodiscard]] virtual BRDF getBRDF(const RayIntersect& intersect) const final { return {}; }

   private:
    Eigen::Matrix4f m_worldFromGeom;
    Sphere m_sphere;
};

struct Material {
      Eigen::Vector3f color;
};

class GeometricPrimitive {
      Material mat;
      std::unique_ptr<Geometry> geometry;
      Eigen::Matrix4f worldFromObject;
};

class AggregatePrimitive {};
}  // namespace VdbFields::RayTracer