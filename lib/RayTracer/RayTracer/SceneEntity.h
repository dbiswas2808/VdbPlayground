#include <memory>
#include <numbers>
#include <optional>
#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <Eigen/Geometry> 
#include <RayTracer/Geometry.h>

namespace VdbFields::RayTracer {
struct Ray;

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

[[nodiscard]] Eigen::Matrix4f lookAt_cameraFromWorld(Eigen::Vector3f eye_world,
                                                     Eigen::Vector3f target_world,
                                                     Eigen::Vector3f up_world);

struct Camera {
    explicit Camera(Eigen::Vector3f origin_camera, 
                           Eigen::Vector2i shape_px, float fov_deg,
                           Eigen::Vector2f minMaxT_mm);

    [[nodiscard]] Ray getRay_camera(Eigen::Vector2f sample_px) const;

   private:
    Eigen::Vector3f m_origin_camera;
    Eigen::Vector2f m_minMaxT;
    Eigen::Matrix3f m_ndcFromPixel;
};

class RayTracer {};

struct Material {
      Eigen::Vector3f color;
};

class AggregatePrimitive {};
}  // namespace VdbFields::RayTracer