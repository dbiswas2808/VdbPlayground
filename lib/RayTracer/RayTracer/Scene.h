#include <RayTracer/SceneEntity.h>
#include <RayTracer/Film.h>

namespace VdbFields::RayTracer {
class Scene {
     public:
      Scene(Sampler<> sampler, Camera camera, RayTracer rayTrace, Film film)
          : m_sampler(sampler), m_camera(camera), m_rayTracer(rayTrace), m_film(film) {}

      void rayTrace();

     private:
      Sampler<> m_sampler;
      Camera m_camera;
      RayTracer m_rayTracer;
      Film m_film;
};
}  // namespace VdbFields::RayTracer