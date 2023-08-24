#include <Eigen/Sparse>

namespace VdbFields::RayTracer {
class Film {
      public:
      explicit Film(Eigen::Vector2i dimensions) : m_image(dimensions[1], 3 * dimensions[0]) {}
      void addSample(Eigen::Vector2i pixel, Eigen::Vector3f color);
      void imageToFile(std::string filename);

     private:
      Eigen::SparseMatrix<float> m_image;
};
}