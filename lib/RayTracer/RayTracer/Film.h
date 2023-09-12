#pragma once
#include <Eigen/Core>

namespace VdbFields::RayTracer {
class Film {
      public:
       explicit Film(Eigen::Vector2i dimensions)
           : m_image(Eigen::MatrixXf::Zero(dimensions[1], 3 * dimensions[0])) {}
       void addSample(Eigen::Vector2i pixel, Eigen::Vector3f color);
       void imageToFile(std::string filename);

      private:
       Eigen::MatrixXf m_image;
};
}