#pragma once
#include <Eigen/Core>

namespace VdbFields::RayTracer {
class Film {
   public:
    explicit Film(const Eigen::Vector2i& shape_px);
    [[nodiscard]] Eigen::Vector2i getShape_px() const { return m_shape_px; }

    void addSample(Eigen::Vector2i pixel, Eigen::Vector3f color);
    void imageToFile(std::string filename) const;

   private:
    Eigen::Vector2i m_shape_px;
    Eigen::MatrixXf m_image;
};
}