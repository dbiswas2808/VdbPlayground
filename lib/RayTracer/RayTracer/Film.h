#pragma once
#include <Eigen/Core>

namespace VdbFields::RayTracer {
class Film {
   public:
    explicit Film(Eigen::Vector2i shape_px)
        : m_shape_px(shape_px), m_image(Eigen::MatrixXf::Zero(m_shape_px[1], 3 * m_shape_px[0])) {}
    [[nodiscard]] Eigen::Vector2i getShape_px() const { return m_shape_px; }

    void addSample(Eigen::Vector2i pixel, Eigen::Vector3f color);
    void imageToFile(std::string filename) const;

   private:
    Eigen::Vector2i m_shape_px;
    Eigen::MatrixXf m_image;
};
}