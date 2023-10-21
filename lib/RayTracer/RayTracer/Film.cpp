#include <RayTracer/Film.h>
#include <png.h>

namespace VdbFields::RayTracer {
namespace {
void writePngGray(Eigen::MatrixXf denseImage, std::string filename) {
    // auto denseImage = image.toDense();
    const float minValue = denseImage.minCoeff();
    const float maxValue = denseImage.maxCoeff();
    const int width = denseImage.cols() / 3;
    const int height = denseImage.rows();

    auto minMat = Eigen::MatrixXf(height, 3 * width);
    minMat.setConstant(minValue);
    denseImage = 255.0f * (denseImage - minMat) / (maxValue - minValue);

    FILE* fp = fopen(filename.c_str(), "wb");
    if (!fp) {
        throw std::runtime_error("Error opening file for writing\n");
    }

    png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    if (!png) {
        fclose(fp);
        throw std::runtime_error("Error opening file for writing\n");
    }

    png_infop info = png_create_info_struct(png);
    if (!info) {
        png_destroy_write_struct(&png, nullptr);
        fclose(fp);
        throw std::runtime_error("Error opening file for writing\n");
    }

    png_init_io(png, fp);
    png_set_IHDR(png, info, width, height, 8, PNG_COLOR_TYPE_GRAY, PNG_INTERLACE_NONE,
                 PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);
    png_write_info(png, info);

    std::vector<png_byte> row(width); // Gray has 1 channel

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            row[x] = static_cast<png_byte>(denseImage(y, 3 * x));       // Red channel
        }
        png_write_row(png, row.data());
    }

    png_write_end(png, nullptr);

    // Cleanup
    png_destroy_write_struct(&png, &info);
    fclose(fp);
}

void writePngRGB(Eigen::MatrixXf denseImage, std::string filename) {
    // auto denseImage = image.toDense();
    const float minValue = denseImage.minCoeff();
    const float maxValue = denseImage.maxCoeff();
    const int width = denseImage.cols();
    const int height = denseImage.rows();

    auto minMat = Eigen::MatrixXf(height, width);
    minMat.setConstant(minValue);
    denseImage = 255.0f * (denseImage - minMat) / (maxValue - minValue);

    FILE* fp = fopen(filename.c_str(), "wb");
    if (!fp) {
        throw std::runtime_error("Error opening file for writing\n");
    }

    png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    if (!png) {
        fclose(fp);
        throw std::runtime_error("Error opening file for writing\n");
    }

    png_infop info = png_create_info_struct(png);
    if (!info) {
        png_destroy_write_struct(&png, nullptr);
        fclose(fp);
        throw std::runtime_error("Error opening file for writing\n");
    }

    png_init_io(png, fp);
    png_set_IHDR(png, info, width / 3, height, 8, PNG_COLOR_TYPE_RGB, PNG_INTERLACE_NONE,
                 PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);
    png_write_info(png, info);

    std::vector<png_byte> row(width); // RGB has 3 channels

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            row[x] = static_cast<png_byte>(denseImage(y, x));       // Red channel
        }
        png_write_row(png, row.data());
    }

    png_write_end(png, nullptr);

    // Cleanup
    png_destroy_write_struct(&png, &info);
    fclose(fp);
}
}  // namespace

Film::Film(const Eigen::Vector2i& shape_px)
    : m_shape_px(shape_px), m_image(Eigen::MatrixXf::Zero(m_shape_px[1], 3 * m_shape_px[0])) {}

void Film::addSample(Eigen::Vector2i pixel, Eigen::Vector3f color) {
    if (color != Eigen::Vector3f::Zero()) {
        m_image.coeffRef(pixel[1], 3 * pixel[0]) += color[0];
        m_image.coeffRef(pixel[1], 3 * pixel[0] + 1) += color[1];
        m_image.coeffRef(pixel[1], 3 * pixel[0] + 2) += color[2];
    }
}

void Film::imageToFile(std::string filename) const {
    writePngRGB(m_image, filename);
}
}  // namespace VdbFields::RayTracer