#include <Eigen/Geometry>

namespace VdbFields::RayTracer {
class Light {
   public:
    virtual ~Light() = default;
    [[nodiscard]] virtual Eigen::Vector3f getDirection(const Eigen::Vector3f& point) const = 0;
    [[nodiscard]] virtual Eigen::Vector3f getIntensity(const Eigen::Vector3f& point) const = 0;
};


class PointLight : public Light {
   public:
    PointLight(const Eigen::Vector3f& position, const Eigen::Vector3f& intensity)
        : m_position(position), m_intensity(intensity) {}

    [[nodiscard]] Eigen::Vector3f getDirection(const Eigen::Vector3f& point) const override {
        return (m_position - point).normalized();
    }

    [[nodiscard]] Eigen::Vector3f getIntensity(const Eigen::Vector3f& point) const override {
        return m_intensity;
    }

   private:
    Eigen::Vector3f m_position;
    Eigen::Vector3f m_intensity;
};

class DirectionalLight : public Light {
   public:
    DirectionalLight(const Eigen::Vector3f& direction, const Eigen::Vector3f& intensity)
        : m_direction(direction), m_intensity(intensity) {}

    [[nodiscard]] Eigen::Vector3f getDirection(const Eigen::Vector3f& point) const override {
        return m_direction;
    }

    [[nodiscard]] Eigen::Vector3f getIntensity(const Eigen::Vector3f& point) const override {
        return m_intensity;
    }

   private:
    Eigen::Vector3f m_direction;
    Eigen::Vector3f m_intensity;
};

class AmbientLight : public Light {
   public:
    explicit AmbientLight(const Eigen::Vector3f& intensity) : m_intensity(intensity) {}

    [[nodiscard]] Eigen::Vector3f getDirection(const Eigen::Vector3f& point) const override {
        return Eigen::Vector3f(0, 0, 0);
    }

    [[nodiscard]] Eigen::Vector3f getIntensity(const Eigen::Vector3f& point) const override {
        return m_intensity;
    }

   private:
    Eigen::Vector3f m_intensity;
};

// Line source
class LineLight : public Light {
   public:
    LineLight(const Eigen::Vector3f& position, const Eigen::Vector3f& direction,
              const Eigen::Vector3f& intensity)
        : m_position(position), m_direction(direction), m_intensity(intensity) {}

    [[nodiscard]] Eigen::Vector3f getDirection(const Eigen::Vector3f& point) const override {
        return m_direction;
    }

    [[nodiscard]] Eigen::Vector3f getIntensity(const Eigen::Vector3f& point) const override {
        return m_intensity;
    }

   private:
    Eigen::Vector3f m_position;
    Eigen::Vector3f m_direction;
    Eigen::Vector3f m_intensity;
};

// Area source
class AreaLight : public Light {
   public:
    AreaLight(const Eigen::Vector3f& position, const Eigen::Vector3f& direction,
              const Eigen::Vector3f& intensity)
        : m_position(position), m_direction(direction), m_intensity(intensity) {}

    [[nodiscard]] Eigen::Vector3f getDirection(const Eigen::Vector3f& point) const override {
        return m_direction;
    }

    [[nodiscard]] Eigen::Vector3f getIntensity(const Eigen::Vector3f& point) const override {
        return m_intensity;
    }

   private:
    Eigen::Vector3f m_position;
    Eigen::Vector3f m_direction;
    Eigen::Vector3f m_intensity;
};
}  // namespace VdbFields::RayTracer