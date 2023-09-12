#include <Eigen/Geometry>

namespace VdbFields::RayTracer {
class Light {
    class Concept {
       public:
        virtual ~Concept() = default;
        [[nodiscard]] virtual Eigen::Vector3f getDirection(const Eigen::Vector3f& point) const = 0;
        [[nodiscard]] virtual float getIntensity(const Eigen::Vector3f& point) const = 0;
        [[nodiscard]] virtual std::unique_ptr<Concept> clone() const = 0;
    };

    template <class T>
    class Model : public Concept {
       public:
        struct DisableCopy {};

        template <class... Args>
        Model(DisableCopy, Args&&... args) : m_t(std::forward<Args>(args)...) {}

        Model(const Model& model) = default;
        Model(Model&& model) = delete;

        [[nodiscard]] virtual Eigen::Vector3f getDirection(
            const Eigen::Vector3f& point) const override {
            return m_t.getDirection(point);
        }
        [[nodiscard]] virtual float getIntensity(const Eigen::Vector3f& point) const override {
            return m_t.getIntensity(point);
        }
        [[nodiscard]] virtual std::unique_ptr<Concept> clone() const override {
            return std::make_unique<Model>(DisableCopy{}, m_t);
        }

       private:
        T m_t;
    };

    Light(std::unique_ptr<Concept> c) : m_light(std::move(c)) {}

   public:
    template <class T, class... Args>
    static Light fromImpl(Args&&... args) {
        return Light(std::make_unique<Model<T>>(typename Model<T>::DisableCopy{},
                                                std::forward<Args>(args)...));
    }

    Light(const Light& other) : m_light(other.m_light->clone()) {}

    Light(Light&& other) : m_light(std::move(other.m_light)) {}

    virtual ~Light() = default;

    [[nodiscard]] virtual Eigen::Vector3f getDirection(const Eigen::Vector3f& point) const {
        return m_light->getDirection(point);
    }
    [[nodiscard]] virtual float getIntensity(const Eigen::Vector3f& point) const {
        return m_light->getIntensity(point);
    }

    std::unique_ptr<Concept> m_light;
};

class PointLight {
   public:
    PointLight(const Eigen::Vector3f& position, float intensity)
        : m_position(position), m_intensity(intensity) {}

    [[nodiscard]] Eigen::Vector3f getDirection(const Eigen::Vector3f& point) const {
        return (m_position - point).normalized();
    }

    [[nodiscard]] float getIntensity(const Eigen::Vector3f& point) const {
        return m_intensity;
    }

   private:
    Eigen::Vector3f m_position;
    float m_intensity;
};

class DirectionalLight {
   public:
    DirectionalLight(const Eigen::Vector3f& direction, float intensity)
        : m_direction(direction), m_intensity(intensity) {}

    [[nodiscard]] Eigen::Vector3f getDirection(const Eigen::Vector3f& point) const {
        return m_direction;
    }

    [[nodiscard]] float getIntensity(const Eigen::Vector3f& point) const {
        return m_intensity;
    }

   private:
    Eigen::Vector3f m_direction;
    float m_intensity;
};

class AmbientLight {
   public:
    explicit AmbientLight(float intensity) : m_intensity(intensity) {}

    [[nodiscard]] Eigen::Vector3f getDirection(const Eigen::Vector3f& point) const {
        return Eigen::Vector3f(0, 0, 0);
    }

    [[nodiscard]] float getIntensity(const Eigen::Vector3f& point) const {
        return m_intensity;
    }

   private:
    float m_intensity;
};

// Line source
class LineLight {
   public:
    LineLight(const Eigen::Vector3f& position, const Eigen::Vector3f& direction, float intensity)
        : m_position(position), m_direction(direction), m_intensity(intensity) {}

    [[nodiscard]] Eigen::Vector3f getDirection(const Eigen::Vector3f& point) const {
        return m_direction;
    }

    [[nodiscard]] float getIntensity(const Eigen::Vector3f& point) const {
        return m_intensity;
    }

   private:
    Eigen::Vector3f m_position;
    Eigen::Vector3f m_direction;
    float m_intensity;
};

// Area source
class AreaLight {
   public:
    AreaLight(const Eigen::Vector3f& position, const Eigen::Vector3f& direction, float intensity)
        : m_position(position), m_direction(direction), m_intensity(intensity) {}

    [[nodiscard]] Eigen::Vector3f getDirection(const Eigen::Vector3f& point) const {
        return m_direction;
    }

    [[nodiscard]] float getIntensity(const Eigen::Vector3f& point) const {
        return m_intensity;
    }

   private:
    Eigen::Vector3f m_position;
    Eigen::Vector3f m_direction;
    float m_intensity;
};
}  // namespace VdbFields::RayTracer