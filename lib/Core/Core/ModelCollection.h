#include <Core/Model.h>

namespace VdbFields::Core {
class Scene {
public:
  Scene() = default;
  void addModel(Model &&model);

  [[nodiscard]] const std::vector<Model> &models() const;

private:
  std::vector<Model> m_modelCollection;
};
} // namespace VdbFields::Core