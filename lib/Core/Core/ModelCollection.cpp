#include <Core/ModelCollection.h>

namespace VdbFields::Core {
  void Scene::addModel(Model &&model) {
    m_modelCollection.push_back(std::move(model));
  }

  [[nodiscard]] const std::vector<Model> &Scene::models() const {
    return m_modelCollection;
  }

}
