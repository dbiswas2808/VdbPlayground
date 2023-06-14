#include <memory>

namespace VdbFields {
class UiStructures {};

class UiMesh : private UiStructures {
public:
  [[nodiscard]] static std::unique_ptr<UiMesh> registerToPolyscope();
};
} // namespace VdbFields