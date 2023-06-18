#include <memory>
#include <mutex>
#include <vector>

#include <polyscope/polyscope.h>
#include <polyscope/view.h>
#pragma GCC optimize ("O0")
namespace VdbFields::UI {
/// @brief Should be singleton as per application instance you can have only a
/// single instance of polyscope. Current implementation focuses on structures,
/// we'll later add support for quantities and field displays.
class UserInterface {
    UserInterface();

   public:
    [[nodiscard]] static UserInterface &instance();

    /// @brief registers object of ui-structures to polyscope. Every UiStructureT
    /// should implement UiStructureT::registerToPolyscope that registers the
    /// object with polyscope and returns a ponter to the underlying
    /// polyscope::Structure
    /// @tparam UiStructureT This type should implement
    /// UiStructureT::registerToPolyscope that returns a pointer to the underlying
    /// polyscope::Structure type
    /// @param structure
    template <typename UiStructureT>
    void registerStructures(const UiStructureT &structure) const {
        std::lock_guard l{m};
        UiStructureT::registerToPolyscope(structure);
    }

    void testSkeletonize() const;

    void display() const;

   private:
    std::mutex m;
};

}  // namespace VdbFields::UI