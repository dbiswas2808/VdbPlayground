#include <UI/UserInterface.h>
// #include <UI/Widgets.h>

#include <Core/Model.h>

#include <Core/ModelLoader.h>
#include <SkeletonGenerators/GrassfireDivergenceSkeleton.h>
#include <polyscope/surface_mesh.h>
#include <filesystem>

namespace VdbFields::UI {
UserInterface::UserInterface() {
    // Constructor called just once per instance of the program
    // Set view options
    polyscope::view::moveScale = 10.0;
    polyscope::view::projectionMode = polyscope::ProjectionMode::Orthographic;
    
    // Set options
    polyscope::options::programName = "Geometry processor";
    polyscope::options::ssaaFactor = 2;
    polyscope::options::screenshotExtension = std::filesystem::current_path().append("images/");

    polyscope::init();
}

[[nodiscard]] /*static*/ UserInterface &UserInterface::instance() {
    // Just one instance created for the whole program
    static UserInterface inst;
    return inst;
}

void UserInterface::testSkeletonize() const {
    auto model = Core::loadTestPolytopeMeshModel(20, 10);
    auto skeletonMesh = SkeletonGenerators::grassfireDivergenceSkeleton(model.m_mesh);
    auto *surfaceMesh =
        polyscope::registerSurfaceMesh("my points", skeletonMesh.m_points, skeletonMesh.m_triFaces);
    (void)surfaceMesh;  // Supress unused warning
}

void UserInterface::display() const { polyscope::show(); }
}  // namespace VdbFields::UI