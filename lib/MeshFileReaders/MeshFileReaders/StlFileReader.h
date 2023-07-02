#include <array>
#include <string>
#include <vector>

namespace VdbFields::MeshFileReaders {
struct StlData {
    std::vector<std::array<float, 3>> vertices;
    std::vector<std::array<int, 3>> faces;
};

[[nodiscard]] StlData readStlFile(const std::string& filename);
}