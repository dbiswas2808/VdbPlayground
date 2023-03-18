#include <iostream>
#include <vector>
#include <ranges>
#include <Eigen/Dense>
#include <openvdb/openvdb.h>
#include <openvdb/tools/LevelSetPlatonic.h>
#include <openvdb/util/NullInterrupter.h>
#include <tbb/flow_graph.h>
#include "PrincipalCurvature.h"

using namespace Eigen;

template <typename T, size_t ...idxSeq>
int makeConsts(T n, std::index_sequence<idxSeq...>) {
    static_assert(std::is_unsigned_v<T>);
    return (static_cast<int>((1 << idxSeq & n) != 0) + ...);
}

int main(int, char**) {
    std::vector<int> v = {1, 2, 3};
    auto rv = std::ranges::views::reverse(v);
    std::cout << "Hello, world!\n" << *rv.begin() << "\n";
    Matrix<double, 4, 4> a;
    Matrix<double, 4, 4> b;
    auto grid = 
    openvdb::tools::createLevelSetOctahedron<openvdb::FloatGrid, openvdb::util::NullInterrupter>();
    auto c = a * b;
    std::cout<< c << "\n";

    fn();
}
