#include <Solvers/Multigrid.h>
#include <catch2/catch_all.hpp>

namespace VdbFields {
TEST_CASE("Test Poisson solver") {
    Eigen::SparseVector<float> gridU(3 * 3);
    Eigen::SparseVector<float> rho(3*3);

    Solvers::GridPointFromNodeNum elemPos{3};
    Solvers::NodeNumFromGridPoint nodeNum{3};
    
    auto gridP = Eigen::Vector2i(1, 1);
    gridU.coeffRef(nodeNum(gridP)) = 1.f;
    rho.coeffRef(nodeNum(gridP)) = 1.f;

    Eigen::Vector2f dxdy(1.f, 1.f);
    Solvers::getGaussSiedel(gridU, 3, rho, dxdy);
    CHECK(gridU.coeffRef(nodeNum(gridP)) == -0.25f);
}
}