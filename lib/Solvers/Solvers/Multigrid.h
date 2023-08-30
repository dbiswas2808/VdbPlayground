#include <Eigen/Sparse>

namespace VdbFields::Solvers {
struct GridPointFromNodeNum {
    int xSize;
    [[nodiscard]] Eigen::Vector2i operator()(int nodeNum) const {
        return Eigen::Vector2i(nodeNum % xSize, nodeNum / xSize);
    }
};

struct NodeNumFromGridPoint {
    int xSize;
    [[nodiscard]] int operator()(Eigen::Vector2i nodeNum) const {
        return nodeNum[0] + nodeNum[1] * xSize;
    }
};

//! multigrid propagate step
[[nodiscard]] Eigen::SparseMatrix<float> prolongate(int fromNx, int fromNy);

//! multigrid restrict step
[[nodiscard]] Eigen::SparseMatrix<float> restrict(int fromNx, int fromNy);

// Gauss-Siedel method
void getGaussSiedel(Eigen::SparseVector<float>& vec, int xN, Eigen::SparseVector<float> rho,
                    Eigen::Vector2f dxdy);
}