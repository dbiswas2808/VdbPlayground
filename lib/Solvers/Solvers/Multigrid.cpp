#include <Solvers/Multigrid.h>

namespace VdbFields {
// multigrid restrict step
Eigen::SparseMatrix<float> Solvers::prolongate(int fromNx, int fromNy) {
    int xCoarse = std::lround(std::pow(2, fromNx)) + 1;
    int yCoarse = std::lround(std::pow(2, fromNy)) + 1;
    GridPointFromNodeNum elemPosCoarse{xCoarse};
    NodeNumFromGridPoint nodeNumCoarse{xCoarse};

    int xFine = std::lround(std::pow(2, fromNx + 1)) + 1;
    int yFine = std::lround(std::pow(2, fromNy + 1)) + 1;
    GridPointFromNodeNum elemPosFine{xFine};
    NodeNumFromGridPoint nodeNumFine{xFine};

    int sizeCoarse = xCoarse * yCoarse;
    int sizeFine = xFine * yFine;

    Eigen::SparseMatrix<float> result{xFine * yFine, xCoarse * yCoarse};
    for (int nodeIdx = 0; nodeIdx < sizeCoarse; ++nodeIdx) {
        auto coarseElem = elemPosCoarse(nodeIdx);
        if (coarseElem.x() == 0 or coarseElem.y() == 0 or coarseElem.x() == xCoarse - 1 or
            coarseElem.y() == yCoarse - 1) {
            continue;
        }

        auto fineElem = coarseElem * 2;
        result.coeffRef(nodeNumFine(fineElem), nodeIdx) = 1.f;

        result.coeffRef(nodeNumFine(fineElem - Eigen::Vector2i(0, 1)), nodeIdx) = 0.5f;
        result.coeffRef(nodeNumFine(fineElem + Eigen::Vector2i(0, 1)), nodeIdx) = 0.5f;
        result.coeffRef(nodeNumFine(fineElem - Eigen::Vector2i(1, 0)), nodeIdx) = 0.5f;
        result.coeffRef(nodeNumFine(fineElem + Eigen::Vector2i(1, 0)), nodeIdx) = 0.5f;

        result.coeffRef(nodeNumFine(fineElem - Eigen::Vector2i(1, 1)), nodeIdx) = 0.25f;
        result.coeffRef(nodeNumFine(fineElem + Eigen::Vector2i(1, 1)), nodeIdx) = 0.25f;
        result.coeffRef(nodeNumFine(fineElem - Eigen::Vector2i(1, -1)), nodeIdx) = 0.25f;
        result.coeffRef(nodeNumFine(fineElem + Eigen::Vector2i(1, -1)), nodeIdx) = 0.25f;
    }

    return result;
}

Eigen::SparseMatrix<float> Solvers::restrict(const int fromNx, const int fromNy) {
    const int xCoarse = std::lround(std::pow(2, fromNx)) + 1;
    const int yCoarse = std::lround(std::pow(2, fromNy)) + 1;
    const GridPointFromNodeNum elemPosCoarse{xCoarse};
    const NodeNumFromGridPoint nodeNumCoarse{xCoarse};

    const int xFine = std::lround(std::pow(2, fromNx - 1)) + 1;
    const int yFine = std::lround(std::pow(2, fromNy - 1)) + 1;
    const GridPointFromNodeNum elemPosFine{xFine};
    const NodeNumFromGridPoint nodeNumFine{xFine};

    const int sizeCoarse = xCoarse * yCoarse;
    const int sizeFine = xFine * yFine;

    Eigen::SparseMatrix<float> result{xCoarse * yCoarse, xFine * yFine};
    for (int nodeIdx = 0; nodeIdx < sizeCoarse; ++nodeIdx) {
        auto coarseElem = elemPosCoarse(nodeIdx);
        if (coarseElem.x() == 0 or coarseElem.y() == 0 or coarseElem.x() == xCoarse - 1 or
            coarseElem.y() == yCoarse - 1) {
            continue;
        }

        auto fineElem = coarseElem * 2;
        result.coeffRef(nodeNumFine(fineElem), nodeIdx) = 0.25f;

        result.coeffRef(nodeIdx, nodeNumFine(fineElem - Eigen::Vector2i(0, 1))) = 1.f / 8;
        result.coeffRef(nodeIdx, nodeNumFine(fineElem + Eigen::Vector2i(0, 1))) = 1.f / 8;
        result.coeffRef(nodeIdx, nodeNumFine(fineElem - Eigen::Vector2i(1, 0))) = 1.f / 8;
        result.coeffRef(nodeIdx, nodeNumFine(fineElem + Eigen::Vector2i(1, 0))) = 1.f / 8;

        result.coeffRef(nodeIdx, nodeNumFine(fineElem - Eigen::Vector2i(1, 1))) = 1.f / 16;
        result.coeffRef(nodeIdx, nodeNumFine(fineElem + Eigen::Vector2i(1, 1))) = 1.f / 16;
        result.coeffRef(nodeIdx, nodeNumFine(fineElem - Eigen::Vector2i(1, -1))) = 1.f / 16;
        result.coeffRef(nodeIdx, nodeNumFine(fineElem + Eigen::Vector2i(1, -1))) = 1.f / 16;
    }

    return result;
}

void Solvers::getGaussSiedel(Eigen::SparseVector<float>& vec, int xN,
                             Eigen::SparseVector<float> rho, Eigen::Vector2f dxdy) {
    int cols =  std::lround(std::pow(2, xN)) + 1;
    int rows = vec.size() / cols;

    GridPointFromNodeNum gridPosFromNodeNum{xN};
    NodeNumFromGridPoint nodeNumFromGridPoint{xN};

    // implementation of Gauss-Siedel method
    // https://en.wikipedia.org/wiki/Gauss%E2%80%93Seidel_method
    for (int nodeIdx = 0; nodeIdx < vec.size(); ++nodeIdx) {
        auto gridPos = gridPosFromNodeNum(nodeIdx);
        if (gridPos[0] == 0 or gridPos[1] == 0 or gridPos[0] == cols - 1 or
            gridPos[1] == rows - 1) {
            continue;
        }

        auto& gridVal = vec.coeffRef(nodeIdx);
        gridVal = std::pow(dxdy.prod(), 2) * rho.coeff(nodeIdx) -
                  dxdy[1] * dxdy[1] *
                      (vec.coeff(nodeNumFromGridPoint(gridPos - Eigen::Vector2i(1, 0))) +
                       vec.coeff(nodeNumFromGridPoint(gridPos + Eigen::Vector2i(1, 0)))) -
                  dxdy[0] * dxdy[0] *
                      (vec.coeff(nodeNumFromGridPoint(gridPos - Eigen::Vector2i(0, 1))) +
                       vec.coeff(nodeNumFromGridPoint(gridPos + Eigen::Vector2i(0, 1))));
        gridVal /= -2 * (dxdy[0] * dxdy[0] + dxdy[1] * dxdy[1]);
    }
}
}  // namespace VdbFields