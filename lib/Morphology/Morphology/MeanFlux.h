#include <openvdb/math/Coord.h>
#include <openvdb/openvdb.h>
#include <openvdb/tools/GridOperators.h>
#include <openvdb/tools/MeshToVolume.h>
#include <fstream>

#include <ranges>

namespace VdbFields {
namespace Morphology {
/// @brief Flux numerical scheme type. MeanFluxOp is specialized on these values
/// for concrete implementations.
enum class MeanFluxScheme { NEIGHBOR_26, NEIGHBOR_98 };

/// @brief Declaration for MeanFluxOp template. Should be specialized for every
/// new flux scheme.
/// @tparam MapType Transform from grid to world space
/// @tparam scheme Mean flux numerical scheme
template <class MapType, MeanFluxScheme sceheme>
struct MeanFluxOp;

template <class MapType>
struct MeanFluxOp<MapType, MeanFluxScheme::NEIGHBOR_26> {
    template <class Accessor>
    [[nodiscrad]] static auto averageFlux(const MapType &mapType, const Accessor &gAcc,
                                          const openvdb::Coord &ijk);
};

template <class MapType>
struct MeanFluxOp<MapType, MeanFluxScheme::NEIGHBOR_98> {
    template <class Accessor>
    [[nodiscrad]] static auto averageFlux(const MapType &mapType, const Accessor &gAcc,
                                          const openvdb::Coord &ijk);
};

using namespace openvdb::math;
template <class MapType, MeanFluxScheme fluxScheme>
struct MeanFluxCalculator {
    template <class Accessor>
    static auto compute(const MapType &map, const Accessor &grid, const openvdb::Coord &ijk) {
        return MeanFluxOp<MapType, fluxScheme>::averageFlux(map, grid, ijk);
    }

    template <class Accessor>
    static auto result(const MapType &map, const Accessor &grid, const openvdb::Coord &ijk) {
        return compute(map, grid, ijk);
    }
};

template <class GridT, MeanFluxScheme scheme = MeanFluxScheme::NEIGHBOR_26,
          class OutSacalarGridT = typename openvdb::tools::VectorToScalarConverter<GridT>::Type,
          class MaskGridType = typename openvdb::tools::gridop::ToMaskGrid<GridT>::Type,
          class InterruptT = openvdb::util::NullInterrupter>
class MeanFluxProcessor {
   public:
    using InGridType = GridT;
    using OutGridType = OutSacalarGridT;

    MeanFluxProcessor(const GridT &grid, InterruptT *interrupt = nullptr)
        : m_InputGrid(grid), m_Interrupt(interrupt), m_Mask(nullptr) {}

    MeanFluxProcessor(const GridT &grid, const MaskGridType &mask, InterruptT *interrupt = nullptr)
        : m_InputGrid(grid), m_Interrupt(interrupt), m_Mask(&mask) {}

    [[nodiscard]] typename OutSacalarGridT::Ptr process(bool threaded = true) {
        typename OutSacalarGridT::Ptr m_OutputGrid;
        auto op = [this, &m_OutputGrid, threaded]<typename MapT>(const MapT &map) {
            using OpT = MeanFluxCalculator<MapT, scheme>;
            openvdb::tools::gridop::GridOperator<GridT, MaskGridType, OutSacalarGridT, MapT, OpT,
                                                 InterruptT>
                op(m_InputGrid, m_Mask, map, m_Interrupt);
            m_OutputGrid = op.process(threaded);  // cache the result
        };

        processTypedMap(m_InputGrid.transform(), op);
        return m_OutputGrid;
    }

   protected:
    struct Functor {
        Functor(const GridT &grid, const MaskGridType *mask, bool threaded, InterruptT *interrupt)
            : m_Threaded(threaded), m_InputGrid(grid), m_Interrupt(interrupt), m_Mask(mask) {}

        template <class MapT>
        void operator()(const MapT &map) {
            using OpT = MeanFluxCalculator<MapT, MeanFluxScheme::NEIGHBOR_26>;
            openvdb::tools::gridop::GridOperator<GridT, MaskGridType, OutSacalarGridT, MapT, OpT,
                                                 InterruptT>
                op(m_InputGrid, m_Mask, map, m_Interrupt);
            m_OutputGrid = op.process(m_Threaded);  // cache the result
        }

        const bool m_Threaded;
        const GridT &m_InputGrid;
        typename OutSacalarGridT::Ptr m_OutputGrid;
        InterruptT *m_Interrupt;
        const MaskGridType *m_Mask;
    };  // Private Functor

    const GridT &m_InputGrid;
    InterruptT *m_Interrupt;
    const MaskGridType *m_Mask;
};

/// @brief Calculates the average flux given boundary normals given a grid with a vector field
template <typename MapType, typename Accessor, typename ArrayT>
auto averageFlux(const MapType &mapType, const Accessor &gAcc, const Coord &ijk,
                 const ArrayT &boundaryNeighborNormals);

/// FUNTION DEFINITIONS:
/// @brief Helper function definitions
namespace {
template <openvdb::Coord::Int32 halfSize>
constexpr size_t numBoundaryPoints() {
    if constexpr (halfSize == 0) {
        return 0;
    } else {
        constexpr size_t numPointsPerSide = 2 * halfSize + 1;
        return numPointsPerSide * numPointsPerSide * numPointsPerSide - 1 -
               numBoundaryPoints<halfSize - 1>();
    }
}

/// @brief Computes the non unit neighbor normals relative to a center coordinate
/// @tparam halfSize Half-size of a side length of the cubic domain
/// @return
template <openvdb::Coord::Int32 halfSize>
[[nodiscard]] constexpr std::array<openvdb::Coord, numBoundaryPoints<halfSize>()>
boundaryNeighorNormals() {
    using openvdb::Coord;
    size_t arrayIdx = 0;
    std::array<Coord, numBoundaryPoints<halfSize>()> result;
    for (Coord::Int32 ii = -halfSize; ii <= halfSize; ++ii) {
        for (Coord::Int32 jj = -halfSize; jj <= halfSize; ++jj) {
            for (Coord::Int32 kk = -halfSize; kk <= halfSize; ++kk) {
                if ((abs(ii) == halfSize || abs(jj) == halfSize || abs(kk) == halfSize)) {
                    result[arrayIdx++] = Coord(ii, jj, kk);
                }
            }
        }
    }

    return result;
}
}  // namespace

template <class MapType>
template <class Accessor>
auto MeanFluxOp<MapType, MeanFluxScheme::NEIGHBOR_26>::averageFlux(const MapType &mapType,
                                                                   const Accessor &gAcc,
                                                                   const openvdb::Coord &ijk) {
    using ValueType = typename Accessor::ValueType::ValueType;
    auto boundaryNeighborNormals = boundaryNeighorNormals<1>();
    return Morphology::averageFlux(mapType, gAcc, ijk, boundaryNeighborNormals);
}

template <class MapType>
template <class Accessor>
auto MeanFluxOp<MapType, MeanFluxScheme::NEIGHBOR_98>::averageFlux(const MapType &mapType,
                                                                   const Accessor &gAcc,
                                                                   const openvdb::Coord &ijk) {
    using ValueType = typename Accessor::ValueType::ValueType;
    auto boundaryNeighborNormals = boundaryNeighorNormals<2>();
    return Morphology::averageFlux(mapType, gAcc, ijk, boundaryNeighborNormals);
}
}  // namespace Morphology

template <typename MapType, typename Accessor, typename ArrayT>
auto Morphology::averageFlux(const MapType &mapType, const Accessor &gAcc,
                             const openvdb::Coord &ijk, const ArrayT &boundaryNeighborNormals) {
    using ValueType = typename Accessor::ValueType::ValueType;
    using openvdb::Coord;
    const auto fluxes = boundaryNeighborNormals |
                        std::views::transform([mapType, gAcc, ijk](const Coord &nonUnitNormal) {
                            return mapType.applyIJT(nonUnitNormal.asVec3d().unit())
                                .dot(gAcc.getValue(ijk + Coord(nonUnitNormal[0], nonUnitNormal[1],
                                                               nonUnitNormal[2])));
                        });

    return std::accumulate(fluxes.begin(), fluxes.end(), ValueType(0.)) /
           boundaryNeighborNormals.size();
}
}  // namespace VdbFields
