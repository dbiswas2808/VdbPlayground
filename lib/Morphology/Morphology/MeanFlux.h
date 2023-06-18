#include <openvdb/math/Coord.h>
#include <openvdb/openvdb.h>
#include <openvdb/tools/GridOperators.h>
#include <openvdb/tools/MeshToVolume.h>

#include <ranges>

namespace VdbFields::Morphology {
using namespace openvdb;

/// @brief Flux numerical scheme type. MeanFluxOp is specialized on these values
/// for concrete implementations.
enum class MeanFluxScheme {
    NEIGHBOR_26
};

/// @brief Declaration for MeanFluxOp template. Should be specialized for every
/// new flux scheme.
/// @tparam MapType Transform from grid to world space
/// @tparam scheme Mean flux numerical scheme
template <class MapType, MeanFluxScheme sceheme>
struct MeanFluxOp;

template <class MapType>
struct MeanFluxOp<MapType, MeanFluxScheme::NEIGHBOR_26> {
    template <class Accessor>
    [[nodiscrad]] static auto averageFlux(const MapType &mapType, const Accessor &gAcc, const Coord &ijk);
};

using namespace openvdb::math;
template <class MapType, MeanFluxScheme fluxScheme>
struct MeanFluxCalculator {
    template <class Accessor>
    static auto compute(const MapType &map, const Accessor &grid, const Coord &ijk) {
        return MeanFluxOp<MapType, fluxScheme>::averageFlux(map, grid, ijk);
    }

    template <class Accessor>
    static auto result(const MapType &map, const Accessor &grid, const Coord &ijk) {
        return compute(map, grid, ijk);
    }
};

template <class GridT, MeanFluxScheme scheme = MeanFluxScheme::NEIGHBOR_26,
          class OutSacalarGridT = typename tools::VectorToScalarConverter<GridT>::Type,
          class MaskGridType = typename tools::gridop::ToMaskGrid<GridT>::Type,
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

namespace {
template <Coord::Int32 halfSize>
constexpr size_t numNeigborPoints() {
    constexpr size_t numPointsPerSide = 2 * halfSize + 1;

    return numPointsPerSide * numPointsPerSide * numPointsPerSide - 1;
}

/// @brief Computes the non unit neighbor normals relative to a center coordinate
/// @tparam halfSize Half-size of a side length of the cubic domain
/// @return 
template <Coord::Int32 halfSize>
[[nodiscard]] constexpr std::array<Coord, numNeigborPoints<halfSize>()> neighborNormals() {
    size_t arrayIdx = 0;
    std::array<Coord, numNeigborPoints<halfSize>()> result;
    for (Coord::Int32 ii = -halfSize; ii <= halfSize; ++ii) {
        for (Coord::Int32 jj = -halfSize; jj <= halfSize; ++jj) {
            for (Coord::Int32 kk = -halfSize; kk <= halfSize; ++kk) {
                if (not(ii == 0 && jj == 0 && kk == 0)) {
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
                                                                   const Coord &ijk) {
    using ValueType = typename Accessor::ValueType::ValueType;
    auto neighbor26Normals = neighborNormals<1>();

    const auto fluxes =
        neighbor26Normals | std::views::transform([mapType, gAcc, ijk](const Coord &nonUnitNormal) {
            return mapType.applyIJT(nonUnitNormal.asVec3d().unit())
                .dot(gAcc.getValue(
                    ijk + Coord(2 * nonUnitNormal[0], 2 * nonUnitNormal[1], 2 * nonUnitNormal[2])));
        });

    return std::accumulate(fluxes.begin(), fluxes.end(), ValueType(0.)) / fluxes.size();
}
}  // namespace VdbFields::Morphology
