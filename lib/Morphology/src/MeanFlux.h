#include <openvdb/openvdb.h>
#include <openvdb/tools/GridOperators.h>
#include <openvdb/math/Coord.h>
#include <ranges>


namespace Morphology {
using namespace openvdb;

enum class MeanFluxScheme {
    NEIGHBOR_26,
};

template <typename MapType, MeanFluxScheme fluxOp>
struct MeanFluxOp;

template <typename MapType>
struct MeanFluxOp<MapType, MeanFluxScheme::NEIGHBOR_26> {
    template <typename Accessor>
    static auto averageFlux(const MapType& mapType, const Accessor& gAcc, const Coord& ijk) {
        using ValueType = typename Accessor::ValueType::ValueType;
        
        auto neighbor26 = std::array<Coord, 26>{
            // cells connected by faces (6 cells)
            Coord{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1},

            // cells connected by edges (12 cells)
            {1, 1, 0}, {-1, -1, 0}, {-1, 1, 0}, {1, -1, 0},
            {1, 0, 1}, {-1, 0, -1}, {-1, 0, 1}, {1, 0, -1},
            {0, 1, 1}, {0, -1, -1}, {0, -1, 1}, {0, 1, -1},

            // cells connected by corners (8 cells)
            {1, 1, 1}, {-1, 1, 1}, {1, -1, 1}, {-1, -1, 1},
            {1, 1, -1}, {1, -1, -1}, {-1, 1, -1}, {-1, -1, -1}
        };

        const auto fluxes = neighbor26 | std::views::transform([mapType, gAcc, ijk](const Coord& pt) {
            return mapType.applyIJT(pt.asVec3d().unit()).dot(gAcc.getValue(ijk + pt));
        });

        return std::accumulate(fluxes.begin(), fluxes.end(), ValueType(0.)) / fluxes.size();
    }
};

using namespace openvdb::math;
template <typename MapType, MeanFluxScheme fluxScheme>
struct MeanFluxCalculator {

    template <typename Accessor>
    static auto compute(const MapType& map,
                        const Accessor& grid,
                        const Coord& ijk) {
        return MeanFluxOp<MapType, MeanFluxScheme::NEIGHBOR_26>::averageFlux(map, grid, ijk);
    }

    template <typename Accessor>
    static auto result(const MapType& map, const Accessor& grid, const Coord& ijk) {
        return compute(map, grid, ijk);
    }
};

template <class GridT,
          class OutSacalarGridT = typename tools::VectorToScalarConverter<GridT>::Type,
          class MaskGridType = typename tools::gridop::ToMaskGrid<GridT>::Type,
          class InterruptT = openvdb::util::NullInterrupter>
class MeanFluxProcessor {
public:
    using InGridType = GridT;
    using OutGridType = OutSacalarGridT;

    MeanFluxProcessor(const GridT& grid, InterruptT* interrupt = nullptr)
        : m_InputGrid(grid)
        , m_Interrupt(interrupt)
        , m_Mask(nullptr) {}

    MeanFluxProcessor(const GridT& grid, const MaskGridType& mask, InterruptT* interrupt = nullptr)
        : m_InputGrid(grid)
        , m_Interrupt(interrupt)
        , m_Mask(&mask) {}

    typename OutSacalarGridT::Ptr process(bool threaded = true) {
        typename OutSacalarGridT::Ptr m_OutputGrid;
        auto op = [this, &m_OutputGrid, threaded] <typename MapT> (const MapT& map) {
            using OpT = MeanFluxCalculator<MapT, MeanFluxScheme::NEIGHBOR_26>;
            openvdb::tools::gridop::GridOperator<GridT, MaskGridType, OutSacalarGridT, MapT, OpT, InterruptT> op(
                m_InputGrid, m_Mask, map, m_Interrupt);
            m_OutputGrid = op.process(threaded); // cache the result
        };

        processTypedMap(m_InputGrid.transform(), op);
        return m_OutputGrid;
    }

protected:
    struct Functor {
        Functor(const GridT& grid, const MaskGridType* mask, bool threaded, InterruptT* interrupt):
              m_Threaded(threaded), m_InputGrid(grid), m_Interrupt(interrupt), m_Mask(mask) {}

        template <class MapT>
        void operator()(const MapT& map) {
            using OpT = MeanFluxCalculator<MapT, MeanFluxScheme::NEIGHBOR_26>;
            openvdb::tools::gridop::GridOperator<GridT, MaskGridType, OutSacalarGridT, MapT, OpT, InterruptT> op(
                m_InputGrid, m_Mask, map, m_Interrupt);
            m_OutputGrid = op.process(m_Threaded); // cache the result
        }

        const bool           m_Threaded;
        const GridT&         m_InputGrid;
        typename OutSacalarGridT::Ptr m_OutputGrid;
        InterruptT* m_Interrupt;
        const MaskGridType*  m_Mask;
    }; // Private Functor

    const GridT&        m_InputGrid;
    InterruptT*         m_Interrupt;
    const MaskGridType* m_Mask;
};
}

void fn() {
    openvdb::Vec3DGrid grid;
    auto avgFluxProcessor = Morphology::MeanFluxProcessor{grid};
    avgFluxProcessor.process();
}