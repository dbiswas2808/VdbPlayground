#pragma once
#include <openvdb/math/Coord.h>
#include <openvdb/openvdb.h>
#include <openvdb/tools/GridOperators.h>
#include <openvdb/tools/MeshToVolume.h>
#include <LevelSetOperators/MeanFluxOp.h>
#include <fstream>

#include <ranges>

namespace VdbFields::Morphology {
template <class MapType, LevelSetOperators::MeanFluxScheme fluxScheme>
struct MeanFluxCalculator {
    template <class Accessor>
    static auto compute(const MapType &map, const Accessor &grid, const openvdb::math::Coord &ijk) {
        return LevelSetOperators::MeanFluxOp<MapType, fluxScheme>::averageFlux(map, grid, ijk);
    }

    template <class Accessor>
    static auto result(const MapType &map, const Accessor &grid, const openvdb::math::Coord &ijk) {
        return compute(map, grid, ijk);
    }
};

template <class GridT, LevelSetOperators::MeanFluxScheme scheme = LevelSetOperators::MeanFluxScheme::neighbor26,
          class OutSacalarGridT = typename openvdb::tools::VectorToScalarConverter<GridT>::Type,
          class MaskGridType = typename openvdb::tools::gridop::ToMaskGrid<GridT>::Type,
          class InterruptT = openvdb::util::NullInterrupter>
class MeanFluxProcessor {
   public:
    using InGridType = GridT;
    using OutGridType = OutSacalarGridT;

    MeanFluxProcessor(const GridT &grid, InterruptT *interrupt = nullptr)
        : m_InputGrid(grid), m_Interrupt(interrupt), m_Mask(nullptr) {}

    MeanFluxProcessor(const GridT &grid, const MaskGridType &mask, InterruptT *interrupt)
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
            using OpT = MeanFluxCalculator<MapT, scheme>;
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
}  // namespace VdbFields::Morphology
