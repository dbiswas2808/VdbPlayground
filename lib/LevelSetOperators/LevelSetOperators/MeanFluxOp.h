#pragma once
#include<ranges>
#include <openvdb/math/Coord.h>
#include <openvdb/openvdb.h>
#include <openvdb/tools/GridOperators.h>

namespace VdbFields {
namespace LevelSetOperators {
/// @brief Flux numerical scheme types implemented by MeanFluxOp
enum class MeanFluxScheme { neighbor26, neighbor98 };

/// @brief Declaration for MeanFluxOp template. Should be specialized for every
/// new flux scheme.
/// @tparam MapType Transform from grid to world space
/// @tparam scheme Mean flux numerical scheme
template <class MapType, MeanFluxScheme sceheme>
struct MeanFluxOp;

/// @brief Calculates the average flux given boundary normals given a grid with a vector field using
/// a neigborhood of 26
template <class MapType>
struct MeanFluxOp<MapType, MeanFluxScheme::neighbor26> {
    template <class Accessor>
    [[nodiscard]] static auto averageFlux(const MapType &mapType, const Accessor &gAcc,
                                          const openvdb::math::Coord &ijk);
};

/// @brief Calculates the average flux given boundary normals given a grid with a vector field using
/// a neigborhood of 98
template <class MapType>
struct MeanFluxOp<MapType, MeanFluxScheme::neighbor98> {
    template <class Accessor>
    [[nodiscard]] static auto averageFlux(const MapType &mapType, const Accessor &gAcc,
                                          const openvdb::math::Coord &ijk);
};


/// @brief Implementation helper types/free-functions
namespace internal {
/// @brief Calculates the average flux given boundary normals given a grid with a vector field
template <typename MapType, typename Accessor, typename ArrayT>
auto averageFlux(const MapType &mapType, const Accessor &gAcc, const openvdb::math::Coord &ijk,
                 const ArrayT &boundaryNeighborNormals);

/// @brief Computes the number of boundary points for a given half-size of the cell
template <openvdb::math::Coord::Int32 halfSize>
constexpr size_t numCellBoundaryPoints() {
    if constexpr (halfSize == 0) {
        return 0;
    } else {
        constexpr size_t numPointsPerSide = 2 * halfSize + 1;
        return numPointsPerSide * numPointsPerSide * numPointsPerSide - 1 -
               numCellBoundaryPoints<halfSize - 1>();
    }
}

/// @brief Computes the non unit neighbor normals relative to a center coordinate
/// @tparam halfSize Half-size of a side length of the cubic domain
/// @return
template <openvdb::math::Coord::Int32 halfSize>
[[nodiscard]] constexpr std::array<openvdb::math::Coord, numCellBoundaryPoints<halfSize>()>
boundaryNeighorNormals();
}  // namespace internal
}  // namespace LevelSetOperators

// IMPLEMENTATION: MeanFluxOp
template <class MapType>
template <class Accessor>
auto LevelSetOperators::MeanFluxOp<MapType, LevelSetOperators::MeanFluxScheme::neighbor26>::
    averageFlux(const MapType &mapType, const Accessor &gAcc, const openvdb::math::Coord &ijk) {
    using ValueType = typename Accessor::ValueType::ValueType;
    auto boundaryNeighborNormals = internal::boundaryNeighorNormals<1>();
    return internal::averageFlux(mapType, gAcc, ijk, boundaryNeighborNormals);
}

template <class MapType>
template <class Accessor>
auto LevelSetOperators::MeanFluxOp<MapType, LevelSetOperators::MeanFluxScheme::neighbor98>::
    averageFlux(const MapType &mapType, const Accessor &gAcc, const openvdb::math::Coord &ijk) {
    using ValueType = typename Accessor::ValueType::ValueType;
    auto boundaryNeighborNormals = internal::boundaryNeighorNormals<2>();
    return internal::averageFlux(mapType, gAcc, ijk, boundaryNeighborNormals);
}

/// @brief Computes the non unit neighbor normals relative to a center coordinate
/// @tparam halfSize Half-size of a side length of the cubic domain
/// @return
template <openvdb::math::Coord::Int32 halfSize>
[[nodiscard]] constexpr std::array<openvdb::math::Coord,
                                   LevelSetOperators::internal::numCellBoundaryPoints<halfSize>()>
LevelSetOperators::internal::boundaryNeighorNormals() {
    using openvdb::math::Coord;
    size_t arrayIdx = 0;
    std::array<Coord, numCellBoundaryPoints<halfSize>()> result;
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

template <typename MapType, typename Accessor, typename ArrayT>
auto LevelSetOperators::internal::averageFlux(const MapType &mapType, const Accessor &gAcc,
                                              const openvdb::math::Coord &ijk,
                                              const ArrayT &boundaryNeighborNormals) {
    using ValueType = typename Accessor::ValueType::ValueType;
    using openvdb::math::Coord;
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