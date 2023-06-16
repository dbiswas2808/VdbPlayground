#include <ranges>

#include <openvdb/math/Coord.h>
#include <openvdb/openvdb.h>
#include <openvdb/tools/GridOperators.h>
#include <openvdb/tools/MeshToVolume.h>

namespace VdbFields::Morphology {
using namespace openvdb;

/// @brief Flux numerical scheme type. MeanFluxOp is specialized on these values
/// for concrete implementations.
enum class MeanFluxScheme {
  NEIGHBOR_26,
};

/// @brief Declaration for MeanFluxOp template. Should be specialized for every
/// new flux scheme.
/// @tparam MapType Transform from grid to world space
/// @tparam scheme Mean flux numerical scheme
template <typename MapType, MeanFluxScheme sceheme> struct MeanFluxOp;

template <typename MapType>
struct MeanFluxOp<MapType, MeanFluxScheme::NEIGHBOR_26> {
  template <typename Accessor>
  static auto averageFlux(const MapType &mapType, const Accessor &gAcc,
                          const Coord &ijk) {
    using ValueType = typename Accessor::ValueType::ValueType;

    auto neighbor26Normals =
        std::array<Coord, 26>{// cells connected by faces (6 cells)
                              Coord{1, 0, 0},
                              {-1, 0, 0},
                              {0, 1, 0},
                              {0, -1, 0},
                              {0, 0, 1},
                              {0, 0, -1},

                              // cells connected by edges (12 cells)
                              {1, 1, 0},
                              {-1, -1, 0},
                              {-1, 1, 0},
                              {1, -1, 0},
                              {1, 0, 1},
                              {-1, 0, -1},
                              {-1, 0, 1},
                              {1, 0, -1},
                              {0, 1, 1},
                              {0, -1, -1},
                              {0, -1, 1},
                              {0, 1, -1},

                              // cells connected by corners (8 cells)
                              {1, 1, 1},
                              {-1, 1, 1},
                              {1, -1, 1},
                              {-1, -1, 1},
                              {1, 1, -1},
                              {1, -1, -1},
                              {-1, 1, -1},
                              {-1, -1, -1}};

    const auto fluxes =
        neighbor26Normals |
        std::views::transform([mapType, gAcc, ijk](const Coord &nonUnitNormal) {
          return mapType.applyIJT(nonUnitNormal.asVec3d().unit())
              .dot(gAcc.getValue(ijk + nonUnitNormal));
        });

    return std::accumulate(fluxes.begin(), fluxes.end(), ValueType(0.)) /
           fluxes.size();
  }
};

using namespace openvdb::math;
template <typename MapType, MeanFluxScheme fluxScheme>
struct MeanFluxCalculator {

  template <typename Accessor>
  static auto compute(const MapType &map, const Accessor &grid,
                      const Coord &ijk) {
    return MeanFluxOp<MapType, fluxScheme>::averageFlux(map, grid, ijk);
  }

  template <typename Accessor>
  static auto result(const MapType &map, const Accessor &grid,
                     const Coord &ijk) {
    return compute(map, grid, ijk);
  }
};

template <class GridT, MeanFluxScheme scheme = MeanFluxScheme::NEIGHBOR_26,
          class OutSacalarGridT =
              typename tools::VectorToScalarConverter<GridT>::Type,
          class MaskGridType = typename tools::gridop::ToMaskGrid<GridT>::Type,
          class InterruptT = openvdb::util::NullInterrupter>
class MeanFluxProcessor {
public:
  using InGridType = GridT;
  using OutGridType = OutSacalarGridT;

  MeanFluxProcessor(const GridT &grid, InterruptT *interrupt = nullptr)
      : m_InputGrid(grid), m_Interrupt(interrupt), m_Mask(nullptr) {}

  MeanFluxProcessor(const GridT &grid, const MaskGridType &mask,
                    InterruptT *interrupt = nullptr)
      : m_InputGrid(grid), m_Interrupt(interrupt), m_Mask(&mask) {}

  typename OutSacalarGridT::Ptr process(bool threaded = true) {
    typename OutSacalarGridT::Ptr m_OutputGrid;
    auto op = [this, &m_OutputGrid, threaded]<typename MapT>(const MapT &map) {
      using OpT = MeanFluxCalculator<MapT, scheme>;
      openvdb::tools::gridop::GridOperator<GridT, MaskGridType, OutSacalarGridT,
                                           MapT, OpT, InterruptT>
          op(m_InputGrid, m_Mask, map, m_Interrupt);
      m_OutputGrid = op.process(threaded); // cache the result
    };

    processTypedMap(m_InputGrid.transform(), op);
    return m_OutputGrid;
  }

protected:
  struct Functor {
    Functor(const GridT &grid, const MaskGridType *mask, bool threaded,
            InterruptT *interrupt)
        : m_Threaded(threaded), m_InputGrid(grid), m_Interrupt(interrupt),
          m_Mask(mask) {}

    template <class MapT> void operator()(const MapT &map) {
      using OpT = MeanFluxCalculator<MapT, MeanFluxScheme::NEIGHBOR_26>;
      openvdb::tools::gridop::GridOperator<GridT, MaskGridType, OutSacalarGridT,
                                           MapT, OpT, InterruptT>
          op(m_InputGrid, m_Mask, map, m_Interrupt);
      m_OutputGrid = op.process(m_Threaded); // cache the result
    }

    const bool m_Threaded;
    const GridT &m_InputGrid;
    typename OutSacalarGridT::Ptr m_OutputGrid;
    InterruptT *m_Interrupt;
    const MaskGridType *m_Mask;
  }; // Private Functor

  const GridT &m_InputGrid;
  InterruptT *m_Interrupt;
  const MaskGridType *m_Mask;
};

template <typename GridType>
typename GridType::Ptr createLevelSetTestCases(int faceCount, float scale,
                                               const openvdb::Vec3f &center,
                                               float voxelSize,
                                               float halfWidth) {
  using namespace openvdb;
  // GridType::ValueType is required to be a floating-point scalar.
  static_assert(std::is_floating_point<typename GridType::ValueType>::value,
                "level set grids must have scalar, floating-point value types");

  const math::Transform::Ptr xform =
      math::Transform::createLinearTransform(voxelSize);

  std::vector<Vec3f> vtx;
  std::vector<Vec3I> tri;
  std::vector<Vec4I> qua;

  if (faceCount == 4) { // Tetrahedron

    vtx.push_back(Vec3f(0.0f, 1.0f, 0.0f));
    vtx.push_back(Vec3f(-0.942810297f, -0.333329707f, 0.0f));
    vtx.push_back(Vec3f(0.471405149f, -0.333329707f, 0.816497624f));
    vtx.push_back(Vec3f(0.471405149f, -0.333329707f, -0.816497624f));

    tri.push_back(Vec3I(0, 2, 3));
    tri.push_back(Vec3I(0, 3, 1));
    tri.push_back(Vec3I(0, 1, 2));
    tri.push_back(Vec3I(1, 3, 2));

  } else if (faceCount == 6) { // Cube

    vtx.push_back(Vec3f(-0.5f, -0.5f, -0.5f));
    vtx.push_back(Vec3f(0.5f, -0.5f, -0.5f));
    vtx.push_back(Vec3f(0.5f, -0.5f, 0.5f));
    vtx.push_back(Vec3f(-0.5f, -0.5f, 0.5f));
    vtx.push_back(Vec3f(-0.5f, 0.5f, -0.5f));
    vtx.push_back(Vec3f(0.5f, 0.5f, -0.5f));
    vtx.push_back(Vec3f(0.5f, 0.5f, 0.5f));
    vtx.push_back(Vec3f(-0.5f, 0.5f, 0.5f));

    qua.push_back(Vec4I(1, 0, 4, 5));
    qua.push_back(Vec4I(2, 1, 5, 6));
    qua.push_back(Vec4I(3, 2, 6, 7));
    qua.push_back(Vec4I(0, 3, 7, 4));
    qua.push_back(Vec4I(2, 3, 0, 1));
    qua.push_back(Vec4I(5, 4, 7, 6));

  } else if (faceCount == 8) { // Octahedron

    vtx.push_back(Vec3f(0.0f, 0.0f, -1.0f));
    vtx.push_back(Vec3f(1.0f, 0.0f, 0.0f));
    vtx.push_back(Vec3f(0.0f, 0.0f, 1.0f));
    vtx.push_back(Vec3f(-1.0f, 0.0f, 0.0f));
    vtx.push_back(Vec3f(0.0f, -1.0f, 0.0f));
    vtx.push_back(Vec3f(0.0f, 1.0f, 0.0f));

    tri.push_back(Vec3I(0, 4, 3));
    tri.push_back(Vec3I(0, 1, 4));
    tri.push_back(Vec3I(1, 2, 4));
    tri.push_back(Vec3I(2, 3, 4));
    tri.push_back(Vec3I(0, 3, 5));
    tri.push_back(Vec3I(0, 5, 1));
    tri.push_back(Vec3I(1, 5, 2));
    tri.push_back(Vec3I(2, 5, 3));

  } else if (faceCount == 12) { // Dodecahedron

    vtx.push_back(Vec3f(0.354437858f, 0.487842113f, -0.789344311f));
    vtx.push_back(Vec3f(0.573492587f, -0.186338872f, -0.78934437f));
    vtx.push_back(Vec3f(0.0f, -0.603005826f, -0.78934443f));
    vtx.push_back(Vec3f(-0.573492587f, -0.186338872f, -0.78934437f));
    vtx.push_back(Vec3f(-0.354437858f, 0.487842113f, -0.789344311f));
    vtx.push_back(Vec3f(-0.573492587f, 0.789345026f, -0.186338797f));
    vtx.push_back(Vec3f(-0.927930415f, -0.301502913f, -0.186338872f));
    vtx.push_back(Vec3f(0.0f, -0.975683928f, -0.186338902f));
    vtx.push_back(Vec3f(0.927930415f, -0.301502913f, -0.186338872f));
    vtx.push_back(Vec3f(0.573492587f, 0.789345026f, -0.186338797f));
    vtx.push_back(Vec3f(0.0f, 0.975683868f, 0.186338902f));
    vtx.push_back(Vec3f(-0.927930415f, 0.301502913f, 0.186338872f));
    vtx.push_back(Vec3f(-0.573492587f, -0.789345026f, 0.186338797f));
    vtx.push_back(Vec3f(0.573492587f, -0.789345026f, 0.186338797f));
    vtx.push_back(Vec3f(0.927930415f, 0.301502913f, 0.186338872f));
    vtx.push_back(Vec3f(0.0f, 0.603005826f, 0.78934443f));
    vtx.push_back(Vec3f(0.573492587f, 0.186338872f, 0.78934437f));
    vtx.push_back(Vec3f(0.354437858f, -0.487842113f, 0.789344311f));
    vtx.push_back(Vec3f(-0.354437858f, -0.487842113f, 0.789344311f));
    vtx.push_back(Vec3f(-0.573492587f, 0.186338872f, 0.78934437f));

    qua.push_back(Vec4I(0, 1, 2, 3));
    tri.push_back(Vec3I(0, 3, 4));
    qua.push_back(Vec4I(0, 4, 5, 10));
    tri.push_back(Vec3I(0, 10, 9));
    qua.push_back(Vec4I(0, 9, 14, 8));
    tri.push_back(Vec3I(0, 8, 1));
    qua.push_back(Vec4I(1, 8, 13, 7));
    tri.push_back(Vec3I(1, 7, 2));
    qua.push_back(Vec4I(2, 7, 12, 6));
    tri.push_back(Vec3I(2, 6, 3));
    qua.push_back(Vec4I(3, 6, 11, 5));
    tri.push_back(Vec3I(3, 5, 4));
    qua.push_back(Vec4I(5, 11, 19, 15));
    tri.push_back(Vec3I(5, 15, 10));
    qua.push_back(Vec4I(6, 12, 18, 19));
    tri.push_back(Vec3I(6, 19, 11));
    qua.push_back(Vec4I(7, 13, 17, 18));
    tri.push_back(Vec3I(7, 18, 12));
    qua.push_back(Vec4I(8, 14, 16, 17));
    tri.push_back(Vec3I(8, 17, 13));
    qua.push_back(Vec4I(9, 10, 15, 16));
    tri.push_back(Vec3I(9, 16, 14));
    qua.push_back(Vec4I(15, 19, 18, 17));
    tri.push_back(Vec3I(15, 17, 16));

  } else if (faceCount == 20) { // Icosahedron

    vtx.push_back(Vec3f(0.0f, 0.0f, -1.0f));
    vtx.push_back(Vec3f(0.0f, 0.894427359f, -0.447213143f));
    vtx.push_back(Vec3f(0.850650847f, 0.276393682f, -0.447213203f));
    vtx.push_back(Vec3f(0.525731206f, -0.723606944f, -0.447213262f));
    vtx.push_back(Vec3f(-0.525731206f, -0.723606944f, -0.447213262f));
    vtx.push_back(Vec3f(-0.850650847f, 0.276393682f, -0.447213203f));
    vtx.push_back(Vec3f(-0.525731206f, 0.723606944f, 0.447213262f));
    vtx.push_back(Vec3f(-0.850650847f, -0.276393682f, 0.447213203f));
    vtx.push_back(Vec3f(0.0f, -0.894427359f, 0.447213143f));
    vtx.push_back(Vec3f(0.850650847f, -0.276393682f, 0.447213203f));
    vtx.push_back(Vec3f(0.525731206f, 0.723606944f, 0.447213262f));
    vtx.push_back(Vec3f(0.0f, 0.0f, 1.0f));

    tri.push_back(Vec3I(2, 0, 1));
    tri.push_back(Vec3I(3, 0, 2));
    tri.push_back(Vec3I(4, 0, 3));
    tri.push_back(Vec3I(5, 0, 4));
    tri.push_back(Vec3I(1, 0, 5));
    tri.push_back(Vec3I(6, 1, 5));
    tri.push_back(Vec3I(7, 5, 4));
    tri.push_back(Vec3I(8, 4, 3));
    tri.push_back(Vec3I(9, 3, 2));
    tri.push_back(Vec3I(10, 2, 1));
    tri.push_back(Vec3I(10, 1, 6));
    tri.push_back(Vec3I(6, 5, 7));
    tri.push_back(Vec3I(7, 4, 8));
    tri.push_back(Vec3I(8, 3, 9));
    tri.push_back(Vec3I(9, 2, 10));
    tri.push_back(Vec3I(6, 11, 10));
    tri.push_back(Vec3I(10, 11, 9));
    tri.push_back(Vec3I(9, 11, 8));
    tri.push_back(Vec3I(8, 11, 7));
    tri.push_back(Vec3I(7, 11, 6));

  } else {
    OPENVDB_THROW(RuntimeError, "Invalid face count");
  }

  // Apply scale and translation to all the vertices
  for (size_t i = 0; i < vtx.size(); ++i)
    vtx[i] = scale * vtx[i] + center;

  typename GridType::Ptr grid;

  util::NullInterrupter tmp;
  grid = tools::meshToSignedDistanceField<GridType>(tmp, *xform, vtx, tri, qua,
                                                    3.f, halfWidth);

  return grid;
}
} // namespace VdbFields::Morphology
