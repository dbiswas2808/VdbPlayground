cmake_minimum_required ( VERSION 3.0.0 )

set ( Tbb_VERSION ${TBB_VERSION} )
set ( USE_BLOSC OFF )
set ( USE_JEMALLOC ON )
set ( OPENVDB_USE_VERSION_NAMESPACE ON )
set ( OPENVDB_CORE_STATIC ON )
set ( OPENVDB_CORE_SHARED OFF )
add_subdirectory ( openvdb )
add_subdirectory ( polyscope )