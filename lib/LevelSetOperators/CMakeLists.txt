cmake_minimum_required ( VERSION 3.12.0 )

add_library ( LevelSetOperators
LevelSetOperators/MeanFluxOp.h
)

set_target_properties ( LevelSetOperators PROPERTIES LINKER_LANGUAGE CXX )
set_property ( TARGET LevelSetOperators PROPERTY CXX_STANDARD 20 )
set_property ( TARGET LevelSetOperators PROPERTY CXX_STANDARD_REQUIRED ON )

target_link_libraries ( LevelSetOperators
    PRIVATE
    openvdb_static
)
target_include_directories ( LevelSetOperators PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/" )