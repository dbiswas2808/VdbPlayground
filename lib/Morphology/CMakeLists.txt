cmake_minimum_required ( VERSION 3.12.0 )

add_library ( Morphology
    Morphology/MeanFlux.h
)
set_target_properties ( Morphology PROPERTIES LINKER_LANGUAGE CXX )
set_property ( TARGET Morphology PROPERTY CXX_STANDARD 20 )
set_property ( TARGET Morphology PROPERTY CXX_STANDARD_REQUIRED ON )
target_link_libraries ( Morphology
    PRIVATE
    openvdb_static
    PUBLIC
    LevelSetOperators
)
target_include_directories ( Morphology PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/" )
