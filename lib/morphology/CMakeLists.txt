cmake_minimum_required(VERSION 3.12.0)

add_library(morphology
    morphology/mean_flux.h
)
set_target_properties(morphology PROPERTIES LINKER_LANGUAGE CXX)
set_property(TARGET morphology PROPERTY CXX_STANDARD 20)
set_property(TARGET morphology PROPERTY CXX_STANDARD_REQUIRED ON)
target_link_libraries(morphology
    PRIVATE
       openvdb_static
)
target_include_directories(morphology PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/")