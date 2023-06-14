cmake_minimum_required(VERSION 3.12.0)

add_library(SkeletonGenerators
SkeletonGenerators/GrassfireDivergenceSkeleton.h
SkeletonGenerators/GrassfireDivergenceSkeleton.cpp
)

set(LINKER_LANG CXX)
set(CMAKE_PROJECT_SkeletonGenerators_LINKER_LANG CXX)
# cpp CXX standard
# set_target_properties(VDBFields PROPERTIES LINKER_LANGUAGE CXX)
set_property(TARGET SkeletonGenerators PROPERTY CXX_STANDARD 20)
set_property(TARGET SkeletonGenerators PROPERTY CXX_STANDARD_REQUIRED ON)

set_target_properties(SkeletonGenerators PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(SkeletonGenerators
PRIVATE
#third-party
openvdb_static
#lib
Core
morphology
)

target_include_directories(SkeletonGenerators PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/")