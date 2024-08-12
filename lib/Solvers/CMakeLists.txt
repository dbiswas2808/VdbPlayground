cmake_minimum_required(VERSION 3.15)

add_library(Solvers
Solvers/Multigrid.h
Solvers/Multigrid.cpp
)

set_target_properties ( Solvers PROPERTIES LINKER_LANGUAGE CXX )
set_property ( TARGET Solvers PROPERTY CXX_STANDARD 20 )
set_property ( TARGET Solvers PROPERTY CXX_STANDARD_REQUIRED ON )

target_link_libraries(
    Solvers
    PUBLIC
    Eigen3::Eigen
)

add_compile_definitions(_GLIBCXX_USE_CXX11_ABI=0)
find_package(Catch2 3 REQUIRED)

target_include_directories(Solvers PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/")

add_executable(SolversTest Solvers/Multigrid.t.cpp)
set_target_properties(SolversTest PROPERTIES LINKER_LANGUAGE CXX)
set_property(TARGET SolversTest PROPERTY CXX_STANDARD 20)
set_property(TARGET SolversTest PROPERTY CXX_STANDARD_REQUIRED ON)

target_link_libraries(SolversTest PRIVATE Catch2::Catch2WithMain Solvers)
