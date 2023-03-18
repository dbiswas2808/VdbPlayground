cmake_minimum_required(VERSION 3.12.0)

add_library(morphology
    src/PrincipalCurvature.h
)
set_target_properties(morphology PROPERTIES LINKER_LANGUAGE CXX)

target_include_directories(morphology PUBLIC src)
target_link_libraries(morphology
    PUBLIC
        openvdb
)