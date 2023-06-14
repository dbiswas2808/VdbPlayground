cmake_minimum_required(VERSION 3.12.0)

add_library(UI 
    UI/user_interface.h
    UI/ui_structure.h
    UI/user_interface.cpp
    UI/ui_structure.cpp
)

target_link_libraries(UI
PRIVATE
polyscope
SkeletonGenerators
Core
)

target_include_directories(UI PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/")