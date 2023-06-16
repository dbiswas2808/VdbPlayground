cmake_minimum_required(VERSION 3.12.0)

add_library(UI 
    UI/UserInterface.h
    UI/UiStructure.h
    UI/UserInterface.cpp
    UI/UiStructure.cpp
)

target_link_libraries(UI
PRIVATE
polyscope
SkeletonGenerators
Core
)

target_include_directories(UI PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/")
