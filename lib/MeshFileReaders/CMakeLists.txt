cmake_minimum_required(VERSION 3.12.0)

add_library(MeshFileReaders
MeshFileReaders/StlFileReader.h
MeshFileReaders/StlFileReader.cpp
)

set_property ( TARGET MeshFileReaders PROPERTY CXX_STANDARD 20 )
target_link_libraries(MeshFileReaders
PRIVATE
assimp::assimp
)

target_include_directories(MeshFileReaders PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
