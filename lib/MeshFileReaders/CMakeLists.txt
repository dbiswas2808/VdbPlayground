cmake_minimum_required(VERSION 3.12.0)

add_library(MeshFileReaders
MeshFileReaders/StlFileReader.h
MeshFileReaders/StlFileReader.cpp
)

set_target_properties ( MeshFileReaders PROPERTIES LINKER_LANGUAGE CXX )
set_property ( TARGET MeshFileReaders PROPERTY CXX_STANDARD 20 )
set_property ( TARGET MeshFileReaders PROPERTY CXX_STANDARD_REQUIRED ON )

target_link_libraries(MeshFileReaders
PRIVATE
assimp::assimp
)

target_include_directories(MeshFileReaders PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
