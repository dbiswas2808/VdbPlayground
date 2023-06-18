cmake_minimum_required ( VERSION 3.12.0 )

add_library ( Core
    Core/Model.h
    Core/ModelCollection.h
    Core/ModelLoader.h
    Core/Model.cpp
    Core/ModelCollection.cpp
    Core/ModelLoader.cpp
)
set_target_properties ( Core PROPERTIES LINKER_LANGUAGE CXX )

target_link_libraries (
    Core
    PUBLIC
    Eigen3::Eigen
)

target_include_directories ( Core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/ )