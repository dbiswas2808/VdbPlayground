cmake_minimum_required(VERSION 3.12.0)
project(VDBFields VERSION 0.1.0)

include(CTest)
enable_testing()

include(./conanbuildinfo.cmake)
conan_basic_setup()
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/")
list(APPEND CMAKE_MODULE_PATH "/usr/local/lib/cmake/OpenVDB")

# Find all conan packages
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(jemalloc REQUIRED)
find_package(TBB REQUIRED)
# find_package(OpenVDB REQUIRED)

# Define main target app VDBFields
add_executable(VDBFields main.cpp)

set(LINKER_LANG CXX)
set(CMAKE_PROJECT_VDBFields_LINKER_LANG CXX)
# cpp CXX standard
# set_target_properties(VDBFields PROPERTIES LINKER_LANGUAGE CXX)
set_property(TARGET VDBFields PROPERTY CXX_STANDARD 20)
set_property(TARGET VDBFields PROPERTY CXX_STANDARD_REQUIRED ON)

# Link the conan libraries to your executable
# Link submodules and other third party libs
add_subdirectory(vendor)
add_subdirectory(lib)
add_subdirectory(application)
target_link_libraries(VDBFields 
    PUBLIC 
    Eigen3::Eigen 
    Boost::boost 
    TBB::tbb 
    openvdb_static
    polyscope 
    # application 
    morphology 
    Core
    UI
    )


set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
