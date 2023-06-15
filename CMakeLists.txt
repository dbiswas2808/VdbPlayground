cmake_minimum_required(VERSION 3.12.0)
project(VDBFields VERSION 0.1.0)

include(CTest)
enable_testing()

include(./conanbuildinfo.cmake)
conan_basic_setup()
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/")

find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
    set(CMAKE_C_COMPILER_LAUNCHER ${CCACHE})
    set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE})
endif(CCACHE_FOUND)

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
    UI
)


set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
