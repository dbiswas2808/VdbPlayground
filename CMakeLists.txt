cmake_minimum_required ( VERSION 3.12.0 )
project ( VDBFields VERSION 0.1.0 )

include ( CTest )
enable_testing ()

# Include Conan
include ( cmake/modules/SetupConan.cmake )

# Enable ccache for caching compilation results
find_program ( CCACHE_FOUND ccache )

if ( CCACHE_FOUND )
    set ( CMAKE_C_COMPILER_LAUNCHER ${CCACHE} )
    set ( CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE} )
endif ( CCACHE_FOUND )

if ( VF_ASAN )
    set ( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fsanitize=address" )
    set ( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address" )
endif ( VF_ASAN )

find_package ( Eigen3 REQUIRED )
find_package ( assimp REQUIRED )
find_package(TBB REQUIRED)

# Define main target app VDBFields
add_executable ( VDBFields main.cpp )

set ( LINKER_LANG CXX )
set ( CMAKE_PROJECT_VDBFields_LINKER_LANG CXX )

# cpp CXX standard
set_property ( TARGET VDBFields PROPERTY CXX_STANDARD 20 )
set_property ( TARGET VDBFields PROPERTY CXX_STANDARD_REQUIRED ON )

# Link submodules and other third party libs
target_link_libraries ( VDBFields
    PUBLIC
    UI
)

add_subdirectory ( vendor )
add_subdirectory ( lib )
add_subdirectory ( application )

set ( CPACK_PROJECT_NAME ${PROJECT_NAME} )
set ( CPACK_PROJECT_VERSION ${PROJECT_VERSION} )
include ( CPack )
