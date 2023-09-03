cmake_minimum_required(VERSION 3.15)

find_package(PNG REQUIRED)
include_directories(${PNG_INCLUDE_DIRS})

add_library(RayTracer
RayTracer/Film.h
RayTracer/Geometry.h
RayTracer/GeometryRayIntersector.h
RayTracer/Light.h
RayTracer/Material.h
RayTracer/Ray.h
RayTracer/Scene.h
RayTracer/SceneEntity.h
RayTracer/Film.cpp
RayTracer/Geometry.cpp
RayTracer/GeometryRayIntersector.cpp
RayTracer/Light.cpp
RayTracer/Material.cpp
RayTracer/Ray.cpp
RayTracer/Scene.cpp
RayTracer/SceneEntity.cpp
)

set_target_properties ( RayTracer PROPERTIES LINKER_LANGUAGE CXX )
set_property ( TARGET RayTracer PROPERTY CXX_STANDARD 20 )
set_property ( TARGET RayTracer PROPERTY CXX_STANDARD_REQUIRED ON )

# Link the eigen library to your executable
target_link_libraries(
    RayTracer
    PUBLIC
    Eigen3::Eigen
    ${PNG_LIBRARIES}
)

message(STATUS "PNG_INCLUDE_DIRS: ${PNG_INCLUDE_DIRS}")

add_compile_definitions(_GLIBCXX_USE_CXX11_ABI=0)

find_package(Catch2 3 REQUIRED)

target_include_directories(RayTracer PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/")

add_executable(RayTracerTest 
    RayTracer/Film.t.cpp
    RayTracer/Geometry.t.cpp
    RayTracer/SceneEntity.t.cpp
    RayTracer/Light.t.cpp
)
set_target_properties(RayTracerTest PROPERTIES LINKER_LANGUAGE CXX)
set_property(TARGET RayTracerTest PROPERTY CXX_STANDARD 20)
set_property(TARGET RayTracerTest PROPERTY CXX_STANDARD_REQUIRED ON)

target_link_libraries(
    RayTracerTest 
    PUBLIC 
    Catch2::Catch2WithMain 
    RayTracer)

