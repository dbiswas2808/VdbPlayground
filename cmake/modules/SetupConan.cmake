cmake_minimum_required(VERSION 3.12.0)

# Execute Conan
execute_process(COMMAND conan install ${CMAKE_SOURCE_DIR}/conan/ --build missing)
include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup(TARGETS)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_BINARY_DIR}/")
