cmake_minimum_required ( VERSION 3.12.0 )

set ( CONAN_OUTPUT_DIR ${CMAKE_BINARY_DIR}/conan-output )
file ( MAKE_DIRECTORY ${CONAN_OUTPUT_DIR} )

# Execute conan and generate the files in a separate directory
execute_process ( COMMAND conan install ${CMAKE_SOURCE_DIR}/conan/ --install-folder ${CONAN_OUTPUT_DIR} --build missing )
include ( ${CONAN_OUTPUT_DIR}/conanbuildinfo.cmake )
conan_basic_setup ( TARGETS )

# # Set CMAKE_MODULE_PATH to conan output directory
list ( APPEND CMAKE_MODULE_PATH ${CONAN_OUTPUT_DIR} )