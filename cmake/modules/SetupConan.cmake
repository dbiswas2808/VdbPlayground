# ensure_conan(_VENV_DIR)

function (patch_conan_generated_cmake_files_INSTALL_FOLDER)
    set(SCRIPT_PATH "${CMAKE_SOURCE_DIR}/cmake/scripts/patch_conan_generated_cmake_files.py")
    set_property(DIRECTORY "${CMAKE_SOURCE_DIR}" APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${SCRIPT_PATH}")
    execute_process_with_error_checking("${Python3_EXECUTABLE}" "${SCRIPT_PATH}" "${_INSTALL_FOLDER}" "${CMAKE_BUILD_TYPE}")

function (setup_conan)
    set_property(DIRECTORY "${CMAKE_SOURCE_DIR}" APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/conanfile.py")
    set(_VENV_DIR "${CMAKE_BINARY_DIR}/venv/conan")
    ensure_conan("${_VENV_DIR}")
    
    include("${CMAKE_SOURCE_DIR}/thirdparty/cmake-conan/external/conan.cmake")
    conan_cmake_autodetect(_CONAN_BUILD_MACHINE_SETTINGS)
    patch_conan_settings(_CONAN_BUILD_MACHINE_SETTINGS)
    conan_cmake_autodetect(_CONAN_TARGET_MACHINE_SETTINGS ARCH x86_64)
    patch_conan_settings(_CONAN_TARGET_MACHINE_SETTINGS)
    conan_build_missing_arg(_BUILD_MISSING_ARG)
    conan_add_remote(NAME conancenter 
                     INDEX 0
                     URL https://center.conan.io
                     VERIFY_SSL True)
    
    set(_INSTALL_FORLDER "${_VENV_DIR}/generated")

    conan_cmake_install(PATH_OR_REFERENCE "${CMAKE_SOURCE_DIR}" 
                        INSTALL_FOLDER "${_INSTALL_FOLDER}" 
                        REMOTE conancenter 
                        SETTINGS_BUILD ${_CONAN_BUILD_MACHINE_SETTINGS} 
                        SETTINGS_HOST ${_CONAN_TARGET_MACHINE_SETTINGS})

    patch_conan_generated_cmake_files("${_INSTALL_FOLDER}")
    list(APPEND CMAKE_PREFIX_PATH "${_INSTALL_FOLDER}")
    set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH}" PARENT_SCOPE)

endfunction()