from conans import ConanFile, CMake

class TestProject(ConanFile):
    name = "Test"
    version = "0.1"
    settings = "os", "compiler", "build_type", "arch"
    options = {"shared": [True, False]}
    default_options = {"shared": False}
    generators = "cmake", "cmake_find_package"
    # exports_sources = "../*"

    def requirements(self):
        self.requires("eigen/3.4.0")
        self.requires("boost/1.81.0")
        self.requires("jemalloc/5.3.0")
        self.requires("assimp/5.2.2")

    def build(self):
        cmake = CMake(self)
        cmake.build()