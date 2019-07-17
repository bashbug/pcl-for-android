from conans import ConanFile, CMake, tools


class FlannConan(ConanFile):
    name = "flann"
    version = "1.9.1"
    settings = "os", "compiler", "arch", "build_type"
    description = "Conan package for flann library"
    url = "http://www.cs.ubc.ca/research/flann/"
    license = "BSD"

    def _to_android_abi(self, arch: str) -> str:
        if arch == "armv7": return "armeabi-v7a"
        if arch == "armv8": return "arm64-v8a"
        return arch

    def _to_android_platform(self, api_level: str) -> str:
        return "android-{}".format(api_level)

    def _configure_toolchain(self, cmake):
        cmake.definitions["CMAKE_TOOLCHAIN_FILE"] = self.deps_env_info["android-toolchain"].ANDROID_TOOLCHAIN_FILE_PATH
        cmake.definitions["ANDROID_STL"] = "c++_shared"
        cmake.definitions["ANDROID_TOOLCHAIN"] = self.settings.compiler
        cmake.definitions["ANDROID_PLATFORM"] = self._to_android_platform(self.settings.os.api_level)
        cmake.definitions["ANDROID_ABI"] = self._to_android_abi(str(self.settings.arch))
        return cmake

    def _configure_cmake(self):
        cmake = CMake(self)
        cmake.definitions["CMAKE_BUILD_TYPE"] = "Release"
        cmake.definitions["CMAKE_INSTALL_PREFIX"] = "{}/{}".format(self.build_folder, self.settings.arch)
        cmake.definitions["CMAKE_CXX_VISIBILITY_PRESET"] = "hidden"
        cmake.definitions["BUILD_C_BINDINGS"] = "OFF"
        cmake.definitions["BUILD_PYTHON_BINDINGS"] = "OFF"
        cmake.definitions["BUILD_MATLAB_BINDINGS"] = "OFF"
        cmake.definitions["BUILD_EXAMPLES"] = "OFF"
        cmake.definitions["BUILD_TESTS"] = "OFF"
        cmake.definitions["BUILD_DOC"] = "OFF"
        cmake = self._configure_toolchain(cmake)
        cmake.configure(build_folder=str(self.settings.arch))
        return cmake

    def requirements(self):
        self.requires("lz4/1.9.1@bashbug/stable")

    def source(self):
        git = tools.Git()
        git.clone("https://github.com/mariusmuja/flann.git", self.version)

    def build(self):
        cmake = self._configure_cmake()
        cmake.build()

    def package(self):
        cmake = self._configure_cmake()
        cmake.install()
        self.copy(pattern="*.a", dst="lib", src="{}/lib".format(self.settings.arch))
        # add lz4 as transtive dependency
        self.run("cp {}/liblz4.a {}/lib".format(self.deps_cpp_info["lz4"].lib_paths[0], self.package_folder))
        self.copy(pattern="*", dst="include", src="{}/include".format(self.settings.arch))

    def package_info(self):
        self.cpp_info.libs = tools.collect_libs(self)
