from conans import ConanFile, CMake, tools


class Lz4Conan(ConanFile):
    name = "lz4"
    version = "1.9.1"
    settings = "os", "compiler", "arch", "build_type"
    description = "Conan package for lz4 library"
    url = "https://github.com/lz4/lz4/"
    license = "BSD 2-Clause"
    exports_sources = ["CMakeLists.txt"]

    def _to_android_abi(self, arch: str) -> str:
        if arch == "armv7": return "armeabi-v7a"
        if arch == "armv8": return "arm64-v8a"
        return None

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
        cmake = self._configure_toolchain(cmake)
        cmake.configure(build_folder=str(self.settings.arch))
        return cmake

    def source(self):
        git = tools.Git(folder="lz4")
        git.clone("https://github.com/lz4/lz4.git", "v{}".format(self.version))

    def build(self):
        cmake = self._configure_cmake()
        cmake.build()

    def package(self):
        self.copy(pattern="*.a", dst="lib", src="{}".format(self.settings.arch))
        self.copy(pattern="lz4.h", dst="include", src="{}/{}/lib/".format(self.source_folder, self.name))

    def package_info(self):
        self.cpp_info.libs = tools.collect_libs(self)
