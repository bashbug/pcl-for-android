from conans import ConanFile, CMake, tools


class PclConan(ConanFile):
    name = "pcl"
    version = "1.9.1"
    settings = "os", "compiler", "arch", "build_type"
    description = "Conan Package for pcl"
    license = "BSD"
    url = "http://www.pointclouds.org/"
    exports_sources = ["no_except.patch", "pcl_binaries.patch"]

    def _to_android_abi(self, arch: str) -> str:
        if arch == "armv7": return "armeabi-v7a"
        if arch == "armv8": return "arm64-v8a"
        return None

    def _to_android_arch(self, arch: str) -> str:
        if arch == "armv7": return "armv7a"
        if arch == "armv8": return "aarch64"
        return None

    def _to_android_platform(self, api_level: str) -> str:
        return "android-{}".format(api_level)

    def _configure_toolchain(self, cmake):
        cmake.definitions["CMAKE_TOOLCHAIN_FILE"] = self.deps_env_info["android-toolchain"].ANDROID_TOOLCHAIN_FILE_PATH
        cmake.definitions["ANDROID_STL"] = "c++_static"
        cmake.definitions["ANDROID_TOOLCHAIN"] = self.settings.compiler
        cmake.definitions["ANDROID_PLATFORM"] = self._to_android_platform(self.settings.os.api_level)
        cmake.definitions["ANDROID_ABI"] = self._to_android_abi(str(self.settings.arch))
        return cmake

    def _configure_cmake(self):
        cmake = CMake(self)
        cmake.definitions["CMAKE_BUILD_TYPE"] = "Release"
        cmake.definitions["CMAKE_INSTALL_PREFIX"] = "{}/{}".format(self.build_folder, self.settings.arch)
        cmake.definitions["CMAKE_CXX_VISIBILITY_PRESET"] = "hidden"
        cmake.definitions["PCL_SHARED_LIBS"] = "OFF"
        cmake.definitions["PCL_BINARIES"] = "OFF"
        cmake.definitions["WITH_CUDA"] = "OFF"
        cmake.definitions["WITH_OPENGL"] = "OFF"
        cmake.definitions["WITH_PCAP"] = "OFF"
        cmake.definitions["WITH_PNG"] = "OFF"
        cmake.definitions["WITH_QHULL"] = "OFF"
        cmake.definitions["WITH_VTK"] = "OFF"
        cmake.definitions["Boost_INCLUDE_DIR"] = self.deps_cpp_info["boost"].include_paths[0]
        cmake.definitions["Boost_DATE_TIME_LIBRARY_RELEASE:FILEPATH"] = "{}/libboost_date_time.a".format(self.deps_cpp_info["boost"].lib_paths[0])
        cmake.definitions["Boost_DATE_TIME_LIBRARY_DEBUG:FILEPATH"] = "{}/libboost_date_time.a".format(self.deps_cpp_info["boost"].lib_paths[0])
        cmake.definitions["Boost_FILESYSTEM_LIBRARY_RELEASE:FILEPATH"] = "{}/libboost_filesystem.a".format(self.deps_cpp_info["boost"].lib_paths[0])
        cmake.definitions["Boost_FILESYSTEM_LIBRARY_DEBUG:FILEPATH"] = "{}/libboost_filesystem.a".format(self.deps_cpp_info["boost"].lib_paths[0])
        cmake.definitions["Boost_IOSTREAMS_LIBRARY_RELEASE:FILEPATH"] = "{}/libboost_iostreams.a".format(self.deps_cpp_info["boost"].lib_paths[0])
        cmake.definitions["Boost_IOSTREAMS_LIBRARY_DEBUG:FILEPATH"] = "{}/libboost_iostreams.a".format(self.deps_cpp_info["boost"].lib_paths[0])
        cmake.definitions["Boost_SYSTEM_LIBRARY_RELEASE:FILEPATH"] = "{}/libboost_system.a".format(self.deps_cpp_info["boost"].lib_paths[0])
        cmake.definitions["Boost_SYSTEM_LIBRARY_DEBUG:FILEPATH"] = "{}/libboost_system.a".format(self.deps_cpp_info["boost"].lib_paths[0])
        cmake.definitions["Boost_THREAD_LIBRARY_RELEASE:FILEPATH"] = "{}/libboost_thread.a".format(self.deps_cpp_info["boost"].lib_paths[0])
        cmake.definitions["Boost_THREAD_LIBRARY_DEBUG:FILEPATH"] = "{}/libboost_thread.a".format(self.deps_cpp_info["boost"].lib_paths[0])
        cmake.definitions["EIGEN_INCLUDE_DIR"] = self.deps_cpp_info["eigen"].include_paths[0]
        cmake.definitions["FLANN_USE_STATIC"] = "ON"
        cmake.definitions["FLANN_INCLUDE_DIRS"] = self.deps_cpp_info["flann"].include_paths[0]
        cmake.definitions["FLANN_INCLUDE_DIR"] = self.deps_cpp_info["flann"].include_paths[0]
        cmake.definitions["FLANN_LIBRARIES"] = "{}/libflann_cpp_s.a;{}/liblz4.a".format(self.deps_cpp_info["flann"].lib_paths[0], self.deps_cpp_info["flann"].lib_paths[0])
        cmake.definitions["FLANN_LIBRARY"] = "{}/libflann_cpp_s.a;{}/liblz4.a".format(self.deps_cpp_info["flann"].lib_paths[0], self.deps_cpp_info["flann"].lib_paths[0])
        cmake = self._configure_toolchain(cmake)
        cmake.configure(source_folder=self.name, build_folder=str(self.settings.arch))
        return cmake

    def requirements(self):
        self.requires("boost/1.70.0@bashbug/stable")
        self.requires("flann/1.9.1@bashbug/stable") # > 1.8.5 have lz4 issue: https://github.com/mariusmuja/flann/issues/384
        self.requires("eigen/3.3.7@conan/stable")

    def source(self):
        git = tools.Git(folder=self.name)
        git.clone("https://github.com/PointCloudLibrary/pcl.git", "{}-{}".format(self.name, self.version))
        tools.patch(base_path=self.name, patch_file="no_except.patch") # >= boost version 1.70 https://github.com/PointCloudLibrary/pcl/commit/5605910a26f299cb53bd792e923598b3aa5bbc18
        tools.patch(base_path=self.name, patch_file="pcl_binaries.patch")

    def build(self):
        cmake = self._configure_cmake()
        cmake.build()

    def package(self):
        cmake = self._configure_cmake()
        cmake.install()
        self.copy(pattern="*.a", dst="lib", src="{}/lib".format(self.settings.arch))
        self.copy(pattern="*", dst="include", src="{}/include".format(self.settings.arch))

    def package_info(self):
        self.cpp_info.libs = tools.collect_libs(self)
