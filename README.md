# pcl-for-android

1. Cross-compilation of [PCL](https://github.com/PointCloudLibrary/pcl) using conan.
2. Full integration in an Android example app.

## Tested setup

* Ubuntu 18.04 (probably any other Linux machine too)
* conan 1.16.1
* pcl build needs with 8 Cores, ~14GB RAM and around 15 min compile time.

## Install

```
sudo apt install cmake git make ninja python3-pip
```

Install [conan](https://docs.conan.io/en/latest/installation.html)

```
sudo pip3 install conan
```

## Cross-compilation
For arm64-v8a with Android NDK r20:
- flann 1.9.1
- lz4 1.9.1
- boost 1.70.0
- PCL 1.9.1 (Eigen 3.3.7 gets automatically installed)

### arm64-v8a
```
./pcl-build-for-android.sh arm64-v8a
```

### armeabi-v7a
NOTE: "Starting August 1, 2019, your apps published on Google Play will need to support 64-bit architectures." (see https://developer.android.com/distribute/best-practices/develop/64-bit)
```
./pcl-build-for-android.sh armeabi-v7a
```

### x86_64 (emulator)
```
./pcl-build-for-android.sh x86_64
```
## Example-app
Set in app/build.gradle the abiFilters depending for which architectures you have cross-compiled.
The default setup is arm64-v8a:
```
android {
    compileSdkVersion 28
    defaultConfig {
        ...
        externalNativeBuild {
            cmake {
                cppFlags "-std=c++11"
            }
        }
        ndk {
            abiFilters "arm64-v8a"
        }
    }
...
```
E.g. set in app/build.gradle to support armeabi-v7a and emulator (x86_64):
```
android {
    defaultConfig {
        ...
        ndk {
            abiFilters "armeabi-v7a", "x86_64"
        }
    }
...
```
Or to support all:
```
android {
    defaultConfig {
        ...
        ndk {
            abiFilters "arm64-v8a", "armeabi-v7a", "x86_64"
        }
    }
...
```
Now you can run the app and you will see in Logcat:
```
I/bashbug.example: pointcloud has size 5
```

## A few details
The CMake based example-app has conan fully integrated:

```
example-app/app/src/main
...
├── cpp
│   ├── cmake
│   │   └── conan.cmake
│   ├── CMakeLists.txt
│   ├── conanfile.txt
│   └── native-lib.cpp
...
```

***conanfile.txt*** defines the project's dependency to PCL.
```
[requires]
pcl/1.9.1@bashbug/stable

[generators]
cmake_paths
cmake_find_package
```
***CMakeLists.txt*** includes cmake/conan.cmake which is a cmake integration of [conan](https://github.com/conan-io/cmake-conan/blob/develop/conan.cmake).
```
include(${CMAKE_SOURCE_DIR}/cmake/conan.cmake)

conan_cmake_run(CONANFILE conanfile.txt
                PROFILE arm64-v8a
                BASIC_SETUP
                UPDATE
                BUILD missing)

include(${CMAKE_CURRENT_BINARY_DIR}/conan_paths.cmake)

```
`conan_cmake_run()` does two things:
* It parses the conanfile.txt what dependencies it should install.
* In conanfile.txt are two generators defined cmake_paths and cmake_find_package.
  * `cmake_paths` creates conan_paths.cmake within the build folder. This adds to the `CMAKE_MODULE_PATH` and `CMAKE_PREFIX_PATH` the search path for the cross-compiled libraries.
  * `cmake_find_package` creates auto-generated Find*.cmake files within the build folder .externalNativeBuild/cmake/debug/arm64-v8a

```
example-app/app/.externalNativeBuild
└── cmake
    └── debug
        └── arm64-v8a
            ...
            ├── conanbuildinfo.cmake
            ├── conanbuildinfo.txt
            ├── conaninfo.txt
            ├── conan_paths.cmake
            ├── Findandroid-toolchain.cmake
            ├── Findboost.cmake
            ├── Findeigen.cmake
            ├── Findflann.cmake
            ├── Findlz4.cmake
            ├── Findpcl.cmake
            ├── graph_info.json
            ├── lib
            │   └── libnative-lib.so
            └── rules.ninja
```
This Find*.cmake files resolve `find_package` calls in CMakeLists.txt of cross-compiled libraries and provide targets like `pcl::pcl`:

```
find_package(pcl REQUIRED)

add_library(native-lib SHARED native-lib.cpp)

find_library(log-lib log)

target_link_libraries(native-lib
    PUBLIC
    ${log-lib}
    pcl::pcl
    )
```
