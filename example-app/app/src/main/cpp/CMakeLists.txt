cmake_minimum_required(VERSION 3.4.1)
project(example)

include(${CMAKE_SOURCE_DIR}/cmake/conan.cmake)

conan_cmake_run(CONANFILE conanfile.txt
                PROFILE arm64-v8a
                BASIC_SETUP
                UPDATE
                BUILD missing)

include(${CMAKE_CURRENT_BINARY_DIR}/conan_paths.cmake)

find_package(pcl REQUIRED)

add_library(native-lib SHARED native-lib.cpp)

find_library(log-lib log)

target_link_libraries(native-lib
    PUBLIC
    ${log-lib}
    pcl::pcl
    )