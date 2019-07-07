cmake_minimum_required(VERSION 3.5.1)
project(lz4)

add_library(${PROJECT_NAME} lz4/lib/lz4.c)
target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lz4/lib>
  )
