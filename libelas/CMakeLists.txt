cmake_minimum_required(VERSION 2.8.3)
project(libelas)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin_simple REQUIRED)

catkin_simple()

# use sse3 instruction set
add_definitions(-msse3)

cs_add_library(elas
  src/descriptor.cpp
  src/elas.cpp
  src/filter.cpp
  src/matrix.cpp
  src/triangle.cpp)

cs_install()
cs_export()
