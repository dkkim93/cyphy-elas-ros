cmake_minimum_required(VERSION 2.8.3)
project(elas_ros)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin_simple REQUIRED)
catkin_simple()
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS signals)

add_definitions(-msse3)
include_directories(
  src 
  ${libelas_INCLUDE_DIRS} 
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${Boost_INCLUDE_DIRS})

cs_add_executable(elas_ros src/elas.cpp)
add_dependencies(elas_ros ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)
target_link_libraries(elas_ros ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

cs_add_executable(elas_reconfigure src/elas_reconfigure.cpp)
add_dependencies(elas_reconfigure ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)


cs_add_executable(pc_construction src/pc_construction.cpp)
add_dependencies(pc_construction ${PROJECT_NAME}_gencpp)
target_link_libraries(pc_construction ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

cs_install()
cs_export()
