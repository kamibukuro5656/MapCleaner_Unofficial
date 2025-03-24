cmake_minimum_required(VERSION 3.0.2)
project(map_cleaner)

add_compile_options(-std=c++14 -O3)

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/mod_pcl")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  tf2
  tf2_ros
  grid_map_core
  grid_map_msgs
  grid_map_ros
  grid_map_rviz_plugin
)

# find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  # ${OpenCV_INCLUDE_DIRS}
  ${MOD_PCL_INCLUDE_DIRS}
  thirdparty
  thirdparty/patchworkpp/include
  include
)

add_library(patchworkpp STATIC
  thirdparty/patchworkpp/src/patchworkpp.cpp
)

target_link_libraries(patchworkpp Eigen3::Eigen)

add_executable(${PROJECT_NAME}
  src/map_cleaner.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  # ${OpenCV_LIBS}
  ${MOD_PCL_LIBRARIES}
  patchworkpp
)
