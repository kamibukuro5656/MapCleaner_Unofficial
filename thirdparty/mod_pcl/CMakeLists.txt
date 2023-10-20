cmake_minimum_required(VERSION 3.0.2)

find_package(PCL REQUIRED)

add_library(voxel_grid_large STATIC src/voxel_grid_large.cpp)
target_include_directories(voxel_grid_large PUBLIC ${PCL_INCLUDE_DIRS} include)
target_link_libraries(voxel_grid_large ${PCL_LIBRARIES})

set(MOD_PCL_LIBRARIES voxel_grid_large PARENT_SCOPE)
set(MOD_PCL_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include PARENT_SCOPE)