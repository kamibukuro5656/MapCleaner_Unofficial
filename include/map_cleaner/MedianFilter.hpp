#pragma once

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <map_cleaner/utils.hpp>
#include <algorithm>

class MedianFilter
{
public:
  typedef std::shared_ptr<MedianFilter> Ptr;
  typedef std::shared_ptr<const MedianFilter> ConstPtr;

private:
  int kernel_size_;

  std::string input_layer_name_;
  std::string output_layer_name_;

  void fillVector(grid_map::GridMap &grid, grid_map::Matrix &input_layer, const grid_map::Index idx, std::vector<float> &cells)
  {
    grid_map::Index start_idx(idx[0] - (kernel_size_ / 2), idx[1] - (kernel_size_ / 2));
    grid_map::Size kernel_size(kernel_size_, kernel_size_);
    for(grid_map::SubmapIterator it(grid, start_idx, kernel_size); !it.isPastEnd(); ++it){
     if(0 > (*it)[0] || (*it)[0] >= input_layer.rows() ||
         0 > (*it)[1] || (*it)[1] >= input_layer.cols()){
        continue;
      }
      float val = input_layer((*it)[0], (*it)[1]);
      if(!std::isfinite(val)){
        continue;
      }

      cells.push_back(val);
    }
  }

public:
  MedianFilter(const int kernel_size,
                 const std::string input_layer = "elevation", 
                 const std::string output_layer = "elevation_filtered")
  {
    kernel_size_ = kernel_size;
    if(kernel_size_ % 2 == 0)
      kernel_size_ + 1;

    input_layer_name_ = input_layer;
    output_layer_name_ = output_layer;
  }

  bool compute(grid_map::GridMap &grid)
  {
    if(!grid.exists(input_layer_name_)){
      ROS_ERROR_STREAM("GridMap Does Not Have The Required Layers.");
      return false;
    }

    if(grid.exists(output_layer_name_))
      grid.clear(output_layer_name_);
    else
      grid.add(output_layer_name_);

    grid_map::Matrix &input_layer = grid[input_layer_name_];
    grid_map::Matrix &output_layer = grid[output_layer_name_];

    for(grid_map::GridMapIterator it(grid); !it.isPastEnd(); ++it){
      const size_t i = it.getLinearIndex();

      if(!std::isfinite(input_layer(i)))
        continue;

      std::vector<float> cells;
      cells.reserve(kernel_size_ * kernel_size_);
      fillVector(grid, input_layer, *it, cells);
      
      if(cells.empty())
        continue;

      size_t n = cells.size() / 2;
      std::nth_element(cells.begin(), cells.begin() + n, cells.end());
      output_layer(i) = cells[n];
    }

    return true;
  }
};