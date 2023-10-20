#pragma once

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <map_cleaner/utils.hpp>

class VarianceFilter
{
public:
  typedef std::shared_ptr<VarianceFilter> Ptr;
  typedef std::shared_ptr<const VarianceFilter> ConstPtr;

private:
  float variance_th_;
  int min_num_;

  std::vector<std::string> input_layer_names_;
  std::string output_layer_name_;

public:
  VarianceFilter(const float variance_th, const int min_num, 
                 const std::string in_count_layer = "count", 
                 const std::string in_variance_layer = "variance", 
                 const std::string in_elevation_layer = "average", 
                 const std::string output_layer = "variance_filtered")
  {
    variance_th_ = variance_th;
    min_num_ = min_num;

    input_layer_names_.clear();
    input_layer_names_.push_back(in_count_layer);
    input_layer_names_.push_back(in_variance_layer);
    input_layer_names_.push_back(in_elevation_layer);

    output_layer_name_ = output_layer;
  }

  bool compute(grid_map::GridMap &grid)
  {
    if(!grid.exists(input_layer_names_[0]) &&
       !grid.exists(input_layer_names_[1]) &&
       !grid.exists(input_layer_names_[2])){
      ROS_ERROR_STREAM("GridMap Does Not Have The Required Layers.");
      return false;
    }

    if(grid.exists(output_layer_name_))
      grid.clear(output_layer_name_);
    else
      grid.add(output_layer_name_);

    grid_map::Matrix &count_layer = grid[input_layer_names_[0]];
    grid_map::Matrix &variance_layer = grid[input_layer_names_[1]];
    grid_map::Matrix &elevation_layer = grid[input_layer_names_[2]];
    grid_map::Matrix &output_layer = grid[output_layer_name_];

    for(grid_map::GridMapIterator it(grid); !it.isPastEnd(); ++it) {
      const size_t i = it.getLinearIndex();
      float count = count_layer(i);
      float variance = variance_layer(i);
      float elevation = elevation_layer(i);
      if(count >= min_num_ && variance < variance_th_)
      {
        output_layer(i) = elevation;
      }
    }

    return true;
  }
};