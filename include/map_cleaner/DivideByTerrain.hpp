#pragma once

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <map_cleaner/utils.hpp>

class DivideByTerrain
{
public:
  typedef std::shared_ptr<DivideByTerrain> Ptr;
  typedef std::shared_ptr<const DivideByTerrain> ConstPtr;

private:
  std::string input_layer_name_;
  double threshold_;
  bool on_terrain_only_;

  void divide(const CloudType &cloud, const PIndices &input_indices, const grid_map::GridMap &grid, 
              PIndices &ground, PIndices &above, PIndices &below, PIndices &other)
  {
    const grid_map::Matrix &terrain_layer = grid[input_layer_name_];
    for(size_t i = 0; i < input_indices.indices.size(); i++)
    {
      int p_idx = input_indices.indices[i];
      const PointType &p = cloud[p_idx];
      grid_map::Index idx;
      if(!grid.getIndex(grid_map::Position(p.x, p.y), idx)){
        other.indices.push_back(p_idx);
      }
      else if(!std::isfinite(terrain_layer(idx[0], idx[1]))){
        other.indices.push_back(p_idx);
      }
      else{
        float diff_z = p.z - terrain_layer(idx[0], idx[1]);
        if(diff_z < -threshold_){
          below.indices.push_back(p_idx);
        }
        else if(diff_z > threshold_){
          above.indices.push_back(p_idx);
        }
        else{
          ground.indices.push_back(p_idx);
        }
      }
    }
  }

public:
  DivideByTerrain(const double threshold, bool on_terrain_only,
                  const std::string &input_layer = "elevation")
  {
    threshold_ = threshold;
    on_terrain_only_ = on_terrain_only;
    input_layer_name_ = input_layer;
  }

  bool compute(const CloudType &cloud, const PIndices &in_ground_indices, const PIndices &in_nonground_indices, 
               const grid_map::GridMap &grid,
               PIndices &out_ground_indices, PIndices &out_above_indices, PIndices &out_below_indices, PIndices &out_other_indices)
  {
    if(!grid.exists(input_layer_name_)){
      ROS_ERROR_STREAM("GridMap Does Not Have The Required Layers.");
      return false;
    }

    out_ground_indices.indices.clear();
    out_above_indices.indices.clear();
    out_below_indices.indices.clear();
    out_other_indices.indices.clear();

    if(on_terrain_only_){
      divide(cloud, in_ground_indices, grid, out_ground_indices, out_above_indices, out_below_indices, out_other_indices);
      divide(cloud, in_nonground_indices, grid, out_ground_indices, out_above_indices, out_below_indices, out_other_indices);
    }
    else{
      divide(cloud, in_ground_indices, grid, out_ground_indices, out_above_indices, out_below_indices, out_ground_indices);
      divide(cloud, in_nonground_indices, grid, out_ground_indices, out_above_indices, out_below_indices, out_above_indices);
    }

    out_ground_indices.indices.shrink_to_fit();
    out_above_indices.indices.shrink_to_fit();
    out_below_indices.indices.shrink_to_fit();
    out_other_indices.indices.shrink_to_fit();

    return true;
  }
};