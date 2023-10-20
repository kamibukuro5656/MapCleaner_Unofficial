#pragma once

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <map_cleaner/utils.hpp>
#include <map_cleaner/DataLoader.hpp>
#include <vector>

class TrajectoryFilter
{
public:
  typedef std::shared_ptr<TrajectoryFilter> Ptr;
  typedef std::shared_ptr<const TrajectoryFilter> ConstPtr;

private:
  double normal_th_;

  std::string input_layer_name_;
  std::string output_layer_name_;
  const std::string normal_layer_name_ = "tmp_layer_normal";
  const std::string mask_layer_name_ = "tmp_layer_mask";

  void computeNormal(grid_map::GridMap &grid)
  {
    if(grid.exists(normal_layer_name_))
      grid.clear(normal_layer_name_);
    else
      grid.add(normal_layer_name_);

    double resolution = grid.getResolution();
    grid_map::Matrix &input_layer = grid[input_layer_name_];
    grid_map::Matrix &normal_layer = grid[normal_layer_name_];

    grid_map::Index start_idx(1, 1);
    grid_map::Size submap_size(grid.getSize() - 2);
    for(grid_map::SubmapIterator it(grid, start_idx, submap_size); !it.isPastEnd(); ++it) {
      float z = input_layer((*it)[0], (*it)[1]);
      float z_l = input_layer((*it)[0], (*it)[1] - 1);
      float z_r = input_layer((*it)[0], (*it)[1] + 1);
      float z_u = input_layer((*it)[0] - 1, (*it)[1]);
      float z_d = input_layer((*it)[0] + 1, (*it)[1]);

      float z_diff_ud;
      float distance_ud;
      if(std::isfinite(z_u) && std::isfinite(z_d)){
        z_diff_ud = z_d - z_u;
        distance_ud = 2.0 * resolution;
      }
      else if(std::isfinite(z) && std::isfinite(z_d)){
        z_diff_ud = z_d - z;
        distance_ud = resolution;
      }
      else if(std::isfinite(z_u) && std::isfinite(z)){
        z_diff_ud = z - z_u;
        distance_ud = resolution;
      }
      else{
        continue;
      }
      
      float z_diff_lr;
      float distance_lr;
      if(std::isfinite(z_l) && std::isfinite(z_r)){
        z_diff_lr = z_r - z_l;
        distance_lr = 2.0 * resolution;
      }
      else if(std::isfinite(z) && std::isfinite(z_r)){
        z_diff_lr = z_r - z;
        distance_lr = resolution;
      }
      else if(std::isfinite(z_l) && std::isfinite(z)){
        z_diff_lr = z - z_l;
        distance_lr = resolution;
      }
      else{
        continue;
      }

      Eigen::Vector3d n;
      n(0) = z_diff_ud / distance_ud;
      n(1) = z_diff_lr / distance_lr;
      n(2) = 1.0;
      n.normalize();
      double n_z = n.dot(Eigen::Vector3d::UnitZ());
      if(n_z < 0.0){
        n_z = -n_z;
      }
      normal_layer((*it)[0], (*it)[1]) = std::acos(n_z);
    }
  }

  void openEightNeighbor(std::vector<grid_map::Index> &open_list, grid_map::Matrix &mask_layer, const grid_map::Index &idx)
  {
    mask_layer(idx[0], idx[1]) = 1.0;
    for(int i = -1; i <= 1; i++){
      for(int j = -1; j <= 1; j++){
        grid_map::Index n_idx(idx[0] + i, idx[1] + j);
        if(0 > n_idx[0] || n_idx[0] >= mask_layer.rows() ||
           0 > n_idx[1] || n_idx[1] >= mask_layer.cols()){
          continue;
        }

        if(!std::isfinite(mask_layer(n_idx[0], n_idx[1]))){
          open_list.push_back(n_idx);
        }
      }
    }
  }

  void computeMask(grid_map::GridMap &grid, const DataLoaderBase::ConstPtr &loader)
  {
    if(grid.exists(mask_layer_name_))
      grid.clear(mask_layer_name_);
    else
      grid.add(mask_layer_name_);
    
    grid_map::Matrix &normal_layer = grid[normal_layer_name_];
    grid_map::Matrix &mask_layer = grid[mask_layer_name_];

    int rows = grid.getSize()[0];
    int cols = grid.getSize()[1];

    std::vector<grid_map::Index> open_list;
    open_list.reserve(rows * cols);
    for(int i = 0; i < loader->getSize(); i++)
    {
      DataLoaderBase::FrameInfo info = loader->getFrameInfo(i);
      grid_map::Index idx;
      if(grid.getIndex(grid_map::Position(info.t[0], info.t[1]), idx)){
        if(std::isfinite(normal_layer(idx[0], idx[1]))){
          open_list.push_back(idx);
        }
      }
    }

    while(!open_list.empty())
    {
      grid_map::Index idx = open_list.back();
      open_list.pop_back();

      float normal = normal_layer(idx[0], idx[1]);
      if(std::isfinite(normal) && !std::isfinite(mask_layer(idx[0], idx[1]))){
        if(std::fabs(normal) < normal_th_){
          openEightNeighbor(open_list, mask_layer, idx);
        }
      }
    }
  }

  void applyMask(grid_map::GridMap &grid)
  {
    if(grid.exists(output_layer_name_))
      grid.clear(output_layer_name_);
    else
      grid.add(output_layer_name_);
    
    grid_map::Matrix &input_layer = grid[input_layer_name_];
    grid_map::Matrix &mask_layer = grid[mask_layer_name_];
    grid_map::Matrix &output_layer = grid[output_layer_name_];

    for(grid_map::GridMapIterator it(grid); !it.isPastEnd(); ++it){
      size_t i = it.getLinearIndex();
      if(std::isfinite(mask_layer(i)))
        output_layer(i) = input_layer(i);
    }
  }

public:
  TrajectoryFilter(const double normal_th,
            const std::string input_layer = "elevation",
            const std::string output_layer = "elevation_filtered")
  {
    normal_th_ = normal_th;

    input_layer_name_ = input_layer;
    output_layer_name_ = output_layer;
  }

  bool compute(grid_map::GridMap &grid, const DataLoaderBase::ConstPtr &loader)
  {
    if(!grid.exists(input_layer_name_)){
      ROS_ERROR_STREAM("GridMap Does Not Have The Required Layers.");
      return false;
    }
    
    computeNormal(grid);
    computeMask(grid, loader);
    applyMask(grid);

    grid.erase(normal_layer_name_);
    grid.erase(mask_layer_name_);

    return true;
  }
};