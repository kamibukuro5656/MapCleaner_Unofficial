#pragma once

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <map_cleaner/utils.hpp>

class GridMapBuilder
{
public:
  typedef std::shared_ptr<GridMapBuilder> Ptr;
  typedef std::shared_ptr<const GridMapBuilder> ConstPtr;

private:
  std::vector<std::string> layer_names_;
  float resolution_;
  std::string frame_id_;

  inline void getAABB2D(const CloudType &input, float &x_min, float &x_max, float &y_min, float &y_max)
  {
    x_min = std::numeric_limits<float>::infinity();
    x_max = -std::numeric_limits<float>::infinity();
    y_min = std::numeric_limits<float>::infinity();
    y_max = -std::numeric_limits<float>::infinity();

    for(size_t i = 0; i < input.size(); i++)
    {
      x_min = std::min(input[i].x, x_min);
      x_max = std::max(input[i].x, x_max);
      y_min = std::min(input[i].y, y_min);
      y_max = std::max(input[i].y, y_max);
    }
  }

  inline void updateCell(const PointType &p, 
                         float &cell_cnt, float &cell_avg, float &cell_var,
                         float &cell_min, float &cell_max, float &cell_int)
  {
    if (!std::isfinite(cell_cnt)) // init
    {
      cell_cnt = 1.0;
      cell_avg = p.z;
      cell_var = 0.0;
      cell_min = p.z;
      cell_max = p.z;
      cell_int = p.intensity;
    }
    else
    {
      float last_avg = cell_avg;
      cell_avg = (cell_avg * cell_cnt + p.z) / (cell_cnt + 1.0);
      cell_var = (((cell_cnt * (cell_var + last_avg * last_avg)) + (p.z * p.z)) / (cell_cnt + 1.0)) - (cell_avg * cell_avg);
      cell_min = std::min(cell_min, p.z);
      cell_max = std::max(cell_max, p.z);
      cell_int = std::max(cell_int, p.intensity);

      cell_cnt += 1.0;
    }
  }

public:
  GridMapBuilder(const float &resolution, const std::string &frame_id, 
                 const std::string &count_layer = "count", 
                 const std::string &average_layer = "average", 
                 const std::string &variance_layer = "variance", 
                 const std::string &min_layer = "min", 
                 const std::string &max_layer = "max", 
                 const std::string &intensity_layer = "intensity"
                 )
  {
    resolution_ = resolution;
    frame_id_ = frame_id;
    
    layer_names_.clear();
    layer_names_.push_back(count_layer);
    layer_names_.push_back(average_layer);
    layer_names_.push_back(variance_layer);
    layer_names_.push_back(min_layer);
    layer_names_.push_back(max_layer);
    layer_names_.push_back(intensity_layer);
  }

  grid_map::GridMapPtr compute(const CloudType &cloud, const PIndices &ground_indices)
  {
    if(cloud.empty() || ground_indices.indices.empty())
      return nullptr;

    grid_map::GridMapPtr grid_ptr(new grid_map::GridMap(layer_names_));
    float x_min, x_max, y_min, y_max;
    getAABB2D(cloud, x_min, x_max, y_min, y_max);
    float x_len = x_max - x_min;
    float y_len = y_max - y_min;

    if(!std::isfinite(x_min) || !std::isfinite(x_max) ||
       !std::isfinite(y_min) || !std::isfinite(y_max))
       return nullptr;

    grid_ptr->setFrameId(frame_id_);
    grid_ptr->setGeometry(grid_map::Length(x_len, y_len), resolution_, grid_map::Position(x_len * 0.5 + x_min, y_len * 0.5 + y_min));

    grid_map::Matrix &count_layer = (*grid_ptr)[layer_names_[0]];
    grid_map::Matrix &average_layer = (*grid_ptr)[layer_names_[1]];
    grid_map::Matrix &variance_layer = (*grid_ptr)[layer_names_[2]];
    grid_map::Matrix &min_layer = (*grid_ptr)[layer_names_[3]];
    grid_map::Matrix &max_layer = (*grid_ptr)[layer_names_[4]];
    grid_map::Matrix &intensity_layer = (*grid_ptr)[layer_names_[5]];

    for(size_t i = 0; i <ground_indices.indices.size(); i++)
    {
      int p_idx = ground_indices.indices[i];
      grid_map::Index idx;
      if(!grid_ptr->getIndex(grid_map::Position(cloud[p_idx].x, cloud[p_idx].y), idx))
        continue;
    
      updateCell(cloud[p_idx], 
                 count_layer(idx[0], idx[1]), average_layer(idx[0], idx[1]), variance_layer(idx[0], idx[1]), 
                 min_layer(idx[0], idx[1]), max_layer(idx[0], idx[1]), intensity_layer(idx[0], idx[1]));
    }

    return grid_ptr;
  }
};