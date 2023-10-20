#pragma once

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <map_cleaner/utils.hpp>
#include <omp.h>

class BGKFilter
{
public:
  typedef std::shared_ptr<BGKFilter> Ptr;
  typedef std::shared_ptr<const BGKFilter> ConstPtr;

private:
  double kernel_radius_;
  double lambda_;
  double sigma_;
  int min_num_;

  std::string input_layer_name_;
  std::string output_layer_name_;

  inline double kernel(const grid_map::Position &pos, const grid_map::Position &center)
  {
    double d_r = (pos - center).norm() / kernel_radius_;
    double theta = 2.0 * M_PI * d_r;
    return (((2.0 + std::cos(theta)) / 3.0) * (1 - d_r)) + (std::sin(theta) / (2.0 * M_PI));
  }

  inline double priorZ(const double center_z)
  {
    double prior_z = 0;
    if(std::isfinite(center_z))
      prior_z = lambda_ * center_z;
    
    return prior_z;
  }

  inline double computeBGK(const grid_map::GridMap &grid, const grid_map::Position &center, const grid_map::Matrix &input_layer)
  {
    int n = 0;
    double sum_k_z = 0.0;
    double sum_k = 0.0;
    for(grid_map::CircleIterator it(grid, center, kernel_radius_); !it.isPastEnd(); ++it)
    {
      float z = input_layer((*it)[0], (*it)[1]);
      if(!std::isfinite(z))
        continue;

      grid_map::Position pos;
      grid.getPosition(*it, pos);
      double k = kernel(pos, center);

      sum_k += k;
      sum_k_z += k * z;

      n++;
    }

    if(n < min_num_)
      return std::numeric_limits<double>::quiet_NaN();

    grid_map::Index center_idx;
    grid.getIndex(center, center_idx);
    float center_z = input_layer(center_idx[0], center_idx[1]);
    double prior_z = priorZ(center_z);

    return (prior_z + sum_k_z) / (lambda_ + sum_k);
  }
  
  inline double computeBilateralBGK(const grid_map::GridMap &grid, const grid_map::Position &center, 
                                   const grid_map::Matrix &input_layer, const grid_map::Matrix &ref_layer)
  {
    int n = 0;
    double sum_c_k_z = 0.0;
    double sum_c_k = 0.0;

    for(grid_map::CircleIterator it(grid, center, kernel_radius_); !it.isPastEnd(); ++it)
    {
      float z_in = input_layer((*it)[0], (*it)[1]);
      float z_ref = ref_layer((*it)[0], (*it)[1]);
      if(!std::isfinite(z_in) || !std::isfinite(z_ref))
        continue;

      grid_map::Position pos;
      grid.getPosition(*it, pos);
      double k = kernel(pos, center);
      
      double delta = z_in - z_ref;
      double c = std::exp(-(delta * delta) / (2.0 * sigma_ * sigma_));

      sum_c_k += c * k;
      sum_c_k_z += c * k * z_in;

      n++;
    }
    
    if(n < min_num_)
      return std::numeric_limits<double>::quiet_NaN();
    
    grid_map::Index center_idx;
    grid.getIndex(center, center_idx);
    float center_z = input_layer(center_idx[0], center_idx[1]);
    double prior_z = priorZ(center_z);

    return (prior_z + sum_c_k_z) / (lambda_ + sum_c_k);
  }

public:
  BGKFilter(const double kernel_radius, const double lambda, const double sigma, const int min_num,
            const std::string input_layer = "elevation",
            const std::string output_layer = "elevation_filtered")
  {
    kernel_radius_ = kernel_radius;
    lambda_ = lambda;
    sigma_ = sigma;
    min_num_ = min_num;

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
    
    const std::string first_pass_layer_name = "tmp_layer_first_path";
    if(grid.exists(first_pass_layer_name))
      grid.clear(first_pass_layer_name);
    else
      grid.add(first_pass_layer_name);
    
    grid_map::Matrix &input_layer = grid[input_layer_name_];
    grid_map::Matrix &first_pass_layer = grid[first_pass_layer_name];
    grid_map::Matrix &output_layer = grid[output_layer_name_];

    int rows = grid.getSize()[0];
    int cols = grid.getSize()[1];

    #pragma omp parallel for
    for (int i = 0; i < rows; i++)
    {
      grid_map::Index start_idx(i, 0);
      grid_map::Size submap_size(1, cols);
      for (grid_map::SubmapIterator it(grid, start_idx, submap_size); !it.isPastEnd(); ++it)
      {
        grid_map::Position center;
        grid.getPosition(*it, center);
        first_pass_layer((*it)[0], (*it)[1]) = computeBGK(grid, center, input_layer);
      }
    }

    #pragma omp parallel for
    for (int i = 0; i < rows; i++)
    {
      grid_map::Index start_idx(i, 0);
      grid_map::Size submap_size(1, cols);
      for (grid_map::SubmapIterator it(grid, start_idx, submap_size); !it.isPastEnd(); ++it)
      {
        if (std::isfinite(input_layer((*it)[0], (*it)[1])))
        {
          output_layer((*it)[0], (*it)[1]) = input_layer((*it)[0], (*it)[1]);
        }
        else if (std::isfinite(first_pass_layer((*it)[0], (*it)[1])))
        {
          grid_map::Position center;
          grid.getPosition(*it, center);
          output_layer((*it)[0], (*it)[1]) = computeBilateralBGK(grid, center, input_layer, first_pass_layer);
        }
      }
    }

    grid.erase(first_pass_layer_name);

    return true;
  }
};