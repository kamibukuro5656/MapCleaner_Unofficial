#pragma once

#include <ros/ros.h>
#include <memory>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <nanoflann/nanoflann.hpp>

#define FRAME_ID "map"

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> CloudType;
typedef std::shared_ptr<ros::Publisher> PublisherPtr;
typedef pcl::PointIndices PIndices;

namespace grid_map{
  typedef std::shared_ptr<grid_map::GridMap> GridMapPtr;
  typedef std::shared_ptr<const grid_map::GridMap> GridMapConstPtr;
}

template<typename PointT>
struct PointCloudAdaptor
{
  typedef typename pcl::PointCloud<PointT>::ConstPtr CloudConstPtr;
  CloudConstPtr cloud_ptr_;
  
  inline size_t kdtree_get_point_count() const
  {
    return cloud_ptr_->size();
  }
  inline float kdtree_get_pt(const size_t idx, int dim) const
  {
    switch(dim){
      case 0:
        return cloud_ptr_->points[idx].x;
      case 1:
        return cloud_ptr_->points[idx].y;
      case 2:
        return cloud_ptr_->points[idx].z;
      default:
        return 0.0;
    }
  }
  template <class BBOX> bool kdtree_get_bbox(BBOX&) const { return false; }
};


template<typename PointT>
using KDTreeNanoFlannPCL = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<float, PointCloudAdaptor<PointT>>,
                                                               PointCloudAdaptor<PointT>, 3, int>;