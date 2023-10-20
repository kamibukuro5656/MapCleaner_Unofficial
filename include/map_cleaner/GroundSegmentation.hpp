#pragma once

#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <map_cleaner/DataLoader.hpp>
#include "travel/tgs.hpp"
#include "travel/aos.hpp"
#include <voxel_grid_large.h>

typedef std::shared_ptr<travel::TravelGroundSeg<PointType>> TravelGroundSegPtr;
typedef std::shared_ptr<const travel::TravelGroundSeg<PointType>> TravelGroundSegConstPtr;
typedef std::shared_ptr<travel::ObjectCluster<PointType>> ObjectClusterPtr;
typedef std::shared_ptr<const travel::ObjectCluster<PointType>> ObjectClusterConstPtr;

void buildTravel(const ros::NodeHandle &nh, TravelGroundSegPtr &ground_seg, ObjectClusterPtr &object_seg)
{
  //
  // Created by Minho Oh & Euigon Jung on 1/31/22.
  // We really appreciate Hyungtae Lim and Prof. Hyun Myung! :)
  //
  float min_range, max_range;
  int vert_scan, horz_scan;
  float min_vert_angle, max_vert_angle;
  nh.param<float>("/lidar/min_range", min_range, 0.0);
  nh.param<float>("/lidar/max_range", max_range, 30.0);
  nh.param<int>   ("/lidar/vert_scan"     , vert_scan, 64);
  nh.param<int>   ("/lidar/horz_scan"     , horz_scan, 1800);
  nh.param<float> ("/lidar/min_vert_angle", min_vert_angle, -30.0);
  nh.param<float> ("/lidar/max_vert_angle", max_vert_angle, 50.0);

  float tgf_res, th_seeds, th_dist, th_outlier, th_normal, th_weight, th_lcc_normal_similarity, th_lcc_planar_dist, th_obstacle;
  int num_iter, num_lpr, num_min_pts;
  bool refine_mode, viz_mode;
  nh.param<float>("/tgs/resolution", tgf_res, 5.0);
  nh.param<int>("/tgs/num_iter", num_iter, 3);
  nh.param<int>("/tgs/num_lpr", num_lpr, 20);
  nh.param<int>("/tgs/num_min_pts", num_min_pts, 10);
  nh.param<float>("/tgs/th_seeds", th_seeds, 0.4);
  nh.param<float>("/tgs/th_dist", th_dist, 0.3);
  nh.param<float>("/tgs/th_outlier", th_outlier, 0.3);
  nh.param<float>("/tgs/th_normal", th_normal, 0.707);
  nh.param<float>("/tgs/th_weight", th_weight, 1.5);
  nh.param<float>("/tgs/th_obstacle", th_obstacle, 1.5);
  nh.param<float>("/tgs/th_lcc_normal", th_lcc_normal_similarity, 1.5);
  nh.param<float>("/tgs/th_lcc_planar", th_lcc_planar_dist, 1.5);
  nh.param<bool>("/tgs/refine_mode", refine_mode, true);
  nh.param<bool>("/tgs/visualization", viz_mode, true);

  float car_width, car_length, lidar_width_offset, lidar_length_offset, horz_merge_thres, vert_merge_thres;
  int downsample, vert_scan_size, horz_scan_size, horz_skip_size, horz_extension_size, min_cluster_size, max_cluster_size;
  nh.param<int>  ("/aos/downsample"           , downsample, 2);
  nh.param<float>("/aos/car_width"            , car_width, 1.0);
  nh.param<float>("/aos/car_length"           , car_length, 1.0);
  nh.param<float>("/aos/lidar_width_offset"   , lidar_width_offset, 0.0);
  nh.param<float>("/aos/lidar_length_offset"  , lidar_length_offset, 0.0);
  nh.param<float>("/aos/th_horz_merg"         , horz_merge_thres, 0.3);
  nh.param<float>("/aos/th_vert_merg"         , vert_merge_thres, 1.0);
  nh.param<int>  ("/aos/vert_scan_size"       , vert_scan_size, 4);
  nh.param<int>  ("/aos/horz_scan_size"       , horz_scan_size, 4);
  nh.param<int>  ("/aos/horz_skip_size"       , horz_skip_size, 4);
  nh.param<int>  ("/aos/horz_extension_size"  , horz_extension_size, 3);
  nh.param<int>  ("/aos/min_cluster_size"     , min_cluster_size, 4);
  nh.param<int>  ("/aos/max_cluster_size"     , max_cluster_size, 100);

  ground_seg.reset(new travel::TravelGroundSeg<PointType>());
  ground_seg->setParams(max_range, min_range, tgf_res,
                    num_iter, num_lpr, num_min_pts, th_seeds,
                    th_dist, th_outlier, th_normal, th_weight,
                    th_lcc_normal_similarity, th_lcc_planar_dist, th_obstacle,
                    refine_mode, viz_mode);

  object_seg.reset(new travel::ObjectCluster<PointType>());
  object_seg->setParams(vert_scan, horz_scan, min_range, max_range, 
                    min_vert_angle, max_vert_angle,
                    horz_merge_thres, vert_merge_thres, vert_scan_size,
                    horz_scan_size, horz_extension_size, horz_skip_size, downsample, 
                    min_cluster_size, max_cluster_size);
}

class GroundSegmentation
{
public:
  typedef std::shared_ptr<GroundSegmentation> Ptr;
  typedef std::shared_ptr<const GroundSegmentation> ConstPtr;

private:
  TravelGroundSegPtr ground_seg_;
  ObjectClusterPtr object_seg_;
  PublisherPtr pub_ptr_;
  tf2_ros::StaticTransformBroadcaster static_tf_br_;
  pcl::VoxelGridLarge<PointType> vg_;
  bool use_voxel_grid_;
  std::string frame_id_;
  double min_range_, max_range_;
  
  void computeTravel(const CloudType &input_cloud, CloudType::Ptr &ground_cloud, CloudType::Ptr &nonground_cloud)
  {
    //
    // Created by Minho Oh & Euigon Jung on 1/31/22.
    // We really appreciate Hyungtae Lim and Prof. Hyun Myung! :)
    //
    CloudType::Ptr filtered_cloud(new CloudType);

    filtered_cloud->header = input_cloud.header;
    filtered_cloud->reserve(input_cloud.size());
    for (auto &point : input_cloud)
    {
      bool is_nan = std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z);
      double pt_range = 0.0;
      if (is_nan)
      {
        continue;
      }
      pt_range = sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2));
      if (pt_range <= min_range_ || pt_range >= max_range_)
      {
        continue;
      }
      filtered_cloud->push_back(point);
    }

    CloudType::Ptr out_nonground_seg;
    if(object_seg_ != nullptr)
      out_nonground_seg.reset(new CloudType);
    else
      out_nonground_seg = nonground_cloud;

    double ground_seg_time;
    ground_seg_->estimateGround(*filtered_cloud, *ground_cloud, *out_nonground_seg, ground_seg_time);

    if(object_seg_ != nullptr)
      object_seg_->segmentObjects(out_nonground_seg, nonground_cloud);
  }

  void publish(const DataLoaderBase::Frame &frame, const CloudType &ground_frame, const CloudType &nonground_frame)
  {
    if(pub_ptr_ == nullptr)
      return;

    pcl::PointCloud<pcl::PointXYZRGB> vis_cloud;
    vis_cloud.reserve(ground_frame.size() + nonground_frame.size());
    for(int i = 0; i < ground_frame.size(); i++)
    {
      pcl::PointXYZRGB vis_p;
      const PointType &p = ground_frame[i];
      vis_p.x = p.x; vis_p.y = p.y; vis_p.z = p.z;
      vis_p.r = 0; vis_p.g = 255; vis_p.b = 0;
      vis_cloud.push_back(vis_p);
    }
    for(int i = 0; i < nonground_frame.size(); i++)
    {
      pcl::PointXYZRGB vis_p;
      const PointType &p = nonground_frame[i];
      vis_p.x = p.x; vis_p.y = p.y; vis_p.z = p.z;
      vis_p.r = 0; vis_p.g = 255; vis_p.b = 255;
      vis_cloud.push_back(vis_p);
    }

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(vis_cloud, cloud_msg);
    cloud_msg.header.frame_id = frame_id_;
    cloud_msg.header.stamp = ros::Time::now();
    pub_ptr_->publish(cloud_msg);

    geometry_msgs::TransformStamped static_tf;
    static_tf.header.stamp = ros::Time::now();
    static_tf.header.frame_id = frame_id_;
    static_tf.child_frame_id = "lidar";
    static_tf.transform.translation.x = frame.t[0];
    static_tf.transform.translation.y = frame.t[1];
    static_tf.transform.translation.z = frame.t[2];
    static_tf.transform.rotation.x = frame.r.x();
    static_tf.transform.rotation.y = frame.r.y();
    static_tf.transform.rotation.z = frame.r.z();
    static_tf.transform.rotation.w = frame.r.w();
    static_tf_br_.sendTransform(static_tf);
  }

public:
  GroundSegmentation(const TravelGroundSegPtr ground_seg, const ObjectClusterPtr object_seg = nullptr, 
                     const bool use_voxel_grid = false, const float voxel_leaf_size = 0.1, 
                     const PublisherPtr pub_ptr = nullptr, const std::string &frame_id = "map")
  {
    ground_seg_ = ground_seg;
    object_seg_ = object_seg;
    pub_ptr_ = pub_ptr;
    frame_id_ = frame_id;
    use_voxel_grid_ = use_voxel_grid;
    vg_.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);

    double tgf_res, th_seeds, th_dist, th_outlier, th_normal, th_weight, th_lcc_normal_similarity, th_lcc_planar_dist, th_obstacle;
    int num_iter, num_lpr, num_min_pts;
    bool refine_mode, viz_mode;

    ground_seg_->getParams(max_range_, min_range_, tgf_res,
                       num_iter, num_lpr, num_min_pts, th_seeds,
                       th_dist, th_outlier, th_normal, th_weight,
                       th_lcc_normal_similarity, th_lcc_planar_dist, th_obstacle,
                       refine_mode, viz_mode);
  };

  bool compute(DataLoaderBase::Ptr &loader, CloudType &cloud, PIndices &ground_indices, PIndices &nonground_indices)
  {
    if(loader->getSize() <= 0)
      return false;

    cloud.clear();
    DataLoaderBase::Frame first_frame = loader->loadFrame(0);
    if(use_voxel_grid_){
      vg_.setInputCloud(first_frame.frame);
      vg_.filter(*first_frame.frame);
    }
    size_t reserve_size = (size_t)((double)(first_frame.frame->size() * loader->getSize()) * 1.1);
    cloud.reserve(reserve_size);
    ground_indices.indices.clear();
    nonground_indices.indices.clear();
    
    for(int i = 0; i < loader->getSize(); ++i)
    {
      DataLoaderBase::Frame frame = loader->loadFrame(i);

      if(frame.frame->empty()){
        ROS_WARN_STREAM("Frame " << frame.idx + 1 << " is empty.");
        continue;
      }

      Eigen::Matrix4f t_mat = Eigen::Matrix4f::Identity();
      Eigen::Matrix4f r_mat = Eigen::Matrix4f::Identity();
      r_mat.block<3, 3>(0, 0) = frame.r.toRotationMatrix();
      t_mat.block<3, 1>(0, 3) = frame.t;
      
      CloudType::Ptr rotated_frame(new CloudType);
      CloudType::Ptr ground_frame(new CloudType);
      CloudType::Ptr nonground_frame(new CloudType);
      
      pcl::transformPointCloud(*frame.frame, *rotated_frame, r_mat);
      computeTravel(*rotated_frame, ground_frame, nonground_frame);
      pcl::transformPointCloud(*ground_frame, *ground_frame, t_mat);
      pcl::transformPointCloud(*nonground_frame, *nonground_frame, t_mat);

      if(use_voxel_grid_)
      {
        vg_.setInputCloud(ground_frame);
        vg_.filter(*ground_frame);
        vg_.setInputCloud(nonground_frame);
        vg_.filter(*nonground_frame);
      }

      if((int64_t)cloud.size() + (int64_t)ground_frame->size() + (int64_t)nonground_frame->size() > INT32_MAX)
      {
        ROS_ERROR_STREAM("Over INT32_MAX");
        return false;
      }

      for(int i = 0; i < ground_frame->size(); i++)
      {
        cloud.push_back(ground_frame->points[i]);
        ground_indices.indices.push_back(cloud.size() - 1);
      }
      for(int i = 0; i < nonground_frame->size(); i++)
      {
        cloud.push_back(nonground_frame->points[i]);
        nonground_indices.indices.push_back(cloud.size() - 1);
      }

      publish(frame, *ground_frame, *nonground_frame);

      if(i % 100 == 0){
        ROS_INFO_STREAM("Compute TRAVEL: " << i + 1 << " / " << loader->getSize());
      }
    }
    
    DataLoaderBase::Frame dummy_frame;
    dummy_frame.t = loader->getFrameInfo(loader->getSize() - 1).t;
    dummy_frame.r = loader->getFrameInfo(loader->getSize() - 1).r;
    publish(dummy_frame, CloudType(), CloudType()); //dummy
    ROS_INFO_STREAM("Compute TRAVEL: " << loader->getSize() << " / " << loader->getSize());

    cloud.points.shrink_to_fit();
    ground_indices.indices.shrink_to_fit();
    nonground_indices.indices.shrink_to_fit();

    return true;
  };
};
