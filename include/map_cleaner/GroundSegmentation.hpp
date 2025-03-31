#pragma once

#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <map_cleaner/DataLoader.hpp>
#include <voxel_grid_large.h>
#include <patchwork/patchworkpp.h>
#include <pcl/filters/extract_indices.h>

typedef std::shared_ptr<patchwork::PatchWorkpp> PatchWorkppPtr;
typedef std::shared_ptr<const patchwork::PatchWorkpp> PatchWorkppConstPtr;

class GroundSegmentation
{
public:
  typedef std::shared_ptr<GroundSegmentation> Ptr;
  typedef std::shared_ptr<const GroundSegmentation> ConstPtr;

private:
  PatchWorkppPtr ground_seg_;
  PublisherPtr pub_ptr_;
  tf2_ros::StaticTransformBroadcaster static_tf_br_;
  pcl::VoxelGridLarge<PointType> vg_;
  bool use_voxel_grid_;
  std::string frame_id_;
  int frame_skip_;
  
  void computePatchWorkpp(const CloudType::Ptr &input_cloud, CloudType::Ptr &ground_cloud, CloudType::Ptr &nonground_cloud)
  {
    Eigen::MatrixXf input_matrix;
    input_matrix.resize(input_cloud->size(), 4);
    for(int i = 0; i < input_cloud->size(); ++i)
      input_matrix.row(i) << (*input_cloud)[i].x, (*input_cloud)[i].y, (*input_cloud)[i].z, (*input_cloud)[i].intensity;

    ground_seg_->estimateGround(input_matrix);
    Eigen::VectorXi ground_idx = ground_seg_->getGroundIndices();
    Eigen::VectorXi nonground_idx = ground_seg_->getNongroundIndices();

    pcl::ExtractIndices<PointType> extract;
    pcl::PointIndices::Ptr pcl_ground_indices(new pcl::PointIndices());
    pcl_ground_indices->indices.resize(ground_idx.size());
    for(int i = 0; i < ground_idx.size(); ++i)
      pcl_ground_indices->indices[i] = ground_idx[i];
    
    extract.setInputCloud(input_cloud);
    extract.setIndices(pcl_ground_indices);
    extract.setNegative(false);
    extract.filter(*ground_cloud);
    
    pcl::PointIndices::Ptr pcl_nonground_indices(new pcl::PointIndices());
    pcl_nonground_indices->indices.resize(nonground_idx.size());
    for(int i = 0; i < nonground_idx.size(); ++i)
      pcl_nonground_indices->indices[i] = nonground_idx[i];
    
    extract.setInputCloud(input_cloud);
    extract.setIndices(pcl_nonground_indices);
    extract.setNegative(false);
    extract.filter(*nonground_cloud);
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
  GroundSegmentation(const patchwork::Params &params, 
                     const bool use_voxel_grid = false, const float voxel_leaf_size = 0.1, const int frame_skip = 0, 
                     const PublisherPtr pub_ptr = nullptr, const std::string &frame_id = "map")
  {
    ground_seg_ = PatchWorkppPtr(new patchwork::PatchWorkpp(params));
    pub_ptr_ = pub_ptr;
    frame_id_ = frame_id;
    use_voxel_grid_ = use_voxel_grid;
    vg_.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    frame_skip_ = frame_skip;
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
    size_t reserve_size = (size_t)((double)(first_frame.frame->size() * loader->getSize() / (frame_skip_ + 1)) * 1.1);
    cloud.reserve(reserve_size);
    ground_indices.indices.clear();
    nonground_indices.indices.clear();
    
    for(int i = 0; i < loader->getSize(); ++i)
    {
      DataLoaderBase::Frame frame = loader->loadFrame(i);

      if(i % (frame_skip_ + 1) != 0)
        continue;

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
      computePatchWorkpp(rotated_frame, ground_frame, nonground_frame);
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
        ROS_INFO_STREAM("Compute PatchWorkpp: " << i + 1 << " / " << loader->getSize());
      }
    }
    
    DataLoaderBase::Frame dummy_frame;
    dummy_frame.t = loader->getFrameInfo(loader->getSize() - 1).t;
    dummy_frame.r = loader->getFrameInfo(loader->getSize() - 1).r;
    publish(dummy_frame, CloudType(), CloudType()); //dummy
    ROS_INFO_STREAM("Compute PatchWorkpp: " << loader->getSize() << " / " << loader->getSize());

    cloud.points.shrink_to_fit();
    ground_indices.indices.shrink_to_fit();
    nonground_indices.indices.shrink_to_fit();

    return true;
  };
};
