#include <ros/ros.h>
#include <map_cleaner/DataLoader.hpp>
#include <map_cleaner/GroundSegmentation.hpp>
#include <map_cleaner/GridMapBuilder.hpp>
#include <map_cleaner/VarianceFilter.hpp>
#include <map_cleaner/BGKFilter.hpp>
#include <map_cleaner/TrajectoryFilter.hpp>
#include <map_cleaner/MedianFilter.hpp>
#include <map_cleaner/DivideByTerrain.hpp>
#include <map_cleaner/MovingPointIdentification.hpp>
#include <map_cleaner/utils.hpp>
#include <voxel_grid_large.h>
#include <pcd_io_with_indices.h>

// #define PUBLISH_GRID_MAP

class MapCleaner
{
  ros::NodeHandle nh_;
  ros::NodeHandle nh_p_;

  std::string save_dir_;

  ros::Publisher pub_ground_, pub_static_, pub_dynamic_, pub_ground_below_, pub_other_, pub_terrain_;
  #ifdef PUBLISH_GRID_MAP
  ros::Publisher pub_grid_map_;
  #endif

  DataLoaderBase::Ptr loader_;
  GroundSegmentation::Ptr ground_seg_;
  GridMapBuilder::Ptr grid_map_builder_;
  VarianceFilter::Ptr variance_filter_;
  BGKFilter::Ptr first_bgk_filter_;
  TrajectoryFilter::Ptr trajectory_filter_;
  BGKFilter::Ptr second_bgk_filter_;
  MedianFilter::Ptr median_filter_;
  DivideByTerrain::Ptr divide_by_terrain_;
  MovingPointIdentification::Ptr moving_point_identification_;

  float resolution_;
  std::string frame_id_;
  
  pcl::VoxelGridLarge<PointType> vis_vg_;

  void publishCloud(const CloudType::ConstPtr &cloud, const PIndices::ConstPtr &indices, const std::string frame_id, ros::Publisher &pub)
  {
    CloudType::Ptr vis_cloud(new CloudType);
    if(indices != nullptr){
      vis_vg_.setInputCloud(cloud);
      vis_vg_.setIndices(indices);
      vis_vg_.filter(*vis_cloud);
    }
    else{
      *vis_cloud = *cloud;
    }

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*vis_cloud, cloud_msg);
    cloud_msg.header.frame_id = frame_id;
    pub.publish(cloud_msg);
  }
  
  #ifdef PUBLISH_GRID_MAP
  void publishGridMap(const grid_map::GridMap &grid, const std::vector<std::string> &publish_layer, ros::Publisher &pub)
  {
    grid_map::GridMap vis_grid;
    vis_grid.setFrameId(grid.getFrameId());
    vis_grid.setGeometry(grid.getLength(), grid.getResolution(), grid.getPosition());
    for(int i = 0; i < publish_layer.size(); i++){
      if(grid.exists(publish_layer[i])){
        vis_grid.add(publish_layer[i], grid[publish_layer[i]]);
      }
    }

    grid_map_msgs::GridMap grid_msg;
    grid_map::GridMapRosConverter::toMessage(vis_grid, grid_msg);
    pub.publish(grid_msg);
  }
  #endif

  CloudType::Ptr gridMap2Cloud(const grid_map::GridMap &grid, const std::string &layer_name, const std::string &intensity_layer_name)
  {
    CloudType::Ptr res(new CloudType);
    if(!grid.exists(layer_name))
      return res;
    const grid_map::Matrix &layer = grid[layer_name];

    const grid_map::Matrix *intensity_layer_ptr = nullptr;
    if(grid.exists(intensity_layer_name))
      intensity_layer_ptr = &(grid[intensity_layer_name]);

    res->reserve(grid.getSize()[0] * grid.getSize()[1]);
    for(grid_map::GridMapIterator it(grid); !it.isPastEnd(); ++it) {
      int i = it.getLinearIndex();
      float z = layer(i);
      if(!std::isfinite(z))
        continue;

      grid_map::Position pos;
      grid.getPosition(*it, pos);

      PointType p;
      p.x = pos[0];
      p.y = pos[1];
      p.z = z;

      if(intensity_layer_ptr != nullptr){
        float intensity = (*intensity_layer_ptr)(i);
        if(std::isfinite(intensity))
          p.intensity = intensity;
        else
          p.intensity = 0;
      }

      res->push_back(p);
    }

    return res;
  }

  bool saveCloud(const std::string &filename, const CloudType &cloud, const PIndices::ConstPtr indices_ptr = nullptr)
  {
    try{
      pcl::PCDWriterWithIndices writer;
      if(indices_ptr == nullptr){
        PIndices dummy_indices;
        dummy_indices.indices.resize(cloud.size());
        for(int i = 0; i < cloud.size(); i++)
          dummy_indices.indices[i] = i;
        writer.writeBinaryWithIndices(filename, cloud, dummy_indices);
      }
      else{
        writer.writeBinaryWithIndices(filename, cloud, *indices_ptr);
      }
      return true;
    }
    catch( ... ){
      return false;
    }
  }

  void initialize()
  {
    nh_ = ros::NodeHandle();
    nh_p_ = ros::NodeHandle("~");
    
    pub_ground_ = nh_.advertise<sensor_msgs::PointCloud2>("ground_cloud", 1, true);
    pub_static_ = nh_.advertise<sensor_msgs::PointCloud2>("static_cloud", 1, true);
    pub_dynamic_ = nh_.advertise<sensor_msgs::PointCloud2>("dynamic_cloud", 1, true);
    pub_ground_below_ = nh_.advertise<sensor_msgs::PointCloud2>("ground_below_cloud", 1, true);
    pub_other_ = nh_.advertise<sensor_msgs::PointCloud2>("other_cloud", 1, true);
    pub_terrain_ = nh_.advertise<sensor_msgs::PointCloud2>("terrain_cloud", 1, true);
    #ifdef PUBLISH_GRID_MAP
    pub_grid_map_ = nh_.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
    #endif
    
    PublisherPtr pub_ptr_in_proc_vis_(new ros::Publisher);
    *pub_ptr_in_proc_vis_ = nh_.advertise<sensor_msgs::PointCloud2>("in_process", 1);

    nh_.param<std::string>("/map_cleaner/frame_id", frame_id_, "map");
    nh_.param<std::string>("/map_cleaner/save_dir", save_dir_, "/tmp");
    if(save_dir_.back() != '/')
      save_dir_ += '/';
    
    float vis_vg_res;
    nh_.param<float>("/map_cleaner/visualization_vg_res", vis_vg_res, 0.8);
    vis_vg_.setLeafSize(vis_vg_res, vis_vg_res, vis_vg_res);

    bool in_process_vis;
    nh_.param<bool>("/map_cleaner/in_process_visualization", in_process_vis, true);

    //Loader
    std::string pcds_dir, pose_file, calibration_file, format;
    int start, end;
    nh_.param<std::string>("/loader/pcds_dir", pcds_dir, "/tmp/pcds");
    if(pcds_dir.back() != '/')
      pcds_dir += '/';
    nh_.param<std::string>("/loader/pose_file", pose_file, "/tmp/poses.csv");
    nh_.param<std::string>("/loader/kitti_calibration_file", calibration_file, "/tmp/calib.txt");
    nh_.param<int>("/loader/start", start, 0);
    nh_.param<int>("/loader/end", end, -1);
    nh_.param<std::string>("/loader/format", format, "erasor");

    if(format == "kitti"){
      loader_.reset(new KittiFormatLoader());
      std::dynamic_pointer_cast<KittiFormatLoader>(loader_)->loadKittiCalibration(calibration_file);
      loader_->loadFrameInfo(pcds_dir, pose_file, start, end);
    }
    else{
      loader_.reset(new ERASORFormatLoader());
      loader_->loadFrameInfo(pcds_dir, pose_file, start, end);
    }
    ROS_INFO_STREAM("Loaded Frames: " << loader_->getSize());

    if(loader_->getSize() == 0)
      exit(1);
    
    //Travel
    bool use_object_seg, use_voxel_grid;
    float seg_voxel_leaf_size;
    nh_.param<bool>("/ground_segmentation/use_object_seg", use_object_seg, true);
    nh_.param<bool>("/ground_segmentation/use_voxel_grid", use_voxel_grid, false);
    nh_.param<float>("/ground_segmentation/voxel_leaf_size", seg_voxel_leaf_size, 0.1);
    TravelGroundSegPtr ground_seg;
    ObjectClusterPtr object_seg;
    buildTravel(nh_, ground_seg, object_seg);
    if(!use_object_seg)
      object_seg = nullptr;

    if(in_process_vis)
      ground_seg_.reset(new GroundSegmentation(ground_seg, object_seg, 
                                               use_voxel_grid, seg_voxel_leaf_size, 
                                               pub_ptr_in_proc_vis_, frame_id_));
    else
      ground_seg_.reset(new GroundSegmentation(ground_seg, object_seg, 
                                               use_voxel_grid, seg_voxel_leaf_size));

    //GridMapBuilder
    nh_.param<float>("/grid_map_builder/grid_res", resolution_, 0.1);
    grid_map_builder_.reset(new GridMapBuilder(resolution_, frame_id_));

    //VarianceFilter
    float variance_th;
    int variance_min_num;
    nh_.param<float>("/variance_filter/variance_th", variance_th, 0.01);
    nh_.param<int>("/variance_filter/min_num", variance_min_num, 3);
    variance_filter_.reset(new VarianceFilter(variance_th, variance_min_num));
    
    //First BGKFilter
    double bgk1_kernel_radius, bgk1_lambda, bgk1_sigma;
    int bgk1_min_num;
    nh_.param<double>("/first_bgk/kernel_radius", bgk1_kernel_radius, 1.0);
    nh_.param<double>("/first_bgk/lambda", bgk1_lambda, 0.0);
    nh_.param<double>("/first_bgk/sigma", bgk1_sigma, 1.0);
    nh_.param<int>("/first_bgk/min_num", bgk1_min_num, 3);
    first_bgk_filter_.reset(
      new BGKFilter(bgk1_kernel_radius, bgk1_lambda, bgk1_sigma, bgk1_min_num, 
                         "variance_filtered", "first_bgk_filtered"));

    //TrajectoryFilter
    double normal_th_deg;
    nh_.param<double>("/trajectory_filter/normal_th_deg", normal_th_deg, 15.0);
    trajectory_filter_.reset(
      new TrajectoryFilter((normal_th_deg / 180.0) * M_PI, 
                           "first_bgk_filtered", "trajectory_filtered"));

    //Second BGKFilter
    double bgk2_kernel_radius, bgk2_lambda, bgk2_sigma;
    int bgk2_min_num;
    nh_.param<double>("/second_bgk/kernel_radius", bgk2_kernel_radius, 1.0);
    nh_.param<double>("/second_bgk/lambda", bgk2_lambda, 0.0);
    nh_.param<double>("/second_bgk/sigma", bgk2_sigma, 1.0);
    nh_.param<int>("/second_bgk/min_num", bgk2_min_num, 3);
    second_bgk_filter_.reset(
      new BGKFilter(bgk2_kernel_radius, bgk2_lambda, bgk2_sigma, bgk2_min_num, 
                         "trajectory_filtered", "second_bgk_filtered"));

    //MedianFilter
    bool enable_median;
    int median_kernel_size;
    nh_.param<bool>("/median/enable", enable_median, true);
    nh_.param<int>("/median/kernel_size", median_kernel_size, 3);
    if(enable_median){
      median_filter_.reset(
        new MedianFilter(median_kernel_size,
                        "second_bgk_filtered", "elevation"));
    }
    else{
      median_filter_ = nullptr;
    }
    
    //DivideByTerrain
    bool on_terrain_only;
    double terrain_th;
    nh_.param<bool>("/divide_by_terrain/on_terrain_only", on_terrain_only, false);
    nh_.param<double>("/divide_by_terrain/threshold", terrain_th, 0.1);
    divide_by_terrain_.reset(
      new DivideByTerrain(terrain_th, on_terrain_only,
                          "elevation"));

    //MovingObjectIdentification
    double voxel_leaf_size, fov_h, fov_v, res_h, res_v, lidar_range, range_distance_th, submap_update_dist;
    int delta_h, delta_v, frame_skip;
    nh_.param<double>("/moving_point_identification/voxel_leaf_size", voxel_leaf_size, 0.2);
    nh_.param<double>("/moving_point_identification/fov_h_deg", fov_h, 360.0);
    fov_h = (fov_h / 180.0) * M_PI;
    nh_.param<double>("/moving_point_identification/fov_v_deg", fov_v, 90.0);
    fov_v = (fov_v / 180.0) * M_PI;
    nh_.param<double>("/moving_point_identification/res_h_deg", res_h, 2.84444);
    res_h = (res_h / 180.0) * M_PI;
    nh_.param<double>("/moving_point_identification/res_v_deg", res_v, 1.40625);
    res_v = (res_v / 180.0) * M_PI;
    nh_.param<double>("/moving_point_identification/lidar_range", lidar_range, 100.0);
    nh_.param<int>("/moving_point_identification/delta_h", delta_h, 2);
    nh_.param<int>("/moving_point_identification/delta_v", delta_v, 1);
    nh_.param<double>("/moving_point_identification/threshold", range_distance_th, 0.5);
    nh_.param<int>("/moving_point_identification/frame_skip", frame_skip, 0);
    nh_.param<double>("/moving_point_identification/submap_update_dist", submap_update_dist, 3.0);

    if(in_process_vis){
      moving_point_identification_.reset(
        new MovingPointIdentification(voxel_leaf_size, res_h, res_v, fov_h, fov_v, lidar_range, 
                                      delta_h, delta_v, range_distance_th, frame_skip, submap_update_dist,
                                      pub_ptr_in_proc_vis_, frame_id_));
    }
    else{
      moving_point_identification_.reset(
        new MovingPointIdentification(voxel_leaf_size, res_h, res_v, fov_h, fov_v, lidar_range, 
                                      delta_h, delta_v, range_distance_th, frame_skip, submap_update_dist));
    }

    ROS_INFO_STREAM("pcds_dir: " << pcds_dir);
    ROS_INFO_STREAM("pose_file: " << pose_file);
    ROS_INFO_STREAM("save_dir: " << save_dir_);
  }

public:
  MapCleaner()
  {
    initialize();
  }

  void exec()
  {
    CloudType::Ptr cloud(new CloudType);
    PIndices::Ptr initial_ground_indices(new PIndices);
    PIndices::Ptr nonground_indices(new PIndices);
    ground_seg_->compute(loader_, *cloud, *initial_ground_indices, *nonground_indices); 
    ROS_INFO_STREAM("Finished: GroundSegmentation");

    grid_map::GridMapPtr grid_map_ptr = grid_map_builder_->compute(*cloud, *initial_ground_indices);
    if(grid_map_ptr == nullptr){
      ROS_ERROR_STREAM("Failed: ground_cloud is empty.");
      return;
    }
    ROS_INFO_STREAM("Finished: GridMapBuilder");
  
    if(!variance_filter_->compute(*grid_map_ptr)){
      return ;
    }
    ROS_INFO_STREAM("Finished: VarianceFilter");
    
    if(!first_bgk_filter_->compute(*grid_map_ptr)){
      return ;
    }
    ROS_INFO_STREAM("Finished: First BGKFilter");
    
    if(!trajectory_filter_->compute(*grid_map_ptr, loader_)){
      return ;
    }
    ROS_INFO_STREAM("Finished: TrajectoryFilter");
    
    if(!second_bgk_filter_->compute(*grid_map_ptr)){
      return ;
    }
    ROS_INFO_STREAM("Finished: Second BGKFilter");

    if(median_filter_ != nullptr){
      if(!median_filter_->compute(*grid_map_ptr)){
        return ;
      }
      ROS_INFO_STREAM("Finished: MedianFilter");
    }
    else{
      grid_map_ptr->add("elevation", (*grid_map_ptr)["second_bgk_filtered"]);
      grid_map_ptr->erase("second_bgk_filtered");
    }
    
    #ifdef PUBLISH_GRID_MAP
    publishGridMap(*grid_map_ptr, {"elevation", "intensity"}, pub_grid_map_);
    #endif

    CloudType::Ptr terrain_pcd = gridMap2Cloud(*grid_map_ptr, "elevation", "intensity");
    publishCloud(terrain_pcd, nullptr, frame_id_, pub_terrain_);
    
    PIndices::Ptr ground_indices(new PIndices);
    PIndices::Ptr ground_above_indices(new PIndices);
    PIndices::Ptr ground_below_indices(new PIndices);
    PIndices::Ptr other_indices(new PIndices);
    if(!divide_by_terrain_->compute(*cloud, *initial_ground_indices, *nonground_indices, 
                                    *grid_map_ptr,
                                    *ground_indices, *ground_above_indices, *ground_below_indices, *other_indices)){
      return ;
    }
    ROS_INFO_STREAM("Finished: Divide By Terrain");
    initial_ground_indices->indices.clear(); //release memory
    initial_ground_indices->indices.shrink_to_fit();
    nonground_indices->indices.clear();
    nonground_indices->indices.shrink_to_fit();

    PIndices::Ptr static_indices(new PIndices);
    PIndices::Ptr dynamic_indices(new PIndices);
    if(!moving_point_identification_->compute(loader_, cloud, ground_above_indices, *static_indices, *dynamic_indices)){
      return ;
    }
    ROS_INFO_STREAM("Finished: Moving Point Identification");

    publishCloud(cloud, ground_indices, frame_id_, pub_ground_);
    publishCloud(cloud, static_indices, frame_id_, pub_static_);
    publishCloud(cloud, dynamic_indices, frame_id_, pub_dynamic_);
    publishCloud(cloud, ground_below_indices, frame_id_, pub_ground_below_);
    publishCloud(cloud, other_indices, frame_id_, pub_other_);

    saveCloud(save_dir_ + "terrain.pcd", *terrain_pcd);
    saveCloud(save_dir_ + "static.pcd", *cloud, static_indices);
    saveCloud(save_dir_ + "dynamic.pcd", *cloud, dynamic_indices);
    saveCloud(save_dir_ + "ground_below.pcd", *cloud, ground_below_indices);
    saveCloud(save_dir_ + "ground.pcd", *cloud, ground_indices);
    saveCloud(save_dir_ + "other.pcd", *cloud, other_indices);
    
    ROS_INFO_STREAM("Finished: Save Cloud");

    ros::Rate rate(100);
    while(ros::ok())
    {
      ros::spinOnce();
      rate.sleep();
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_cleaner");
  MapCleaner map_cleaner;
  map_cleaner.exec();

  return 0;
}
